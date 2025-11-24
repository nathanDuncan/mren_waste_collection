#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import os
import time
from sensor_msgs.msg import Image
from geometry_msgs.msg import Quaternion
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO

# --- Constants ---
S_TARGET = 150.0 

class CanErrorPublisher:
    def __init__(self):
        rospy.init_node('can_error_publisher', anonymous=True)
        
        self.bridge = CvBridge()
        
        # --- Load Model ---
        script_dir = os.path.dirname(os.path.realpath(__file__))
        
        # Check for OpenVINO export
        openvino_path = os.path.join(script_dir, "best_openvino_model")
        onnx_path = os.path.join(script_dir, "best.onnx")
        pt_path = os.path.join(script_dir, "best.pt")
        
        try:
            if os.path.exists(openvino_path):
                rospy.loginfo(f"🚀 Loading OpenVINO model: {openvino_path}")
                self.model = YOLO(openvino_path)
            elif os.path.exists(onnx_path):
                rospy.loginfo(f"🚀 Loading ONNX model: {onnx_path}")
                self.model = YOLO(onnx_path)
            else:
                rospy.loginfo(f"Loading PyTorch model: {pt_path}")
                self.model = YOLO(pt_path)
            rospy.loginfo("YOLO model loaded successfully.")
        except Exception as e:
            rospy.logerr(f"Failed to load model: {e}")
            exit()

        # --- Image Stats ---
        self.image_center_u = 0
        self.image_center_v = 0
        self.initialized = False
        
        # --- Performance Stats ---
        self.prev_time = time.time()
        self.frame_count = 0
        self.processing = False # Flag to drop frames if busy

        # --- Publishers ---
        self.error_pub = rospy.Publisher('/control_errors', Quaternion, queue_size=1)
        self.debug_pub = rospy.Publisher('/yolo/debug_image', Image, queue_size=1)

        # --- Subscriber ---
        self.image_sub = rospy.Subscriber(
            '/camera_frames', 
            Image, 
            self.image_callback, 
            queue_size=1, 
            buff_size=2**24
        )

    def image_callback(self, ros_image):
        # 1. Latency Check: If we are already processing a frame, DROP this one.
        if self.processing:
            return
        
        # 2. Latency Check: If the message is too old, DROP it.
        # This handles network/buffer lag.
        # Note: This requires the sender (realsense_publisher) to stamp messages correctly.
        # If timestamps are 0, this check is skipped.
        if ros_image.header.stamp.to_sec() > 0:
            delay = rospy.Time.now().to_sec() - ros_image.header.stamp.to_sec()
            if delay > 0.1: # Drop if older than 100ms
                return

        self.processing = True
        start_time = time.time()

        try:
            frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            self.processing = False
            return

        if not self.initialized:
            h, w = frame.shape[:2]
            self.image_center_u = w / 2.0
            self.image_center_v = h / 2.0
            self.initialized = True

        # --- Inference ---
        # Run at native resolution (640) for speed
        results = self.model(frame, verbose=False, conf=0.65, imgsz=640) 
        
        # --- Calculate FPS ---
        self.frame_count += 1
        if self.frame_count % 10 == 0: # Print every 10 frames
            curr_time = time.time()
            fps = 10 / (curr_time - self.prev_time)
            self.prev_time = curr_time
            rospy.loginfo_throttle(1, f"Inference Speed: {fps:.2f} FPS")

        # --- Find Largest Detection ---
        best_box = None
        max_area = 0
        best_conf = 0.0
        
        if len(results) > 0 and len(results[0].boxes) > 0:
            for box in results[0].boxes:
                coords = box.xyxy[0].cpu().numpy() 
                x1, y1, x2, y2 = int(coords[0]), int(coords[1]), int(coords[2]), int(coords[3])
                
                width = x2 - x1
                height = y2 - y1
                area = width * height
                
                if area > 500 and area > max_area:
                    max_area = area
                    best_box = coords
                    best_conf = float(box.conf[0])

        # --- Calculate Errors ---
        error_msg = Quaternion()

        if best_box is not None:
            x1, y1, x2, y2 = int(best_box[0]), int(best_box[1]), int(best_box[2]), int(best_box[3])
            
            cx = x1 + (x2 - x1) / 2.0
            cy = y1 + (y2 - y1) / 2.0
            current_h = y2 - y1

            error_msg.x = cx - self.image_center_u
            error_msg.y = cy - self.image_center_v
            error_msg.z = S_TARGET - current_h
            error_msg.w = 1.0

            # Debug Draw
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(frame, (int(cx), int(cy)), 5, (0, 0, 255), -1)
            label = f"Can: {best_conf:.2f}"
            cv2.putText(frame, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
        
        else:
            error_msg.x = 0.0
            error_msg.y = 0.0
            error_msg.z = 0.0
            error_msg.w = 0.0

        self.error_pub.publish(error_msg)
        
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            # Copy timestamp from original message to sync visualization
            debug_msg.header.stamp = ros_image.header.stamp 
            self.debug_pub.publish(debug_msg)
        except CvBridgeError as e:
            rospy.logerr(e)
            
        self.processing = False

def main():
    try:
        CanErrorPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()