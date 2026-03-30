#!/usr/bin/env python3

import sys
import os
import json
import numpy as np
import cv2
import rospy
import rospkg
import threading
import time
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from ultralytics import YOLO

class ObjectDetector:
    def __init__(self):
        rospy.init_node('object_detector', anonymous=True)
        
        # 1. Parameters & Path
        self.enable_vis = rospy.get_param('~enable_vis', True)
        self.bridge = CvBridge()

        # Get package path using rospkg
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('mren_waste_collector')
        self.model_path = os.path.join(package_path, 'models', 'segmentation_small_openvino_model')
        
        rospy.loginfo(f"Loading Model: {self.model_path}")
        try:
            self.model = YOLO(self.model_path, task="segment")
            rospy.loginfo("✅ YOLO Model loaded successfully.")
        except Exception as e:
            rospy.logerr(f"❌ Failed to load YOLO Model: {e}")
            raise e

        # 2. Publishers
        self.publisher_ = rospy.Publisher('detected_objects', String, queue_size=10)
        self.debug_pub_ = rospy.Publisher('debug_image', Image, queue_size=10) 

        # 3. Subscribers
        self.latest_colour = None
        self.latest_depth = None
        self.latest_intrinsics = None
        self.latest_depth_scale = None
        self.data_lock = threading.Lock()

        rospy.Subscriber('camera_frames', Image, self.color_callback)
        rospy.Subscriber('depth_frames', Image, self.depth_callback)
        rospy.Subscriber('camera_info', CameraInfo, self.info_callback)
        rospy.Subscriber('depth_scale', Float32, self.scale_callback)

        # ROS 1 Timer
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        self.last_timer_time = time.time()
        rospy.loginfo("✅ Node Initialized and Running. Subscribed to camera topics.")

    def color_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self.data_lock:
                self.latest_colour = cv_image
        except Exception as e:
            rospy.logerr(f"Color callback error: {e}")

    def depth_callback(self, msg):
        try:
            # Depth is 16-bit unsigned (16UC1)
            cv_depth = self.bridge.imgmsg_to_cv2(msg, "16UC1")
            with self.data_lock:
                self.latest_depth = cv_depth
        except Exception as e:
            rospy.logerr(f"Depth callback error: {e}")

    def info_callback(self, msg):
        with self.data_lock:
            self.latest_intrinsics = msg

    def scale_callback(self, msg):
        with self.data_lock:
            self.latest_depth_scale = msg.data

    def timer_callback(self, event):
        # Refresh parameter value in case it changed
        show_debug = rospy.get_param('~enable_vis', self.enable_vis)

        with self.data_lock:
            if self.latest_colour is None or self.latest_depth is None or self.latest_intrinsics is None or self.latest_depth_scale is None:
                return
            frame = self.latest_colour.copy()
            depth_img = self.latest_depth.copy()
            intrinsics = self.latest_intrinsics
            depth_scale = self.latest_depth_scale
            
        # Log periodic status
        current_time = time.time()
        if current_time - self.last_timer_time > 5.0:
            rospy.loginfo("Processing loop active. Receiving data from topics.")
            self.last_timer_time = current_time

        H, W = frame.shape[:2]
        debug_frame = frame.copy() if show_debug else None

        results = self.model.predict(frame, imgsz=640, conf=0.75, verbose=False)
        r = results[0]
        detected_list = []

        if r.masks is not None:
            for mask_points in r.masks.xy:
                c = mask_points.astype(np.int32)
                if len(c) < 5: continue 

                ellipse = cv2.fitEllipse(c)
                (cx, cy), (MA, ma), angle = ellipse
                
                cx_int = max(0, min(int(cx), W - 1))
                cy_int = max(0, min(int(cy), H - 1))

                # Calculate distance in meters from depth map
                dist_raw = depth_img[cy_int, cx_int]
                distance_meters = dist_raw * depth_scale
                
                if distance_meters <= 0: continue 

                # Use K[0] for fx and K[4] for fy (row-major: K[0,1,2, 3,4,5, 6,7,8])
                fx = intrinsics.K[0]
                fy = intrinsics.K[4]

                real_width_cm = (ma * distance_meters / fx) * 100
                real_length_cm = (MA * distance_meters / fy) * 100
                
                # Angle Logic
                if angle > 90:
                    angle_major = angle - 180
                else:
                    angle_major = angle
                angle_major = -angle_major

                if show_debug:
                    cv2.drawContours(debug_frame, [c], -1, (0, 255, 0), 2)
                    cv2.circle(debug_frame, (cx_int, cy_int), 5, (0, 0, 255), -1)
                    labels = [
                        f"Dist: {distance_meters:.2f}m",
                        f"Size: {real_width_cm:.1f}x{real_length_cm:.1f}cm",
                        f"Ang: {angle_major:.1f}deg",
                    ]
                    text_x = min(cx_int + 15, W - 160)
                    text_y = cy_int
                    for line in labels:
                        cv2.putText(debug_frame, line, (text_x, text_y), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
                        text_y += 18

                obj_data = {
                    "x_pos": float(cx), "y_pos": float(cy),
                    "dist_meters": float(distance_meters),
                    "width_cm": float(real_width_cm),
                    "length_cm": float(real_length_cm),
                    "angle": float(angle_major)
                }
                detected_list.append(obj_data)

        # Publish JSON string
        self.publisher_.publish(String(data=json.dumps(detected_list)))

        # Publish Debug Image
        if show_debug and debug_frame is not None:
            self.debug_pub_.publish(self.bridge.cv2_to_imgmsg(debug_frame, "bgr8"))

    def stop(self):
        pass

if __name__ == '__main__':
    try:
        detector = ObjectDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        if 'detector' in locals():
            detector.stop()