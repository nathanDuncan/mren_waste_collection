#!/usr/bin/env python3

import sys
import os
import json
import numpy as np
import cv2
import rospy
import threading
import time
import socket
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

# Import generated Protobuf
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.append(script_dir)

try:
    import detection_pb2
except ImportError:
    rospy.logerr("❌ detection_pb2.py not found. Protobuf transmission will fail.")

class OrangeDetector:
    def __init__(self):
        rospy.init_node('orange_detector', anonymous=True)
        
        # 0. Debug Environment
        rospy.loginfo("--- Orange Detector: Initializing ---")
        rospy.loginfo(f"ROS_MASTER_URI: {os.environ.get('ROS_MASTER_URI')}")
        rospy.loginfo(f"Full Node Name: {rospy.get_name()}")
        
        # 1. Parameters & Constants
        self.enable_vis = rospy.get_param('~enable_vis', True)
        self.bridge = CvBridge()
        
        # Color Masking Constants
        self.LOWER_ORANGE = np.array([5, 60, 60])
        self.UPPER_ORANGE = np.array([25, 255, 255])
        self.MIN_CONTOUR_AREA = 500
        self.kernel = np.ones((5, 5), np.uint8)

        # 2. Publishers
        self.publisher_ = rospy.Publisher('detected_objects', String, queue_size=10)
        self.debug_image_pub = rospy.Publisher('/orange_detector/debug_image', Image, queue_size=10)
        self.mask_pub = rospy.Publisher('/orange_detector/debug_mask', Image, queue_size=10)

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

        # UDP Setup for Protobuf transmission
        self.target_ip = "192.168.12.128"
        self.target_port = 25006
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        rospy.loginfo(f"✅ Orange Detector Initialized. Sending Protobuf to {self.target_ip}:{self.target_port}")

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
            
        H, W = frame.shape[:2]
        debug_frame = frame.copy() if show_debug else None

        # --- Detection Logic (Orange Masking) ---
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        orange_mask = cv2.inRange(hsv_frame, self.LOWER_ORANGE, self.UPPER_ORANGE)
        mask_opened = cv2.morphologyEx(orange_mask, cv2.MORPH_OPEN, self.kernel, iterations=2)
        mask_closed = cv2.morphologyEx(mask_opened, cv2.MORPH_CLOSE, self.kernel, iterations=2)
        contours, _ = cv2.findContours(mask_closed.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected_list = []

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.MIN_CONTOUR_AREA:
                continue

            # Process the contour to find size and orientation
            if len(cnt) < 5:
                # Cannot fit ellipse, use bounding box as fallback for centroid
                x, y, w, h = cv2.boundingRect(cnt)
                cx, cy = x + w/2, y + h/2
                MA, ma, angle = w, h, 0 
                c = cnt
            else:
                ellipse = cv2.fitEllipse(cnt)
                (cx, cy), (MA, ma), angle = ellipse
                c = cnt

            cx_int = max(0, min(int(cx), W - 1))
            cy_int = max(0, min(int(cy), H - 1))

            # Calculate distance in meters from depth map
            dist_raw = depth_img[cy_int, cx_int]
            distance_meters = dist_raw * depth_scale
            
            if distance_meters <= 0:
                continue 

            # Use K[0] for fx and K[4] for fy
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
                "angle": float(angle_major),
                "area": float(area)
            }
            detected_list.append(obj_data)

        # 4. Select largest object and send via Protobuf
        if detected_list:
            # Find the object with the greatest contour area
            largest_obj = max(detected_list, key=lambda x: x['area'])
            
            # Construct Protobuf message
            try:
                frame_pb = detection_pb2.DetectionFrame()
                obj_pb = frame_pb.objects.add()
                obj_pb.x_pos = largest_obj['x_pos']
                obj_pb.y_pos = largest_obj['y_pos']
                obj_pb.dist_meters = largest_obj['dist_meters']
                obj_pb.width_cm = largest_obj['width_cm']
                obj_pb.length_cm = largest_obj['length_cm']
                obj_pb.angle = largest_obj['angle']

                # Send via UDP
                serialized_data = frame_pb.SerializeToString()
                self.sock.sendto(serialized_data, (self.target_ip, self.target_port))
            except Exception as e:
                rospy.logerr(f"❌ Protobuf Sending Error: {e}")

        # Publish JSON string
        self.publisher_.publish(String(data=json.dumps(detected_list)))

        # Publish Debug Image & Mask
        if show_debug and debug_frame is not None:
            self.debug_image_pub.publish(self.bridge.cv2_to_imgmsg(debug_frame, "bgr8"))
            
            # Create a BGR mask for visualization
            debug_mask_bgr = cv2.cvtColor(mask_closed, cv2.COLOR_GRAY2BGR)
            self.mask_pub.publish(self.bridge.cv2_to_imgmsg(debug_mask_bgr, "bgr8"))
            
            cv2.imshow("Orange Detection Debug", debug_frame)
            cv2.waitKey(1)
        elif not show_debug:
            cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        detector = OrangeDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
