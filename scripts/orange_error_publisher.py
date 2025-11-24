#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Quaternion # CHANGED: Standardized message type
from cv_bridge import CvBridge, CvBridgeError

class ErrorCalculator:
    def __init__(self):
        rospy.loginfo("Starting Orange Error Publisher node (Standardized)...")
        
        self.bridge = CvBridge()

        # --- Constants ---
        self.LOWER_ORANGE = np.array([5, 60, 60])
        self.UPPER_ORANGE = np.array([25, 255, 255])
        self.MIN_CONTOUR_AREA = 500
        self.S_TARGET = 150.0 

        # --- Morphological Kernel ---
        self.kernel = np.ones((5, 5), np.uint8)
        
        # Image Stats
        self.image_width = 0
        self.image_height = 0
        self.image_center_u = 0
        self.image_center_v = 0

        # ROS Publishers
        # Changed to Quaternion to match YOLO node
        self.error_pub = rospy.Publisher('/control_errors', Quaternion, queue_size=10)
        self.debug_image_pub = rospy.Publisher('/orange_detector/debug_image', Image, queue_size=10)
        self.mask_pub = rospy.Publisher('/orange_detector/debug_mask', Image, queue_size=10)

        # ROS Subscriber
        self.image_sub = rospy.Subscriber('/camera_frames', Image, self.image_callback)

    def image_callback(self, ros_image):
        try:
            frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        if self.image_width == 0:
            try:
                self.image_height, self.image_width, _ = frame.shape
                self.image_center_u = self.image_width / 2.0
                self.image_center_v = self.image_height / 2.0
                rospy.loginfo(f"Image dimensions set: {self.image_width} x {self.image_height}")
            except Exception as e:
                rospy.logerr(f"Failed to get image shape: {e}")
                return

        # --- Processing ---
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        orange_mask = cv2.inRange(hsv_frame, self.LOWER_ORANGE, self.UPPER_ORANGE)
        mask_opened = cv2.morphologyEx(orange_mask, cv2.MORPH_OPEN, self.kernel, iterations=2)
        mask_closed = cv2.morphologyEx(mask_opened, cv2.MORPH_CLOSE, self.kernel, iterations=2)
        contours, _ = cv2.findContours(mask_closed.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best_contour = None
        max_area = 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > self.MIN_CONTOUR_AREA and area > max_area:
                max_area = area
                best_contour = cnt

        debug_mask_bgr = cv2.cvtColor(mask_closed, cv2.COLOR_GRAY2BGR)

        # --- Calculate Errors ---
        error_msg = Quaternion() # Using Quaternion now
        
        if best_contour is not None:
            x, y, w, h = cv2.boundingRect(best_contour)
            
            centroid_u = x + (w / 2.0)
            centroid_v = y + (h / 2.0)
            
            e_u = centroid_u - self.image_center_u
            e_v = centroid_v - self.image_center_v
            e_s = self.S_TARGET - h
            
            # Map to Quaternion fields (same as YOLO node)
            error_msg.x = e_u
            error_msg.y = e_v
            error_msg.z = e_s
            error_msg.w = 1.0  # Flag: Found
            
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
            cv2.circle(frame, (int(centroid_u), int(centroid_v)), 5, (0, 0, 255), -1)
            cv2.rectangle(debug_mask_bgr, (x, y), (x + w, y + h), (255, 0, 0), 2)
            
        else:
            error_msg.x = 0.0
            error_msg.y = 0.0
            error_msg.z = 0.0
            error_msg.w = 0.0 # Flag: Not found

        self.error_pub.publish(error_msg)
        
        try:
            debug_ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.debug_image_pub.publish(debug_ros_image)
            
            debug_mask_image = self.bridge.cv2_to_imgmsg(debug_mask_bgr, "bgr8")
            self.mask_pub.publish(debug_mask_image)

        except CvBridgeError as e:
            rospy.logerr(e)

def main():
    rospy.init_node('orange_error_publisher', anonymous=True)
    ec = ErrorCalculator()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down Orange Error Publisher.")

if __name__ == '__main__':
    main()