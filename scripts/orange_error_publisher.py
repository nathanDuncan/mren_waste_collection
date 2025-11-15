#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge, CvBridgeError

# --- Constants ---
LOWER_ORANGE = np.array([5, 110, 110])
UPPER_ORANGE = np.array([25, 255, 255])
MIN_CONTOUR_AREA = 400

# --- Control Target Constants (Tunable) ---
# We want the ball to be 150 pixels high in the frame
S_TARGET = 150.0 

class ErrorCalculator:
    def __init__(self):
        rospy.loginfo("Starting Orange Error Publisher node...")
        
        self.bridge = CvBridge()
        
        # We need the image width to calculate the center
        self.image_width = 0
        self.image_center_u = 0

        # ROS Publishers
        self.error_pub = rospy.Publisher('/control_errors', Vector3, queue_size=10)
        
        # This publisher is for debugging. We'll publish the frame with
        # the bounding box drawn on it.
        self.debug_image_pub = rospy.Publisher('/orange_detector/debug_image', Image, queue_size=10)

        # ROS Subscriber
        self.image_sub = rospy.Subscriber('/camera_frames', Image, self.image_callback)

    def image_callback(self, ros_image):
        try:
            # Convert the ROS Image message to an OpenCV image (BGR)
            frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # --- Get Image Dimensions (run once) ---
        if self.image_width == 0:
            try:
                self.image_height, self.image_width, _ = frame.shape
                self.image_center_u = self.image_width / 2.0
                rospy.loginfo(f"Image dimensions set: {self.image_width} x {self.image_height}")
            except Exception as e:
                rospy.logerr(f"Failed to get image shape: {e}")
                return

        # --- 1. Run the Orange Detector Logic ---
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        orange_mask = cv2.inRange(hsv_frame, LOWER_ORANGE, UPPER_ORANGE)
        orange_mask = cv2.erode(orange_mask, None, iterations=2)
        orange_mask = cv2.dilate(orange_mask, None, iterations=2)
        contours, _ = cv2.findContours(orange_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # --- 2. Find the Largest Orange Object ---
        best_contour = None
        max_area = 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > MIN_CONTOUR_AREA and area > max_area:
                max_area = area
                best_contour = cnt

        # --- 3. Calculate Errors ---
        error_msg = Vector3()
        
        if best_contour is not None:
            # We found the ball!
            x, y, w, h = cv2.boundingRect(best_contour)
            
            # Calculate centroid and size
            centroid_u = x + (w / 2.0)
            size_s = h # Use height for size
            
            # Calculate errors
            e_u = centroid_u - self.image_center_u
            e_s = S_TARGET - size_s
            
            # Populate the error message
            error_msg.x = e_u
            error_msg.y = e_s
            error_msg.z = 1.0  # Flag to show the ball is found
            
            # --- Draw on the debug frame ---
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
            cv2.circle(frame, (int(centroid_u), int(y + h / 2.0)), 5, (0, 0, 255), -1)
            
        else:
            # No ball found
            error_msg.x = 0.0
            error_msg.y = 0.0
            error_msg.z = 0.0 # Flag to show the ball is NOT found

        # --- 4. Publish Errors and Debug Image ---
        self.error_pub.publish(error_msg)
        
        try:
            # Publish the debug frame
            debug_ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.debug_image_pub.publish(debug_ros_image)
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