#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge, CvBridgeError

class ErrorCalculator:
    def __init__(self):
        rospy.loginfo("Starting Orange Error Publisher node (v2)...")
        
        self.bridge = CvBridge()

        # --- Constants ---
        # Widen the S and V ranges to be more robust to lighting.
        # We're now allowing saturation and value to go as low as 60.
        self.LOWER_ORANGE = np.array([5, 100, 100])
        self.UPPER_ORANGE = np.array([20, 255, 255])
        self.MIN_CONTOUR_AREA = 300

        # --- Control Target Constants (Tunable) ---
        self.S_TARGET = 150.0 

        # --- Morphological Kernel ---
        # We'll use this for cleaning up the mask
        self.kernel = np.ones((5, 5), np.uint8)
        
        # We need the image width to calculate the center
        self.image_width = 0
        self.image_center_u = 0

        # ROS Publishers
        self.error_pub = rospy.Publisher('/control_errors', Vector3, queue_size=10)
        
        # Publishes the *original* frame + bounding box
        self.debug_image_pub = rospy.Publisher('/orange_detector/debug_image', Image, queue_size=10)
        
        # --- NEW DEBUG PUBLISHER ---
        # Publishes the *mask* + bounding box
        self.mask_pub = rospy.Publisher('/orange_detector/debug_mask', Image, queue_size=10)

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

        # --- 1. Create the Mask ---
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        orange_mask = cv2.inRange(hsv_frame, self.LOWER_ORANGE, self.UPPER_ORANGE)

        # --- 2. Robust Morphology ---
        # First, "Open" the mask: Erode then Dilate.
        # This removes small white specks (false positives) in the background.
        mask_opened = cv2.morphologyEx(orange_mask, cv2.MORPH_OPEN, self.kernel, iterations=2)
        
        # Second, "Close" the mask: Dilate then Erode.
        # This fills in small black holes (false negatives) *inside* the main object.
        mask_closed = cv2.morphologyEx(mask_opened, cv2.MORPH_CLOSE, self.kernel, iterations=2)

        # --- 3. Find Contours ---
        # Find contours on the *final, cleaned* mask
        contours, _ = cv2.findContours(mask_closed.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # --- 4. Find the Largest Orange Object ---
        best_contour = None
        max_area = 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > self.MIN_CONTOUR_AREA and area > max_area:
                max_area = area
                best_contour = cnt

        # --- 5. Prepare Debug Images ---
        # Create a BGR version of the final mask to draw on
        debug_mask_bgr = cv2.cvtColor(mask_closed, cv2.COLOR_GRAY2BGR)

        # --- 6. Calculate Errors ---
        error_msg = Vector3()
        
        if best_contour is not None:
            # We found the ball!
            x, y, w, h = cv2.boundingRect(best_contour)
            
            # Calculate centroid and size
            centroid_u = x + (w / 2.0)
            size_s = h # Use height for size
            
            # Calculate errors
            e_u = centroid_u - self.image_center_u
            e_s = self.S_TARGET - size_s
            
            # Populate the error message
            error_msg.x = e_u
            error_msg.y = e_s
            error_msg.z = 1.0  # Flag to show the ball is found
            
            # --- Draw on both debug frames ---
            # Green box on the original frame
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
            cv2.circle(frame, (int(centroid_u), int(y + h / 2.0)), 5, (0, 0, 255), -1)
            
            # Blue box on the mask frame
            cv2.rectangle(debug_mask_bgr, (x, y), (x + w, y + h), (255, 0, 0), 2)
            
        else:
            # No ball found
            error_msg.x = 0.0
            error_msg.y = 0.0
            error_msg.z = 0.0 # Flag to show the ball is NOT found

        # --- 7. Publish Errors and Debug Images ---
        self.error_pub.publish(error_msg)
        
        try:
            # Publish the original frame + box
            debug_ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.debug_image_pub.publish(debug_ros_image)
            
            # Publish the mask frame + box
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