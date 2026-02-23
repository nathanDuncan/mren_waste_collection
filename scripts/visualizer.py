#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import math
from std_msgs.msg import Float32MultiArray

class VisualizerNode:
    def __init__(self):
        rospy.init_node('visualizer_node')
        
        # Camera parameters
        self.width = rospy.get_param('~width', 640)
        self.height = rospy.get_param('~height', 480)
        self.center_x = rospy.get_param('~center_x', self.width / 2.0)
        self.center_y = rospy.get_param('~center_y', self.height / 2.0)
        self.scale = rospy.get_param('~length_scale', 5.0) # pixels per cm
        
        # Subscriber for camera data
        self.sub = rospy.Subscriber('/camera_data', Float32MultiArray, self.callback)
        
        # State
        self.current_data = None
        
        rospy.loginfo("✅ Visualizer Node started. Subscribed to /camera_data")

    def callback(self, msg):
        # Expected format: [x_pos, y_pos, dist_meters, width_cm, length_cm, angle]
        if len(msg.data) >= 6:
            self.current_data = msg.data

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            # Create a black background
            img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
            
            # Draw red rectangle representing camera frame (border)
            cv2.rectangle(img, (0, 0), (self.width - 1, self.height - 1), (0, 0, 255), 2)
            
            # Draw center dot (blue)
            cv2.circle(img, (int(self.center_x), int(self.center_y)), 5, (255, 0, 0), -1)
            
            # Draw object if data is available
            if self.current_data:
                x_pos = self.current_data[0]
                y_pos = self.current_data[1]
                length_cm = self.current_data[4]
                angle = self.current_data[5] # degrees
                
                # Origin (x_pos, y_pos) - assuming these are in pixel coordinates
                start_point = (int(x_pos), int(y_pos))
                
                # Calculate end point
                # Angle from vertical (negative Y direction)
                # x_end = x_pos + length * sin(angle)
                # y_end = y_pos - length * cos(angle)
                length_px = length_cm * self.scale
                end_x = x_pos + length_px * math.sin(math.radians(angle))
                end_y = y_pos - length_px * math.cos(math.radians(angle))
                end_point = (int(end_x), int(end_y))
                
                # Draw the line (green)
                cv2.line(img, start_point, end_point, (0, 255, 0), 3)
                
                # Draw a small dot at the start point (yellow)
                cv2.circle(img, start_point, 3, (0, 255, 255), -1)
                
                # Display text info
                cv2.putText(img, f"Dist: {self.current_data[2]:.2f}m", (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(img, f"Angle: {angle:.1f}deg", (10, 60), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            # Show the image
            cv2.imshow("Waste Collector Visualizer", img)
            
            # WaitKey is necessary for OpenCV windows to update
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
            rate.sleep()
        
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        node = VisualizerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
