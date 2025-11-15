#!/usr/bin/env python3
    
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def main():
    # Initialize the ROS node
    rospy.init_node('webcam_publisher', anonymous=True)

    # Create a publisher for the camera frames
    # The topic is 'camera_frames', message type is Image
    image_pub = rospy.Publisher('camera_frames', Image, queue_size=10)

    # Create a CvBridge object to convert between OpenCV and ROS images
    bridge = CvBridge()

    # --- Camera Setup ---
    # Try to open the default camera (usually index 0)
    # If 0 doesn't work, you might need to try 1 or 2.
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        rospy.logerr("Error: Could not open webcam.")
        return

    # Set the loop rate (e.g., 30 Hz)
    # We'll use this to control the publishing frequency
    rate = rospy.Rate(30) # 30 Hz

    rospy.loginfo("Webcam publisher node started. Publishing to /camera_frames")

    while not rospy.is_shutdown():
        # 1. Read a frame from the camera
        ret, frame = cap.read()
        
        if not ret:
            rospy.logwarn("Failed to grab frame from webcam.")
            continue

        try:
            # 2. Convert the OpenCV image (BGR) to a ROS Image message
            ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")

            # 3. Publish the ROS Image message
            image_pub.publish(ros_image)

        except CvBridgeError as e:
            rospy.logerr(e)
        
        # 4. Sleep to maintain the desired loop rate
        rate.sleep()

    # Clean up when the node is shut down
    rospy.loginfo("Shutting down webcam publisher.")
    cap.release()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass