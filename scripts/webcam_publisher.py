#!/usr/bin/env python3
    
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError

def main():
    # Initialize the ROS node
    rospy.init_node('webcam_publisher', anonymous=True)

    # --- Publishers ---
    image_pub = rospy.Publisher('camera_frames', Image, queue_size=10)
    depth_pub = rospy.Publisher('depth_frames', Image, queue_size=10)
    info_pub = rospy.Publisher('camera_info', CameraInfo, queue_size=10)
    scale_pub = rospy.Publisher('depth_scale', Float32, queue_size=10)

    # Create a CvBridge object to convert between OpenCV and ROS images
    bridge = CvBridge()

    # --- Camera Setup ---
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        rospy.logerr("Error: Could not open webcam.")
        return

    # Set the loop rate
    rate = rospy.Rate(30) # 30 Hz

    # --- Mock Data Setup ---
    # Assume 640x480 resolution
    WIDTH = 640
    HEIGHT = 480
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)

    # Uniform depth at 1.0m (1000mm with scale 0.001)
    mock_depth = np.full((HEIGHT, WIDTH), 1000, dtype=np.uint16)
    
    # Mock Intrinsics
    mock_info = CameraInfo()
    mock_info.width = WIDTH
    mock_info.height = HEIGHT
    mock_info.distortion_model = "plumb_bob"
    # K = [fx 0 ppx; 0 fy ppy; 0 0 1]
    mock_info.K = [600.0, 0, 320.0, 0, 600.0, 240.0, 0, 0, 1]
    # P = [fx 0 ppx 0; 0 fy ppy 0; 0 0 1 0]
    mock_info.P = [600.0, 0, 320.0, 0, 0, 600.0, 240.0, 0, 0, 0, 1, 0]

    rospy.loginfo("Webcam publisher (with mock depth/info) started.")

    while not rospy.is_shutdown():
        # 1. Read a frame from the camera
        ret, frame = cap.read()
        
        if not ret:
            rospy.logwarn("Failed to grab frame from webcam.")
            continue

        # Ensure frame is the right size for mock depth
        if frame.shape[0] != HEIGHT or frame.shape[1] != WIDTH:
            frame = cv2.resize(frame, (WIDTH, HEIGHT))

        try:
            # Create ROS Header
            header = rospy.Header()
            header.stamp = rospy.Time.now()
            header.frame_id = "camera_link"

            # 2. Publish Color
            ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")
            ros_image.header = header
            image_pub.publish(ros_image)

            # 3. Publish Mock Depth
            ros_depth = bridge.cv2_to_imgmsg(mock_depth, "16UC1")
            ros_depth.header = header
            depth_pub.publish(ros_depth)

            # 4. Publish Mock Info
            mock_info.header = header
            info_pub.publish(mock_info)

            # 5. Publish Mock Scale (0.001 = mm to meters)
            scale_pub.publish(Float32(data=0.001))

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