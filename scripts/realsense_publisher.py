#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import pyrealsense2 as rs
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def main():
    # Initialize the ROS node
    rospy.init_node('realsense_publisher', anonymous=True)

    # --- Publishers ---
    # Publisher for RGB color frames
    color_pub = rospy.Publisher('/camera_frames', Image, queue_size=10)
    
    # Publisher for Depth frames (16-bit grayscale)
    # We use the standard topic name pattern for depth
    depth_pub = rospy.Publisher('/camera/depth/image_raw', Image, queue_size=10)

    # Create a CvBridge object
    bridge = CvBridge()

    # --- RealSense Setup ---
    pipeline = rs.pipeline()
    config = rs.config()

    # Get device info (optional, just for logging)
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    try:
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        rospy.loginfo(f"RealSense device found: {device.get_info(rs.camera_info.name)}")
    except RuntimeError as e:
        rospy.logerr(f"RealSense not found: {e}")
        return

    # --- Configure Streams ---
    # 1. Enable Color Stream (640x480, 30fps, RGB8)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    
    # 2. Enable Depth Stream (640x480, 30fps, Z16)
    # Z16 format is 16-bit unsigned integer, representing depth in millimeters.
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    # Start streaming
    try:
        pipeline.start(config)
        rospy.loginfo("RealSense pipeline started (Color + Depth).")
    except RuntimeError as e:
        rospy.logerr(f"RealSense error: {e}")
        return

    # Set the loop rate
    rate = rospy.Rate(30) # 30 Hz

    try:
        while not rospy.is_shutdown():
            # 1. Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            
            if not color_frame or not depth_frame:
                continue

            # --- Process Color Frame ---
            # Convert to numpy array
            color_image = np.asanyarray(color_frame.get_data())
            
            # Publish Color
            try:
                ros_color_msg = bridge.cv2_to_imgmsg(color_image, "bgr8")
                ros_color_msg.header.stamp = rospy.Time.now() # Timestamp for sync
                color_pub.publish(ros_color_msg)
            except CvBridgeError as e:
                rospy.logerr(f"Color Bridge Error: {e}")

            # --- Process Depth Frame ---
            # Convert to numpy array (16-bit)
            depth_image = np.asanyarray(depth_frame.get_data())
            
            # Publish Depth
            # Note: We use "16UC1" encoding for 16-bit unsigned single-channel image
            try:
                ros_depth_msg = bridge.cv2_to_imgmsg(depth_image, "16UC1")
                ros_depth_msg.header.stamp = rospy.Time.now() # Timestamp for sync
                depth_pub.publish(ros_depth_msg)
            except CvBridgeError as e:
                rospy.logerr(f"Depth Bridge Error: {e}")

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
    finally:
        # Stop streaming
        pipeline.stop()
        rospy.loginfo("RealSense pipeline stopped.")

if __name__ == '__main__':
    main()