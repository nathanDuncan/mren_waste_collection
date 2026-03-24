#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import pyrealsense2 as rs
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def main():
    # 1. Initialize ROS Node
    rospy.init_node('realsense_publisher', anonymous=True)
    
    # --- Configuration ---
    # Using 15 FPS and 640x480 to keep bandwidth low on the Go1 internal bus
    WIDTH = 640
    HEIGHT = 480
    FPS = 6

    # --- Publishers ---
    color_pub = rospy.Publisher('/camera/color/image_raw', Image, queue_size=10)
    depth_pub = rospy.Publisher('/camera/depth/image_raw', Image, queue_size=10)
    
    bridge = CvBridge()

    # --- RealSense Setup ---
    pipeline = rs.pipeline()
    config = rs.config()

    # Explicitly enable streams
    config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
    # config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)

    # Align object: This ensures depth and color frames are perfectly overlaid
    # We align depth to color because color usually has a wider field of view
    align_to = rs.stream.color
    align = rs.align(align_to)

    try:
        # Start streaming
        # ... existing config setup ...
        profile = pipeline.start(config)
        device = profile.get_device()

        # This tells the camera: "Just send me the pixels, no extra data."
        # This is often required for stable streaming on ARM processors like the Pi.
        for sensor in device.query_sensors():
            if sensor.supports(rs.option.metadata_attrs_enabled):
                sensor.set_option(rs.option.metadata_attrs_enabled, 0)
        
        # Get depth scale (useful if you want to convert pixels to meters later)
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        rospy.loginfo(f"RealSense pipeline started. Depth scale is: {depth_scale}")

    except Exception as e:
        rospy.logerr(f"Could not start pipeline: {e}")
        return

    rate = rospy.Rate(FPS)

    try:
        while not rospy.is_shutdown():
            # Wait for frameset
            try:
                frames = pipeline.wait_for_frames(timeout_ms=5000)
            except RuntimeError:
                rospy.logwarn("Timeout waiting for frames. Skipping...")
                continue

            # Align the depth frame to color frame
            aligned_frames = align.process(frames)
            
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()

            if not color_frame or not depth_frame:
                continue

            # --- Convert to OpenCV/Numpy arrays ---
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            # --- Publish Color ---
            try:
                color_msg = bridge.cv2_to_imgmsg(color_image, "bgr8")
                color_msg.header.stamp = rospy.Time.now()
                color_msg.header.frame_id = "camera_color_optical_frame"
                color_pub.publish(color_msg)
            except CvBridgeError as e:
                rospy.logerr(f"Color Bridge Error: {e}")

            # --- Publish Depth ---
            try:
                # Depth is 16-bit unsigned (millimeters)
                depth_msg = bridge.cv2_to_imgmsg(depth_image, encoding="16UC1")
                depth_msg.header.stamp = rospy.Time.now()
                depth_msg.header.frame_id = "camera_depth_optical_frame"
                depth_pub.publish(depth_msg)
            except CvBridgeError as e:
                rospy.logerr(f"Depth Bridge Error: {e}")

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
    finally:
        pipeline.stop()
        rospy.loginfo("RealSense pipeline stopped.")

if __name__ == '__main__':
    main()