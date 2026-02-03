#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import pyrealsense2 as rs
import time
import sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def hardware_reset_camera():
    """
    Forces a hardware reset on the first connected RealSense device.
    Useful for kicking the camera out of a 'zombie' state.
    """
    try:
        ctx = rs.context()
        devices = ctx.query_devices()
        
        if len(devices) == 0:
            rospy.logwarn("No RealSense devices detected during reset attempt.")
            return

        dev = devices[0]
        rospy.loginfo(f"Resetting device: {dev.get_info(rs.camera_info.name)}")
        try:
            rospy.loginfo(f"Firmware Version: {dev.get_info(rs.camera_info.firmware_version)}")
        except:
            pass
        dev.hardware_reset()
        
        # Wait for the camera to cycle power and reconnect (usually takes 3-5 seconds)
        rospy.loginfo("Waiting 5 seconds for camera to restart...")
        time.sleep(5)
    except Exception as e:
        rospy.logerr(f"Error during reset: {e}")

def main():
    # Initialize the ROS node
    rospy.init_node('realsense_publisher', anonymous=True)

    rospy.loginfo("--- STARTING COLOR ONLY MODE ---")
    rospy.loginfo("Configuration: Color Only, 15 FPS, 640x480")
    
    # 1. Attempt a hardware reset to clear "Frame didn't arrive" errors
    hardware_reset_camera()

    # --- Publishers ---
    # Only publishing color now
    color_pub = rospy.Publisher('/camera_frames', Image, queue_size=10)

    bridge = CvBridge()

    # --- RealSense Setup ---
    pipeline = rs.pipeline()
    config = rs.config()

    # Get device info
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    try:
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        rospy.loginfo(f"RealSense device found: {device.get_info(rs.camera_info.name)}")
        
        # Check USB Type (Critical for diagnosing bandwidth)
        usb_type = device.get_info(rs.camera_info.usb_type_descriptor)
        rospy.loginfo(f"USB Connection Type: {usb_type}")
        if "2.1" in usb_type:
            rospy.logwarn("WARNING: Camera detected as USB 2.1. Bandwidth is severely limited.")
        
    except RuntimeError as e:
        rospy.logerr(f"RealSense not found: {e}")
        return

    # --- Configure Streams (COLOR ONLY) ---
    # 1. Enable Color Stream
    # 640x480 @ 15 FPS is a good balance for stability
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
    
    # 2. Disable Depth Stream explicitly to save bandwidth
    config.disable_stream(rs.stream.depth)

    # Start streaming
    try:
        pipeline.start(config)
        rospy.loginfo("RealSense pipeline started (Color Only @ 15fps).")
    except RuntimeError as e:
        rospy.logerr(f"RealSense error: {e}")
        return

    rate = rospy.Rate(15) # Match the camera FPS

    try:
        while not rospy.is_shutdown():
            # Wait for frames with a timeout (default is 5000ms)
            try:
                frames = pipeline.wait_for_frames(timeout_ms=5000)
            except RuntimeError as e:
                rospy.logerr("CRITICAL: Timeout waiting for frames.")
                rospy.logerr("DIAGNOSIS HINT: Check 'dmesg | grep -i usb' for disconnects.")
                continue

            color_frame = frames.get_color_frame()
            
            if not color_frame:
                continue

            # --- Process Color Frame ---
            color_image = np.asanyarray(color_frame.get_data())
            try:
                ros_color_msg = bridge.cv2_to_imgmsg(color_image, "bgr8")
                ros_color_msg.header.stamp = rospy.Time.now()
                color_pub.publish(ros_color_msg)
                # rospy.loginfo_throttle(5, "Success: Publishing color frames...") 
            except CvBridgeError as e:
                rospy.logerr(f"Color Bridge Error: {e}")

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
    finally:
        pipeline.stop()
        rospy.loginfo("RealSense pipeline stopped.")

if __name__ == '__main__':
    main()