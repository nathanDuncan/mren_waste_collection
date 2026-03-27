#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import pyrealsense2 as rs
import socket
import zlib
import time
import sys
import os

# Import generated Protobuf
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.append(script_dir)

try:
    import image_stream_pb2
except ImportError:
    print(f"❌ Error: image_stream_pb2.py not found. Run 'protoc --python_out=. image_stream.proto' first.")
    sys.exit(1)

def main():
    rospy.init_node('image_sender', anonymous=True)

    # --- Configuration ---
    TARGET_IP = rospy.get_param('~target_ip', '127.0.0.1')
    PORT = rospy.get_param('~port', 25001)
    WIDTH = 640
    HEIGHT = 480
    FPS = 6

    rospy.loginfo(f"🚀 Image Sender starting. Target: {TARGET_IP}:{PORT}")

    # --- Networking ---
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # Increase buffer size for large UDP packets (if possible)
    # sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 2**20) # 1MB

    # --- RealSense Setup ---
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
    config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)

    align_to = rs.stream.color
    align = rs.align(align_to)

    try:
        profile = pipeline.start(config)
        
        # Get intrinsics
        color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
        intrinsics = color_stream.get_intrinsics()
        
        # Get depth scale
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        
        rospy.loginfo(f"RealSense started. Depth scale: {depth_scale}")
    except Exception as e:
        rospy.logerr(f"Could not start RealSense: {e}")
        return

    rate = rospy.Rate(FPS)

    try:
        while not rospy.is_shutdown():
            try:
                frames = pipeline.wait_for_frames(timeout_ms=5000)
            except RuntimeError:
                continue

            aligned_frames = align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()

            if not color_frame or not depth_frame:
                continue

            # --- Process Images ---
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            # 1. Compress Color (JPEG)
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]
            _, color_encoded = cv2.imencode('.jpg', color_image, encode_param)
            color_bytes = color_encoded.tobytes()

            # 2. Compress Depth (Zlib)
            # Depth is 16-bit, so we convert to bytes directly
            depth_bytes = depth_image.tobytes()
            depth_compressed = zlib.compress(depth_bytes)

            # --- Create Protobuf ---
            msg = image_stream_pb2.ImageFrame()
            msg.color_data = color_bytes
            msg.depth_data = depth_compressed
            msg.width = WIDTH
            msg.height = HEIGHT
            msg.timestamp = rospy.get_time()
            msg.fx = intrinsics.fx
            msg.fy = intrinsics.fy
            msg.ppx = intrinsics.ppx
            msg.ppy = intrinsics.ppy
            msg.depth_scale = depth_scale

            # --- Send via UDP ---
            serialized_data = msg.SerializeToString()
            
            # WARNING: UDP payload limit is ~64KB. 
            # Our depth (even compressed) might exceed this.
            # If so, we need to split it or use TCP.
            # For this task, we'll try to send and warn if it's too large.
            
            if len(serialized_data) > 65507:
                rospy.logwarn_throttle(2, f"Payload too large for UDP ({len(serialized_data)} bytes). Consider using TCP or lower resolution.")
            
            try:
                sock.sendto(serialized_data, (TARGET_IP, PORT))
            except Exception as e:
                rospy.logerr_throttle(2, f"Send error: {e}")

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
    finally:
        pipeline.stop()
        sock.close()

if __name__ == '__main__':
    main()
