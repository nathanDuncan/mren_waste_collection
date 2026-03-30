#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import pyrealsense2 as rs
import socket
import zlib
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

class ImageSender(Node):
    def __init__(self):
        super().__init__('image_sender')

        # --- Configuration ---
        self.declare_parameter('target_ip', '127.0.0.1')
        self.declare_parameter('port', 25001)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 6)

        self.target_ip = self.get_parameter('target_ip').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        self.fps = self.get_parameter('fps').get_parameter_value().integer_value

        self.get_logger().info(f"🚀 Image Sender starting. Target: {self.target_ip}:{self.port}")

        # --- Networking ---
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # --- RealSense Setup ---
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
        self.config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)

        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)

        try:
            self.profile = self.pipeline.start(self.config)
            
            # Get intrinsics
            color_stream = self.profile.get_stream(rs.stream.color).as_video_stream_profile()
            self.intrinsics = color_stream.get_intrinsics()
            
            # Get depth scale
            depth_sensor = self.profile.get_device().first_depth_sensor()
            self.depth_scale = depth_sensor.get_depth_scale()
            
            self.get_logger().info(f"RealSense started. Depth scale: {self.depth_scale}")
        except Exception as e:
            self.get_logger().error(f"Could not start RealSense: {e}")
            raise e

        # Create timer for the loop
        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        try:
            # timeout_ms=0 means it won't block the executor if frames aren't ready? 
            # Actually, wait_for_frames with a timeout is okay if we use a MultiThreadedExecutor,
            # but in a SingleThreadedExecutor it will block other callbacks.
            # However, this node only has this one timer, so it's fine.
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
        except RuntimeError:
            return

        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if not color_frame or not depth_frame:
            return

        # --- Process Images ---
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        # 1. Compress Color (JPEG)
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]
        _, color_encoded = cv2.imencode('.jpg', color_image, encode_param)
        color_bytes = color_encoded.tobytes()

        # 2. Compress Depth (Zlib)
        depth_bytes = depth_image.tobytes()
        depth_compressed = zlib.compress(depth_bytes)

        # --- Create Protobuf ---
        msg = image_stream_pb2.ImageFrame()
        msg.color_data = color_bytes
        msg.depth_data = depth_compressed
        msg.width = self.width
        msg.height = self.height
        msg.timestamp = self.get_clock().now().nanoseconds / 1e9
        msg.fx = self.intrinsics.fx
        msg.fy = self.intrinsics.fy
        msg.ppx = self.intrinsics.ppx
        msg.ppy = self.intrinsics.ppy
        msg.depth_scale = self.depth_scale

        # --- Send via UDP ---
        serialized_data = msg.SerializeToString()
        
        if len(serialized_data) > 65507:
            # Using throttle_duration_sec is ROS2 way
            self.get_logger().warning(
                f"Payload too large for UDP ({len(serialized_data)} bytes).", 
                throttle_duration_sec=2.0
            )
        
        try:
            self.sock.sendto(serialized_data, (self.target_ip, self.port))
        except Exception as e:
            self.get_logger().error(f"Send error: {e}", throttle_duration_sec=2.0)

    def destroy_node(self):
        self.pipeline.stop()
        self.sock.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    image_sender = ImageSender()
    try:
        rclpy.spin(image_sender)
    except KeyboardInterrupt:
        pass
    finally:
        # Explicitly stop the pipeline if it hasn't been stopped
        # image_sender.destroy_node() is called by the executor normally, 
        # but let's be safe.
        image_sender.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
