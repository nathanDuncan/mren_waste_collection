#!/usr/bin/env python3

import sys
import os
import json
import numpy as np
import cv2
import rospy
import rospkg
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# --- Import RealSense ---
# Keep your specific Pi 5 build path
realsense_dir = "/home/pi5/librealsense/build/Release"
if realsense_dir not in sys.path:
    sys.path.append(realsense_dir)

try:
    import pyrealsense2 as rs
except ImportError:
    print(f"❌ Error: Could not find pyrealsense2 in {realsense_dir}")
    sys.exit(1)

from ultralytics import YOLO
import socket
import zlib
import threading

# Import generated Protobuf
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.append(script_dir)

try:
    import image_stream_pb2
except ImportError:
    rospy.logwarn("image_stream_pb2.py not found. Network mode will not work.")


class ObjectDetector:
    def __init__(self):
        rospy.init_node('object_detector', anonymous=True)
        
        # 1. Parameters & Path
        # In Noetic, params are usually loaded from the parameter server
        self.enable_vis = rospy.get_param('~enable_vis', True)
        self.bridge = CvBridge()

        # Get package path using rospkg
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('autonomous_litter_bot_package')
        self.model_path = os.path.join(package_path, 'models', 'segmentation_small_openvino_model')
        
        rospy.loginfo(f"Loading Model: {self.model_path}")
        self.model = YOLO(self.model_path, task="segment")

        # 2. Publishers
        self.publisher_ = rospy.Publisher('detected_objects', String, queue_size=10)
        self.debug_pub_ = rospy.Publisher('debug_image', Image, queue_size=10) 

        # 3. Mode Selection
        self.use_network = rospy.get_param('~use_network', False)
        self.network_port = rospy.get_param('~network_port', 25001)
        
        self.latest_color = None
        self.latest_depth = None
        self.latest_intrinsics = None
        self.latest_depth_scale = None
        self.data_lock = threading.Lock()

        if self.use_network:
            rospy.loginfo(f"🌐 Network Mode Enabled. Listening on port {self.network_port}")
            self.receiver_thread = threading.Thread(target=self.network_receiver, daemon=True)
            self.receiver_thread.start()
        else:
            rospy.loginfo("📷 Local RealSense Mode Enabled.")
            self.setup_realsense()

        # ROS 1 Timer uses a Duration object
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        rospy.loginfo("✅ Node Initialized and Running.")

    def setup_realsense(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 6)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 6)
        self.align = rs.align(rs.stream.color)

        profile = self.pipeline.start(self.config)
        color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
        self.intrinsics = color_stream.get_intrinsics()
        # Get depth scale
        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()

    def network_receiver(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("0.0.0.0", self.network_port))
        sock.settimeout(1.0)
        
        while not rospy.is_shutdown():
            try:
                data, addr = sock.recvfrom(1024 * 1024) # 1MB buffer
                msg = image_stream_pb2.ImageFrame()
                msg.ParseFromString(data)
                
                # Decompress Color
                color_np = np.frombuffer(msg.color_data, dtype=np.uint8)
                color_img = cv2.imdecode(color_np, cv2.IMREAD_COLOR)
                
                # Decompress Depth
                depth_decompressed = zlib.decompress(msg.depth_data)
                depth_img = np.frombuffer(depth_decompressed, dtype=np.uint16).reshape((msg.height, msg.width))
                
                with self.data_lock:
                    self.latest_color = color_img
                    self.latest_depth = depth_img
                    self.latest_intrinsics = msg
                    self.latest_depth_scale = msg.depth_scale
                    
            except socket.timeout:
                continue
            except Exception as e:
                rospy.logerr_throttle(5, f"Receiver error: {e}")

    def timer_callback(self, event):
        # Refresh parameter value in case it changed
        show_debug = rospy.get_param('~enable_vis', self.enable_vis)

        if self.use_network:
            with self.data_lock:
                if self.latest_color is None or self.latest_depth is None:
                    return
                frame = self.latest_color.copy()
                depth_img = self.latest_depth.copy()
                intrinsics = self.latest_intrinsics
                depth_scale = self.latest_depth_scale
        else:
            try:
                frames = self.pipeline.wait_for_frames(timeout_ms=5000)
            except RuntimeError:
                return

            aligned_frames = self.align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            
            if not color_frame or not depth_frame: return

            frame = np.asanyarray(color_frame.get_data())
            depth_img = np.asanyarray(depth_frame.get_data())
            intrinsics = self.intrinsics
            depth_scale = self.depth_scale

        H, W = frame.shape[:2]
        debug_frame = frame.copy() if show_debug else None

        results = self.model.predict(frame, imgsz=640, conf=0.75, verbose=False)
        r = results[0]
        detected_list = []

        if r.masks is not None:
            for mask_points in r.masks.xy:
                c = mask_points.astype(np.int32)
                if len(c) < 5: continue 

                ellipse = cv2.fitEllipse(c)
                (cx, cy), (MA, ma), angle = ellipse
                
                cx_int = max(0, min(int(cx), W - 1))
                cy_int = max(0, min(int(cy), H - 1))

                # Calculate distance in meters from depth map
                # depth_img[cy, cx] is in units specified by depth_scale (usually mm)
                dist_raw = depth_img[cy_int, cx_int]
                distance_meters = dist_raw * depth_scale
                
                if distance_meters <= 0: continue 

                real_width_cm = (ma * distance_meters / intrinsics.fx) * 100
                real_length_cm = (MA * distance_meters / intrinsics.fy) * 100
                
                # Angle Logic (Kept same as original)
                if angle > 90:
                    angle_major = angle - 180
                else:
                    angle_major = angle
                angle_major = -angle_major

                if show_debug:
                    cv2.drawContours(debug_frame, [c], -1, (0, 255, 0), 2)
                    cv2.circle(debug_frame, (cx_int, cy_int), 5, (0, 0, 255), -1)
                    labels = [
                        f"Dist: {distance_meters:.2f}m",
                        f"Size: {real_width_cm:.1f}x{real_length_cm:.1f}cm",
                        f"Ang: {angle_major:.1f}deg",
                    ]
                    text_x = min(cx_int + 15, W - 160)
                    text_y = cy_int
                    for line in labels:
                        cv2.putText(debug_frame, line, (text_x, text_y), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
                        text_y += 18

                obj_data = {
                    "x_pos": float(cx), "y_pos": float(cy),
                    "dist_meters": float(distance_meters),
                    "width_cm": float(real_width_cm),
                    "length_cm": float(real_length_cm),
                    "angle": float(angle_major)
                }
                detected_list.append(obj_data)

        # Publish JSON string
        self.publisher_.publish(String(data=json.dumps(detected_list)))

        # Publish Debug Image
        if show_debug and debug_frame is not None:
            self.debug_pub_.publish(self.bridge.cv2_to_imgmsg(debug_frame, "bgr8"))

    def stop(self):
        if hasattr(self, 'pipeline'):
            self.pipeline.stop()

if __name__ == '__main__':
    try:
        detector = ObjectDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        if 'detector' in locals():
            detector.stop()