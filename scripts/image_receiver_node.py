#!/usr/bin/env python3

# import rospy
# import socket
# import numpy as np
# import cv2
# import zlib
# import sys
# import os
# from sensor_msgs.msg import Image, CameraInfo
# from std_msgs.msg import Float32
# from cv_bridge import CvBridge

# # Import generated Protobuf
# script_dir = os.path.dirname(os.path.abspath(__file__))
# if script_dir not in sys.path:
#     sys.path.append(script_dir)

# try:
#     import image_stream_pb2
# except ImportError:
#     rospy.logerr("image_stream_pb2.py not found. Network mode will not work.")
#     sys.exit(1)

# class ImageReceiverNode:
#     def __init__(self):
#         rospy.init_node('image_receiver_node', anonymous=True)
        
#         self.bridge = CvBridge()
#         self.port = rospy.get_param('~network_port', 25005)
        
#         # Publishers
#         self.image_pub = rospy.Publisher('camera_frames', Image, queue_size=10)
#         self.depth_pub = rospy.Publisher('depth_frames', Image, queue_size=10)
#         self.info_pub = rospy.Publisher('camera_info', CameraInfo, queue_size=10)
#         self.depth_scale_pub = rospy.Publisher('depth_scale', Float32, queue_size=10)
        
#         # UDP Socket Setup
#         self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#         self.sock.bind(("0.0.0.0", self.port))
#         self.sock.settimeout(1.0)
        
#         rospy.loginfo(f"Listening for ImageFrame protobuf on UDP port {self.port}")

#     def run(self):
#         while not rospy.is_shutdown():
#             try:
#                 data, addr = self.sock.recvfrom(1024 * 1024) # 1MB buffer
#                 msg = image_stream_pb2.ImageFrame()
#                 msg.ParseFromString(data)
                
#                 # Create ROS Header
#                 header = rospy.Header()
#                 header.stamp = rospy.Time.now()
#                 header.frame_id = "camera_link"
                
#                 # 1. Process Color Image
#                 colour_np = np.frombuffer(msg.colour_data, dtype=np.uint8)
#                 colour_img = cv2.imdecode(colour_np, cv2.IMREAD_COLOR)
#                 if colour_img is not None:
#                     ros_image = self.bridge.cv2_to_imgmsg(colour_img, "bgr8")
#                     ros_image.header = header
#                     self.image_pub.publish(ros_image)
                
#                 # 2. Process Depth Image
#                 depth_decompressed = zlib.decompress(msg.depth_data)
#                 depth_img = np.frombuffer(depth_decompressed, dtype=np.uint16).reshape((msg.height, msg.width))
#                 if depth_img is not None:
#                     ros_depth = self.bridge.cv2_to_imgmsg(depth_img, "16UC1")
#                     ros_depth.header = header
#                     self.depth_pub.publish(ros_depth)
                
#                 # 3. Process Camera Info
#                 cam_info = CameraInfo()
#                 cam_info.header = header
#                 cam_info.width = msg.width
#                 cam_info.height = msg.height
#                 cam_info.distortion_model = "plumb_bob"
#                 # K = [fx 0 ppx; 0 fy ppy; 0 0 1]
#                 cam_info.K = [msg.fx, 0, msg.ppx, 0, msg.fy, msg.ppy, 0, 0, 1]
#                 # P = [fx 0 ppx 0; 0 fy ppy 0; 0 0 1 0]
#                 cam_info.P = [msg.fx, 0, msg.ppx, 0, 0, msg.fy, msg.ppy, 0, 0, 0, 1, 0]
#                 self.info_pub.publish(cam_info)
                
#                 # 4. Process Depth Scale
#                 self.depth_scale_pub.publish(Float32(data=msg.depth_scale))
                
#             except socket.timeout:
#                 continue
#             except Exception as e:
#                 rospy.logerr_throttle(5, f"Image Receiver error: {e}")

# if __name__ == '__main__':
#     try:
#         node = ImageReceiverNode()
#         node.run()
#     except rospy.ROSInterruptException:
#         pass


### TCP Server ###

#!/usr/bin/env python3

import rospy
import socket
import numpy as np
import cv2
import zlib
import sys
import os
import struct  # Required for unpacking the 4-byte length header
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32
from cv_bridge import CvBridge

# Import generated Protobuf
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.append(script_dir)

try:
    import image_stream_pb2
except ImportError:
    rospy.logerr("image_stream_pb2.py not found. Network mode will not work.")
    sys.exit(1)

class ImageReceiverNode:
    def __init__(self):
        rospy.init_node('image_receiver_node', anonymous=True)
        
        self.bridge = CvBridge()
        self.port = rospy.get_param('~network_port', 25005)
        
        # Publishers
        self.image_pub = rospy.Publisher('camera_frames', Image, queue_size=10)
        self.depth_pub = rospy.Publisher('depth_frames', Image, queue_size=10)
        self.info_pub = rospy.Publisher('camera_info', CameraInfo, queue_size=10)
        self.depth_scale_pub = rospy.Publisher('depth_scale', Float32, queue_size=10)
        
        # --- TCP SERVER SETUP ---
        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_sock.bind(("0.0.0.0", self.port))
        self.server_sock.listen(1)
        self.server_sock.settimeout(1.0)
        
        rospy.loginfo(f"Listening for ImageFrame TCP connections on port {self.port}")

    def recv_all(self, conn, n):
        """Helper function to receive exactly n bytes or return None if connection closes."""
        data = bytearray()
        while len(data) < n:
            try:
                packet = conn.recv(n - len(data))
                if not packet:
                    return None
                data.extend(packet)
            except socket.timeout:
                if rospy.is_shutdown():
                    return None
                continue
        return data

    def run(self):
        while not rospy.is_shutdown():
            rospy.loginfo("Waiting for a connection...")
            try:
                conn, addr = self.server_sock.accept()
                rospy.loginfo(f"Connected by {addr}")
                conn.settimeout(1.0)
            except socket.timeout:
                continue

            with conn:
                while not rospy.is_shutdown():
                    try:
                        # 1. Read the 4-byte length header
                        header = self.recv_all(conn, 4)
                        if header is None:
                            rospy.logwarn("Sender disconnected.")
                            break
                        
                        msg_len = struct.unpack('>I', header)[0]
                        
                        # 2. Read the actual Protobuf payload
                        payload = self.recv_all(conn, msg_len)
                        if payload is None:
                            rospy.logwarn("Connection lost during payload transfer.")
                            break
                        
                        # 3. Parse Protobuf
                        msg = image_stream_pb2.ImageFrame()
                        msg.ParseFromString(payload)
                        
                        # --- Processing & Publishing ---
                        header_ros = rospy.Header()
                        header_ros.stamp = rospy.Time.now()
                        header_ros.frame_id = "camera_link"
                        
                        # Color
                        colour_np = np.frombuffer(msg.colour_data, dtype=np.uint8)
                        colour_img = cv2.imdecode(colour_np, cv2.IMREAD_COLOR)
                        if colour_img is not None:
                            ros_image = self.bridge.cv2_to_imgmsg(colour_img, "bgr8")
                            ros_image.header = header_ros
                            self.image_pub.publish(ros_image)
                        
                        # Depth
                        depth_decompressed = zlib.decompress(msg.depth_data)
                        depth_img = np.frombuffer(depth_decompressed, dtype=np.uint16).reshape((msg.height, msg.width))
                        if depth_img is not None:
                            ros_depth = self.bridge.cv2_to_imgmsg(depth_img, "16UC1")
                            ros_depth.header = header_ros
                            self.depth_pub.publish(ros_depth)
                        
                        # Camera Info
                        cam_info = CameraInfo()
                        cam_info.header = header_ros
                        cam_info.width = msg.width
                        cam_info.height = msg.height
                        cam_info.distortion_model = "plumb_bob"
                        cam_info.K = [msg.fx, 0, msg.ppx, 0, msg.fy, msg.ppy, 0, 0, 1]
                        cam_info.P = [msg.fx, 0, msg.ppx, 0, 0, msg.fy, msg.ppy, 0, 0, 0, 1, 0]
                        self.info_pub.publish(cam_info)
                        
                        # Depth Scale
                        self.depth_scale_pub.publish(Float32(data=msg.depth_scale))

                    except Exception as e:
                        rospy.logerr(f"Session error: {e}")
                        break

if __name__ == '__main__':
    try:
        node = ImageReceiverNode()
        node.run()
    except rospy.ROSInterruptException:
        pass