#!/usr/bin/env python3

import rospy
import socket
import numpy as np
import cv2
import zlib
import sys
import os
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
        
        # UDP Socket Setup
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", self.port))
        self.sock.settimeout(1.0)
        
        rospy.loginfo(f"Listening for ImageFrame protobuf on UDP port {self.port}")

    def run(self):
        while not rospy.is_shutdown():
            try:
                data, addr = self.sock.recvfrom(1024 * 1024) # 1MB buffer
                msg = image_stream_pb2.ImageFrame()
                msg.ParseFromString(data)
                
                # Create ROS Header
                header = rospy.Header()
                header.stamp = rospy.Time.now()
                header.frame_id = "camera_link"
                
                # 1. Process Color Image
                colour_np = np.frombuffer(msg.colour_data, dtype=np.uint8)
                colour_img = cv2.imdecode(colour_np, cv2.IMREAD_COLOR)
                if colour_img is not None:
                    ros_image = self.bridge.cv2_to_imgmsg(colour_img, "bgr8")
                    ros_image.header = header
                    self.image_pub.publish(ros_image)
                
                # 2. Process Depth Image
                depth_decompressed = zlib.decompress(msg.depth_data)
                depth_img = np.frombuffer(depth_decompressed, dtype=np.uint16).reshape((msg.height, msg.width))
                if depth_img is not None:
                    ros_depth = self.bridge.cv2_to_imgmsg(depth_img, "16UC1")
                    ros_depth.header = header
                    self.depth_pub.publish(ros_depth)
                
                # 3. Process Camera Info
                cam_info = CameraInfo()
                cam_info.header = header
                cam_info.width = msg.width
                cam_info.height = msg.height
                cam_info.distortion_model = "plumb_bob"
                # K = [fx 0 ppx; 0 fy ppy; 0 0 1]
                cam_info.K = [msg.fx, 0, msg.ppx, 0, msg.fy, msg.ppy, 0, 0, 1]
                # P = [fx 0 ppx 0; 0 fy ppy 0; 0 0 1 0]
                cam_info.P = [msg.fx, 0, msg.ppx, 0, 0, msg.fy, msg.ppy, 0, 0, 0, 1, 0]
                self.info_pub.publish(cam_info)
                
                # 4. Process Depth Scale
                self.depth_scale_pub.publish(Float32(data=msg.depth_scale))
                
            except socket.timeout:
                continue
            except Exception as e:
                rospy.logerr_throttle(5, f"Image Receiver error: {e}")

if __name__ == '__main__':
    try:
        node = ImageReceiverNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
