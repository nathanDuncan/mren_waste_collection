#!/usr/bin/env python3

import rospy
import socket
import struct
from geometry_msgs.msg import Quaternion

class SenderNode:
    def __init__(self):
        rospy.init_node('sender_node')
        
        # Configuration
        # TODO: Parameterize IP
        self.secondary_ip = rospy.get_param('~secondary_ip', '192.168.123.15') # Default Go1 IP? Or User's secondary?
        self.port = rospy.get_param('~port', 25000)
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        rospy.Subscriber('/target_pixel_data', Quaternion, self.callback)
        rospy.loginfo(f"Sender Node Initialized. Target: {self.secondary_ip}:{self.port}")

    def callback(self, msg):
        try:
            # Pack data: x, y, depth(z), flag(w)
            # Use 'f' for float (4 bytes) -> 4 floats = 16 bytes
            data = struct.pack('ffff', msg.x, msg.y, msg.z, msg.w)
            self.sock.sendto(data, (self.secondary_ip, self.port))
        except Exception as e:
            rospy.logerr(f"Socket Error: {e}")

if __name__ == '__main__':
    try:
        SenderNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
