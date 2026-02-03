#!/usr/bin/env python3

import rospy
import socket
import struct
from geometry_msgs.msg import Quaternion

class ReceiverNode:
    def __init__(self):
        rospy.init_node('receiver_node')
        
        self.port = rospy.get_param('~port', 25000)
        self.center_x = rospy.get_param('~center_x', 320.0)
        self.center_y = rospy.get_param('~center_y', 240.0)
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", self.port))
        self.sock.settimeout(0.1) # Non-blocking with timeout for ROS spin
        
        self.error_pub = rospy.Publisher('/control_errors', Quaternion, queue_size=10)
        
        rospy.loginfo(f"Receiver Node Listening on port {self.port}")

    def run(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            try:
                data, addr = self.sock.recvfrom(1024)
                if len(data) == 16: # 4 floats
                    x, y, depth, flag = struct.unpack('ffff', data)
                    
                    error_msg = Quaternion()
                    
                    if flag > 0.5:
                        # Calculate Error (Target - Center) 
                        # Or (Current - Target)? 
                        # simple_servoing expects:
                        # P-Control: cmd = -kp * error.
                        # If error is positive, we turn negative. 
                        # If Target is Right (x > center), Error should be Positive.
                        # So Error = x - center.
                        
                        error_msg.x = x - self.center_x
                        error_msg.y = y - self.center_y
                        error_msg.z = depth # Pass through depth/size
                        error_msg.w = 1.0   # Found
                    else:
                        error_msg.x = 0.0
                        error_msg.y = 0.0
                        error_msg.z = 0.0
                        error_msg.w = 0.0   # Not found
                        
                    self.error_pub.publish(error_msg)
                    
            except socket.timeout:
                pass
            except Exception as e:
                rospy.logerr(f"Receiver Error: {e}")
            
            rate.sleep()

if __name__ == '__main__':
    try:
        node = ReceiverNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
