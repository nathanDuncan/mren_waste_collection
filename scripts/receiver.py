#!/usr/bin/env python3

import rospy
import socket
import sys
import os
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float32MultiArray

script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.append(script_dir)

# Attempt to import the proto definition
try:
    import detection_pb2
except ImportError:
    print(f"❌ CRITICAL: detection_pb2.py not found in {script_dir}")
    sys.exit(1)

class ReceiverNode:
    def __init__(self):
        rospy.init_node('receiver_node', log_level=rospy.DEBUG)
        
        self.port = rospy.get_param('~port', 25000)
        self.center_x = rospy.get_param('~center_x', 320.0)
        self.center_y = rospy.get_param('~center_y', 240.0)
        
        # Setup UDP Socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.bind(("0.0.0.0", self.port))
        except Exception as e:
            print(f"❌ Socket Bind Error: {e}")
            sys.exit(1)
            
        self.sock.settimeout(0.5) 
        
        self.camera_data_pub = rospy.Publisher('/camera_data', Float32MultiArray, queue_size=10)
        
        rospy.loginfo(f"✅ Protobuf Receiver Listening on port {self.port}")
        print(f"--- Manual Debug: Listening on port {self.port} ---")

    def run(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            try:
                # Receive raw data
                data, addr = self.sock.recvfrom(4096)
                
                # DIAGNOSTIC: Print raw info immediately
                # print(f"📩 Received {len(data)} bytes from {addr}")
                
                # Parse Protobuf
                frame = detection_pb2.DetectionFrame()
                frame.ParseFromString(data)
                
                camera_msg = Float32MultiArray()
                
                if len(frame.objects) > 0:
                    obj = frame.objects[0] # Take first detected object
                    
                    # Calculate error
                    '''
                    float dist_meters = 3; 
                    float width_cm = 4;    
                    float length_cm = 5;   
                    float angle = 6;      
                    '''
                    camera_data = [obj.x_pos, obj.y_pos, obj.dist_meters, obj.width_cm, obj.length_cm, obj.angle]
                    
                    camera_msg.data = camera_data

                    
                    # rospy.loginfo(f"🎯 Object: dist={obj.dist_meters:.2f}m, err_x={err_x:.1f}")
                    # print(f"   -> Detection: X:{obj.x_pos:.1f}, Dist:{obj.dist_meters:.2f}m")
                else:
                    camera_msg.data = [-1.0, 0.0, 0.0, 0.0, 0.0]
                    # camera_msg.w = 0.0 # No objects in frame
                    # print("   -> Empty frame received (0 objects)")
                
                self.camera_data_pub.publish(camera_msg)
                    
            except socket.timeout:
                # This is normal, happens every 0.5s if no data arrives
                pass
            except Exception as e:
                rospy.logerr(f"Receiver Logic Error: {e}")
                print(f"❌ Error: {e}")
            
            rate.sleep()

if __name__ == '__main__':
    try:
        node = ReceiverNode()
        node.run()
    except rospy.ROSInterruptException:
        pass