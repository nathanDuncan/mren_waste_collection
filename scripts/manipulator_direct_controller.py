#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from open_manipulator_msgs.msg import JointPosition
from open_manipulator_msgs.srv import SetJointPosition

class ManipulatorDirectController:
    def __init__(self):
        rospy.init_node('manipulator_direct_controller')

        self.service_name = '/open_manipulator/goal_joint_space_path'
        rospy.wait_for_service(self.service_name)
        self.set_joint_position = rospy.ServiceProxy(self.service_name, SetJointPosition)

        self.sub = rospy.Subscriber('/manipulator_joint_command', String, self.callback)
        
        rospy.loginfo("Manipulator Direct Controller Initialized")
        rospy.loginfo("Send commands to /manipulator_joint_command as: 'J1 J2 J3 J4' (e.g., '0.0 0.5 -0.5 0.0')")

    def callback(self, msg):
        try:
            # Parse string "0.0 1.1 0.0 0.4" -> [0.0, 1.1, 0.0, 0.4]
            parts = msg.data.split()
            if len(parts) != 4:
                rospy.logwarn(f"Invalid command format. Expected 4 values, got {len(parts)}: {msg.data}")
                return
            
            positions = [float(p) for p in parts]
            
            joint_position = JointPosition()
            joint_position.joint_name = ["joint1", "joint2", "joint3", "joint4"]
            joint_position.position = positions

            rospy.loginfo(f"Moving to: {positions}")

            # Args: planning_group, joint_position, path_time
            self.set_joint_position("arm", joint_position, 2.0)
            
        except ValueError as e:
            rospy.logwarn(f"Error parsing command '{msg.data}': {e}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    try:
        controller = ManipulatorDirectController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
