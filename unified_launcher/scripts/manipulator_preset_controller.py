#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
from open_manipulator_msgs.msg import JointPosition
from open_manipulator_msgs.srv import SetJointPosition

from sensor_msgs.msg import JointState

class ManipulatorPresetController:
    def __init__(self):
        rospy.init_node('manipulator_preset_controller')

        self.service_name = '/open_manipulator/goal_joint_space_path'
        rospy.wait_for_service(self.service_name)
        self.set_joint_position = rospy.ServiceProxy(self.service_name, SetJointPosition)

        self.current_joint_state = None
        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        self.sub = rospy.Subscriber('/manipulator_state', Int32, self.callback)
        
        rospy.loginfo("Manipulator Preset Controller Initialized")

    def joint_state_callback(self, msg):
        self.current_joint_state = msg

    def callback(self, msg):
        joint_position = JointPosition()
        joint_position.joint_name = ["joint1", "joint2", "joint3", "joint4"]
        
        if msg.data == 0:
            if self.current_joint_state:
                # Filter for the relevant joints if needed, but for now print all from topic
                # Assuming /joint_states contains 'joint1'...'joint4' in order or we map them.
                # OpenMANIPULATOR usually publishes them.
                
                # Check if we can map names to positions
                try:
                    positions = []
                    for name in joint_position.joint_name:
                        idx = self.current_joint_state.name.index(name)
                        positions.append(self.current_joint_state.position[idx])
                    
                    rospy.loginfo("--- Current Joint Configuration ---")
                    rospy.loginfo(f"Positions: {positions}")
                    rospy.loginfo("Copy-paste ready: joint_position.position = " + str(positions))
                    rospy.loginfo("-----------------------------------")
                except ValueError:
                     rospy.logwarn("Could not find all manipulator joints in /joint_states")
                     rospy.loginfo(f"Available joints: {self.current_joint_state.name}")
            else:
                rospy.logwarn("No joint state data received yet.")
            return

        # Define presets (radians)
        # TODO: Tune these values for real robot safety
        if msg.data == 1:
            # Home
            joint_position.position = [0.0, 0.0, 0.0, 0.0]
            rospy.loginfo("Moving to Home Pose")
        elif msg.data == 2:
            # Collect (Example values)
            joint_position.position = [0.0, 0.5, 0.5, 0.0] 
            rospy.loginfo("Moving to Collect Pose")
        elif msg.data == 3:
            # Dump (Example values)
            joint_position.position = [0.0, -0.5, -0.5, 0.0]
            rospy.loginfo("Moving to Dump Pose")
        else:
            rospy.logwarn(f"Unknown state: {msg.data}")
            return

        try:
            # Args: planning_group, joint_position, path_time
            self.set_joint_position("arm", joint_position, 2.0)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    try:
        controller = ManipulatorPresetController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
