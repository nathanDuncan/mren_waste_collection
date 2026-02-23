#!/usr/bin/env python3

import rospy
import smach
import smach_ros
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from unitree_legged_msgs.msg import HighCmd
from open_manipulator_msgs.msg import JointPosition
from open_manipulator_msgs.srv import SetJointPosition

# Joint Presets
JOINT_SET_1 = [0.0, -1.0, 0.3, 0.7] # Home
JOINT_SET_2 = [-0.0061, -0.7777, -0.3451, 1.9527] # Short View

class StateMachineData:
    def __init__(self):
        self.camera_data = None
        self.lost_count = 0
        self.max_lost = 5
        self.debug = rospy.get_param('~debug_mode', False)
        if self.debug:
            rospy.loginfo("🛠 DEBUG MODE ENABLED: Base and posture commands will be logged but not published.")

    def update_camera(self, msg):
        if len(msg.data) >= 6:
            self.camera_data = msg.data
            self.lost_count = 0
        else:
            self.lost_count += 1
            if self.lost_count >= self.max_lost:
                self.camera_data = None

def move_manipulator(joint_angles, path_time=2.0):
    rospy.wait_for_service('/goal_joint_space_path', timeout=5.0)
    try:
        service = rospy.ServiceProxy('/goal_joint_space_path', SetJointPosition)
        joint_msg = JointPosition()
        joint_msg.joint_name = ["joint1", "joint2", "joint3", "joint4"]
        joint_msg.position = joint_angles
        resp = service(planning_group="arm", joint_position=joint_msg, path_time=path_time)
        return resp.is_planned
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logerr(f"Manipulator service call failed: {e}")
        return False

# --- States ---

class Idle(smach.State):
    def __init__(self, data):
        smach.State.__init__(self, outcomes=['detected', 'preempted'])
        self.data = data
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def execute(self, userdata):
        rospy.loginfo("Entering State: IDLE")
        
        # Move arm to Home
        move_manipulator(JOINT_SET_1)
        
        # Stop robot
        stop_msg = Twist()
        if not self.data.debug:
            self.cmd_vel_pub.publish(stop_msg)
        else:
            rospy.loginfo("[DEBUG] IDLE: Would publish zero cmd_vel")
        
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.data.camera_data is not None:
                return 'detected'
            rate.sleep()
        return 'preempted'

class ApproachCoarse(smach.State):
    def __init__(self, data):
        smach.State.__init__(self, outcomes=['centered', 'lost', 'preempted'])
        self.data = data
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def execute(self, userdata):
        rospy.loginfo("Entering State: APPROACH (COARSE)")
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            if self.data.camera_data is None:
                return 'lost'
            
            x_pos = self.data.camera_data[0]
            y_pos = self.data.camera_data[1]
            
            # Visual Servoing
            twist = Twist()
            
            # Simple P-control
            # Center X = 320
            error_x = 320 - x_pos
            twist.angular.z = error_x * 0.002 # Gain
            
            # Target Y > 400
            if y_pos > 400 and abs(error_x) < 20:
                self.cmd_vel_pub.publish(Twist()) # Stop before transitioning
                return 'centered'
            
            # Constant forward speed if not centered enough
            twist.linear.x = 0.1
            
            if not self.data.debug:
                self.cmd_vel_pub.publish(twist)
            else:
                rospy.loginfo(f"[DEBUG] APPROACH_COARSE: Twist(lin={twist.linear.x:.2f}, ang={twist.angular.z:.2f})")
            rate.sleep()
            
        return 'preempted'

class ApproachFine(smach.State):
    def __init__(self, data):
        smach.State.__init__(self, outcomes=['reached', 'lost', 'preempted'])
        self.data = data
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def execute(self, userdata):
        rospy.loginfo("Entering State: APPROACH (FINE)")
        
        # Move arm to Short View
        move_manipulator(JOINT_SET_2)
        
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.data.camera_data is None:
                return 'lost'
            
            x_pos = self.data.camera_data[0]
            y_pos = self.data.camera_data[1]
            
            # Visual Servoing to center (320, 240)
            twist = Twist()
            
            error_x = 320 - x_pos
            error_y = 240 - y_pos # Robot might need to move forward/backward to adjust y in frame
            
            twist.angular.z = error_x * 0.001
            twist.linear.x = error_y * 0.001
            
            if abs(error_x) < 10 and abs(error_y) < 10:
                if not self.data.debug:
                    self.cmd_vel_pub.publish(Twist())
                else:
                    rospy.loginfo("[DEBUG] APPROACH_FINE: Reached target, would stop.")
                return 'reached'
            
            if not self.data.debug:
                self.cmd_vel_pub.publish(twist)
            else:
                rospy.loginfo(f"[DEBUG] APPROACH_FINE: Twist(lin={twist.linear.x:.3f}, ang={twist.angular.z:.3f})")
            rate.sleep()
            
        return 'preempted'

class Sit(smach.State):
    def __init__(self, data):
        smach.State.__init__(self, outcomes=['finished', 'lost', 'preempted'])
        self.data = data
        self.high_cmd_pub = rospy.Publisher('/high_cmd', HighCmd, queue_size=1)

    def execute(self, userdata):
        rospy.loginfo("Entering State: SIT")
        
        cmd = HighCmd()
        cmd.mode = 1 # Forced stand / control mode
        cmd.bodyHeight = -0.2
        
        if not self.data.debug:
            self.high_cmd_pub.publish(cmd)
        else:
            rospy.loginfo(f"[DEBUG] SIT: HighCmd(mode={cmd.mode}, bodyHeight={cmd.bodyHeight})")
        
        rospy.sleep(2.0) # Wait for crouch
        
        # Maintain state or move to finish? 
        # Requirement says "If at any point the object is lost for 5 loops, return to Idle"
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.data.camera_data is None:
                # Reset posture before going home?
                reset_cmd = HighCmd()
                reset_cmd.mode = 1
                reset_cmd.bodyHeight = 0.0
                if not self.data.debug:
                    self.high_cmd_pub.publish(reset_cmd)
                else:
                    rospy.loginfo("[DEBUG] SIT -> IDLE: Would reset posture.")
                return 'lost'
            rate.sleep()
            
        return 'finished'

def main():
    rospy.init_node('waste_collector_sm')

    data = StateMachineData()
    rospy.Subscriber('/camera_data', Float32MultiArray, data.update_camera)

    # Create SMACH state machine
    sm = smach.StateMachine(outcomes=['finished', 'preempted'])

    with sm:
        smach.StateMachine.add('IDLE', Idle(data), 
                               transitions={'detected':'APPROACH_COARSE', 
                                            'preempted':'preempted'})
        
        smach.StateMachine.add('APPROACH_COARSE', ApproachCoarse(data), 
                               transitions={'centered':'APPROACH_FINE', 
                                            'lost':'IDLE', 
                                            'preempted':'preempted'})
        
        smach.StateMachine.add('APPROACH_FINE', ApproachFine(data), 
                               transitions={'reached':'SIT', 
                                            'lost':'IDLE', 
                                            'preempted':'preempted'})
        
        smach.StateMachine.add('SIT', Sit(data), 
                               transitions={'lost':'IDLE', 
                                            'finished':'finished', 
                                            'preempted':'preempted'})

    # Execute SMACH plan
    outcome = sm.execute()
    rospy.loginfo(f"State Machine Finished with outcome: {outcome}")

if __name__ == '__main__':
    main()
