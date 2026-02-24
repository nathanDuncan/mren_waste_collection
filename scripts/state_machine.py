#!/usr/bin/env python3

import rospy
import math
import smach
import smach_ros
from sensor_msgs.msg import JointState
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
        
        # Joint state tracking
        self.joint2_pos = JOINT_SET_1[1] # Default to home
        self.joint4_pos = JOINT_SET_1[3]
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)

        if self.debug:
            rospy.loginfo("🛠 DEBUG MODE ENABLED: Base and posture commands will be logged but not published.")

    def joint_state_callback(self, msg):
        try:
            # OpenManipulator joint names are usually joint1, joint2...
            if 'joint2' in msg.name:
                idx = msg.name.index('joint2')
                self.joint2_pos = msg.position[idx]
            if 'joint4' in msg.name:
                idx = msg.name.index('joint4')
                self.joint4_pos = msg.position[idx]
        except (ValueError, IndexError):
            pass

    def update_camera(self, msg):
        if len(msg.data) >= 6 and msg.data[0] != -1.0:
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
        self.high_cmd_pub = rospy.Publisher('/high_cmd', HighCmd, queue_size=1)

    def create_high_cmd(self, linear_x=0, linear_y=0, yaw_speed=0, mode=0, gait_type=0, body_height=0):
        cmd = HighCmd()
        cmd.head = [0xFE, 0xEF]
        cmd.levelFlag = 0xee # HIGHLEVEL
        cmd.mode = mode
        cmd.gaitType = gait_type
        cmd.velocity = [linear_x, linear_y]
        cmd.yawSpeed = yaw_speed
        cmd.bodyHeight = body_height
        return cmd

    def execute(self, userdata):
        rospy.loginfo("Entering State: IDLE")
        
        # Move arm to Home
        move_manipulator(JOINT_SET_1)
        
        # Stop robot - Force Stand
        stop_cmd = self.create_high_cmd(mode=1)
        if not self.data.debug:
            self.high_cmd_pub.publish(stop_cmd)
        else:
            rospy.loginfo("[DEBUG] IDLE: Would publish HighCmd mode=1 (Force Stand)")
        
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
        self.high_cmd_pub = rospy.Publisher('/high_cmd', HighCmd, queue_size=1)

    def create_high_cmd(self, linear_x=0, linear_y=0, yaw_speed=0, mode=0, gait_type=0, body_height=0):
        cmd = HighCmd()
        cmd.head = [0xFE, 0xEF]
        cmd.levelFlag = 0xee # HIGHLEVEL
        cmd.mode = mode
        cmd.gaitType = gait_type
        cmd.velocity = [linear_x, linear_y]
        cmd.yawSpeed = yaw_speed
        cmd.bodyHeight = body_height
        return cmd

    def execute(self, userdata):
        rospy.loginfo("Entering State: APPROACH (COARSE) - Discrete Mode")
        
        while not rospy.is_shutdown():
            # --- PHASE 1: STABILIZE & OBSERVE ---
            # Stop any movement and wait for motion blur to settle
            stop_cmd = self.create_high_cmd(mode=1)
            if not self.data.debug:
                self.high_cmd_pub.publish(stop_cmd)
            
            # rospy.loginfo("PHASE: STABILIZE & OBSERVE (Wait for clear image)")
            rospy.sleep(2.0) # Wait for camera to stabilize
            
            # Verify we still have the object
            if self.data.camera_data is None:
                rospy.logwarn("Object lost during observation.")
                return 'lost'
            
            # Collect stable data (maybe check a few samples?)
            x_pos = self.data.camera_data[0]
            y_pos = self.data.camera_data[1]
            dist_meters = self.data.camera_data[2]
            
            # --- PHASE 2: CALCULATE ---
            # 1. Yaw centering (X error)
            error_x = 320 - x_pos
            # 2. Distance tracking (Linear velocity)
            # Target dist = 0.4
            kp_dist = 0.5
            target_vel = kp_dist * (dist_meters - 0.4)
            
            # 3. Manipulator Control (Joint 4 Tracking)
            error_y = 240 - y_pos
            kp_j4 = 0.0005 
            new_j4 = self.data.joint4_pos - (error_y * kp_j4)
            new_j4 = max(min(new_j4, 2.04-0.1), -1.79+0.1)

            # Check for completion
            if abs(dist_meters - 0.4) < 0.05 and abs(error_x) < 40 and abs(error_y) < 20:
                rospy.loginfo("Target reached and centered.")
                return 'centered'

            # --- PHASE 3: EXECUTE (Short Burst) ---
            vx = max(min(target_vel, 0.2), -0.2)
            vyaw = error_x * 0.003
            move_cmd = self.create_high_cmd(linear_x=vx, yaw_speed=vyaw, mode=2, gait_type=1)
            
            rospy.loginfo(f"PHASE: MOVE - x_err={error_x}, dist_err={dist_meters-0.4:.2f}, j4={new_j4:.3f}")
            
            # Move Arm\
            # rospy.loginfo(f"Moving manipulator joint 4 from {self.data.joint4_pos:.3f} to {new_j4:.3f}")
            move_manipulator([0.0, -1.0, 0.3, new_j4], path_time=0.5)
            
            if not self.data.debug:
                # Move Base for 0.5s
                rospy.loginfo(f"Moving base for 1.0s: lin={vx:.2f}, ang={vyaw:.2f}")
                start_time = rospy.Time.now()
                move_duration = rospy.Duration(1.0)
                rate = rospy.Rate(10)
                while rospy.Time.now() - start_time < move_duration:
                    self.high_cmd_pub.publish(move_cmd)
                    rate.sleep()
                self.high_cmd_pub.publish(stop_cmd)
            else:
                # rospy.loginfo(f"[DEBUG] Would move for 1.0s: lin={vx:.2f}, ang={vyaw:.2f}")
                rospy.sleep(0.5)

        return 'preempted'

class ApproachFine(smach.State):
    def __init__(self, data):
        smach.State.__init__(self, outcomes=['reached', 'lost', 'preempted'])
        self.data = data
        self.high_cmd_pub = rospy.Publisher('/high_cmd', HighCmd, queue_size=1)

    def create_high_cmd(self, linear_x=0, linear_y=0, yaw_speed=0, mode=0, gait_type=0, body_height=0):
        cmd = HighCmd()
        cmd.head = [0xFE, 0xEF]
        cmd.levelFlag = 0xee # HIGHLEVEL
        cmd.mode = mode
        cmd.gaitType = gait_type
        cmd.velocity = [linear_x, linear_y]
        cmd.yawSpeed = yaw_speed
        cmd.bodyHeight = body_height
        return cmd

    def execute(self, userdata):
        rospy.loginfo("Entering State: APPROACH (FINE) - Discrete Mode")
        
        # Move arm to Short View first
        move_manipulator(JOINT_SET_2)
        
        while not rospy.is_shutdown():
            # --- PHASE 1: STABILIZE & OBSERVE ---
            stop_cmd = self.create_high_cmd(mode=1)
            if not self.data.debug:
                self.high_cmd_pub.publish(stop_cmd)
            
            # rospy.loginfo("PHASE: STABILIZE & OBSERVE")
            rospy.sleep(1.0)
            
            if self.data.camera_data is None:
                return 'lost'
            
            x_pos = self.data.camera_data[0]
            y_pos = self.data.camera_data[1]
            
            # --- PHASE 2: CALCULATE ---
            error_x = 320 - x_pos
            error_y = 240 - y_pos 
            
            if abs(error_x) < 10 and abs(error_y) < 10:
                rospy.loginfo("Centering complete.")
                return 'reached'
            
            # --- PHASE 3: EXECUTE (Short Burst) ---
            vx = error_y * 0.001
            vyaw = error_x * 0.001
            move_cmd = self.create_high_cmd(linear_x=vx, yaw_speed=vyaw, mode=2, gait_type=1)
            
            # rospy.loginfo(f"PHASE: MOVE - error_x={error_x}, error_y={error_y}")
            
            if not self.data.debug:
                start_time = rospy.Time.now()
                move_duration = rospy.Duration(0.5)
                rate = rospy.Rate(10)
                while rospy.Time.now() - start_time < move_duration:
                    self.high_cmd_pub.publish(move_cmd)
                    rate.sleep()
                self.high_cmd_pub.publish(stop_cmd)
            else:
                # rospy.loginfo(f"[DEBUG] Would move for 0.5s: lin={vx:.3f}, ang={vyaw:.3f}")
                rospy.sleep(0.5)
            
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
