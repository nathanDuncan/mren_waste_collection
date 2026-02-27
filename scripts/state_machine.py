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

class CorrectYaw(smach.State):
    def __init__(self, data):
        smach.State.__init__(self, outcomes=['aligned', 'lost', 'preempted'])
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
        rospy.loginfo("Entering State: CORRECT YAW")
        
        while not rospy.is_shutdown():
            # Stop linearly, just yaw
            stop_cmd = self.create_high_cmd(mode=1)
            if not self.data.debug:
                self.high_cmd_pub.publish(stop_cmd)
            
            rospy.sleep(1.0) # Wait for camera blur
            
            if self.data.camera_data is None:
                return 'lost'
            
            x_pos = self.data.camera_data[0]
            error_x = 320 - x_pos
            
            if abs(error_x) < 30:
                rospy.loginfo("Yaw aligned.")
                return 'aligned'
            
            # Execute Yaw Burst
            vyaw = error_x * 0.003
            move_cmd = self.create_high_cmd(yaw_speed=vyaw, mode=2, gait_type=1)
            
            if not self.data.debug:
                start_time = rospy.Time.now()
                move_duration = rospy.Duration(0.5)
                rate = rospy.Rate(10)
                while rospy.Time.now() - start_time < move_duration:
                    self.high_cmd_pub.publish(move_cmd)
                    rate.sleep()
                self.high_cmd_pub.publish(stop_cmd)
            else:
                rospy.loginfo(f"[DEBUG] CORRECT YAW: Would yaw at {vyaw:.2f}")
                rospy.sleep(0.5)
                
        return 'preempted'

class ApproachCoarse(smach.State):
    def __init__(self, data):
        smach.State.__init__(self, outcomes=['reach_circle', 'error_yaw', 'lost', 'preempted'])
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
        rospy.loginfo("Entering State: APPROACH (COARSE)")
        
        while not rospy.is_shutdown():
            # Stop to observe
            stop_cmd = self.create_high_cmd(mode=1)
            if not self.data.debug:
                self.high_cmd_pub.publish(stop_cmd)
            
            rospy.sleep(1.0)
            
            if self.data.camera_data is None:
                return 'lost'
            
            x_pos = self.data.camera_data[0]
            y_pos = self.data.camera_data[1]
            dist_meters = self.data.camera_data[2]
            
            error_x = 320 - x_pos
            error_y = 240 - y_pos
            
            # Check for yaw error fallback
            if abs(error_x) > 80:
                rospy.logwarn(f"Yaw error too large ({error_x}), returning to CorrectYaw")
                return 'error_yaw'
            
            # Check if reached circle (1.0m)
            # thresholds and depth is approximately 1 metre
            if 0.9 <= dist_meters <= 1.1 and abs(error_x) < 40 and abs(error_y) < 20:
                rospy.loginfo("Reached 1m circle and centered. Moving to CorrectAngle.")
                return 'reach_circle'

            # Linear approach burst - No Yaw
            kp_dist = 0.5
            target_vel = kp_dist * (dist_meters - 1.0) # Target 1m
            vx = max(min(target_vel, 0.2), -0.2)
            
            # Manipulator Control (Joint 4 Tracking)
            kp_j4 = 0.0005 
            new_j4 = self.data.joint4_pos - (error_y * kp_j4)
            new_j4 = max(min(new_j4, 2.04-0.1), -1.79+0.1)

            move_cmd = self.create_high_cmd(linear_x=vx, mode=2, gait_type=1) # yaw_speed=0
            
            rospy.loginfo(f"APPROACH COARSE: dist={dist_meters:.2f}m, error_x={error_x:.1f}, vx={vx:.2f}")
            
            # Move Arm
            move_manipulator([0.0, -1.0, 0.3, new_j4], path_time=0.5)
            
            if not self.data.debug:
                start_time = rospy.Time.now()
                move_duration = rospy.Duration(0.5)
                rate = rospy.Rate(10)
                while rospy.Time.now() - start_time < move_duration:
                    self.high_cmd_pub.publish(move_cmd)
                    rate.sleep()
                self.high_cmd_pub.publish(stop_cmd)
            else:
                rospy.sleep(0.5)

        return 'preempted'

class CorrectAngle(smach.State):
    def __init__(self, data):
        smach.State.__init__(self, outcomes=['aligned', 'error_dist', 'error_yaw', 'lost', 'preempted'])
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
        rospy.loginfo("Entering State: CORRECT ANGLE")
        
        while not rospy.is_shutdown():
            stop_cmd = self.create_high_cmd(mode=1)
            if not self.data.debug:
                self.high_cmd_pub.publish(stop_cmd)
            
            rospy.sleep(1.0)
            
            if self.data.camera_data is None:
                return 'lost'
            
            x_pos = self.data.camera_data[0]
            dist_meters = self.data.camera_data[2]
            angle_raw = self.data.camera_data[5] # assuming this is the angle in degrees
            
            error_x = 320 - x_pos
            
            # Check Fallbacks
            if dist_meters > 1.2 or dist_meters < 0.8:
                rospy.logwarn(f"Distance out of range ({dist_meters:.2f}), returning to ApproachCoarse")
                return 'error_dist'
            
            if abs(error_x) > 50:
                rospy.logwarn(f"Yaw error too large ({error_x}), returning to CorrectYaw")
                return 'error_yaw'
            
            # Target 0 or 180
            # Wrap angle to -180 to 180 if needed, then find error to 0 or 180
            a = angle_raw % 360
            if a > 180: a -= 360
            
            # Error to 0
            err0 = a
            # Error to 180
            if a > 0: err180 = a - 180
            else: err180 = a + 180
            
            if abs(err0) < abs(err180):
                angle_error = err0
            else:
                angle_error = err180

            if abs(angle_error) < 10 and abs(error_x) < 50:
                rospy.loginfo(f"Angle aligned: error={angle_error:.1f}")
                return 'aligned'
            
            # Move along circle
            # vy determines speed around circle. kp_angle * angle_error
            kp_angle = 0.01 
            vy = -angle_error * kp_angle # Direction might need tuning
            vy = max(min(vy, 0.15), -0.15)
            
            # To stay looking at object at 1m: yaw_speed = vy / R
            # Since R = 1.0, yaw_speed = vy
            # However, check sign. If vy is positive (left), yaw should be positive (CCW) to stay facing center.
            vyaw = vy / 1.0 
            
            move_cmd = self.create_high_cmd(linear_y=vy, yaw_speed=vyaw, mode=2, gait_type=1)
            
            rospy.loginfo(f"CORRECT ANGLE: angle_err={angle_error:.1f}, vy={vy:.2f}, yaw={vyaw:.2f}")
            
            if not self.data.debug:
                start_time = rospy.Time.now()
                move_duration = rospy.Duration(0.5)
                rate = rospy.Rate(10)
                while rospy.Time.now() - start_time < move_duration:
                    self.high_cmd_pub.publish(move_cmd)
                    rate.sleep()
                self.high_cmd_pub.publish(stop_cmd)
            else:
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
                                transitions={'detected':'CORRECT_YAW', 
                                             'preempted':'preempted'})
        
        smach.StateMachine.add('CORRECT_YAW', CorrectYaw(data),
                                transitions={'aligned':'APPROACH_COARSE',
                                             'lost':'IDLE',
                                             'preempted':'preempted'})

        smach.StateMachine.add('APPROACH_COARSE', ApproachCoarse(data), 
                                transitions={'reach_circle':'CORRECT_ANGLE',
                                             'error_yaw':'CORRECT_YAW',
                                             'lost':'IDLE', 
                                             'preempted':'preempted'})
        
        smach.StateMachine.add('CORRECT_ANGLE', CorrectAngle(data),
                                transitions={'aligned':'APPROACH_FINE',
                                             'error_dist':'APPROACH_COARSE',
                                             'error_yaw':'CORRECT_YAW',
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
