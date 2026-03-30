#!/usr/bin/env python3

import rospy
import math
import smach
import smach_ros
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Twist, Quaternion
from unitree_legged_msgs.msg import HighCmd
from open_manipulator_msgs.msg import JointPosition, KinematicsPose
from open_manipulator_msgs.srv import SetJointPosition, SetKinematicsPose
from master_control_script import Controller

# Joint Presets
JOINT_HOME = [0.0, -1.0, 0.3, 0.7]
JOINT_SCAN_START = [-1.0, -1.0, 0.3, 0.7] # joint1 at -1.0  3## Dear Daniel, does this look right? Yeah, could prob move joint 1 further too
JOINT_SCAN_END = [1.0, -1.0, 0.3, 0.7]   # joint1 at 1.0

TIME = 1.0
JOINT_SPACE_TIME = 3.0
DROP_THRESHOLD = 0.00
H = 480
W = 640

controller = None

class StateMachineData:
    def __init__(self):
        self.camera_data = None
        self.lost_count = 0
        self.max_lost = 5
        self.debug = rospy.get_param('~debug_mode', False)
        
        # Joint state tracking
        self.joint_states = [0.0, 0.0, 0.0, 0.0] # j1, j2, j3, j4
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        
        # Message handling
        self.idle_trigger = False
        rospy.Subscriber('/idle_cmd', String, self.idle_callback)

    def joint_state_callback(self, msg):
        try:
            # OpenManipulator joint names: joint1, joint2, joint3, joint4
            indices = {name: i for i, name in enumerate(msg.name)}
            for i in range(1, 5):
                name = f'joint{i}'
                if name in indices:
                    self.joint_states[i-1] = msg.position[indices[name]]
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

    def idle_callback(self, msg):
        if msg.data == "start" or msg.data == "grab":
            self.idle_trigger = msg.data

def move_manipulator(joint_angles, path_time=2.0):
    rospy.wait_for_service('/goal_joint_space_path', timeout=5.0)
    try:
        service = rospy.ServiceProxy('/goal_joint_space_path', SetJointPosition)
        joint_msg = JointPosition()
        joint_msg.joint_name = ["joint1", "joint2", "joint3", "joint4"]
        joint_msg.position = joint_angles
        resp = service(planning_group="arm", joint_position=joint_msg, path_time=path_time)
        rospy.sleep(path_time)
        return resp.is_planned
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logerr(f"Manipulator service call failed: {e}")
        return False

def project_location(camera_data):
    """Stub for projecting pixel data to GO1 X,Y."""
    rospy.loginfo(f"Projecting location for object at {camera_data[0]}, {camera_data[1]}")
    controller.project_object()
    location = [controller.can_global_pos[0], controller.can_global_pos[1], -0.1]

    return location # Returns dummy 3D coords # 3rd coord would be angle

def send_message(data):
    """Stub for sending data to other nodes."""
    rospy.loginfo(f"Sending message: {data}")
    # Dear Daniel, here is where we will message pi5

# --- States ---

class Scan(smach.State):
    def __init__(self, data):
        smach.State.__init__(self, outcomes=['detected', 'finished', 'preempted'])
        self.data = data
        self.sweep_count = 0
        self.direction = 1 # 1 for right, -1 for left

    def execute(self, userdata):
        rospy.loginfo("Entering State: SCAN")
        self.sweep_count = 0
        
        while not rospy.is_shutdown() and self.sweep_count < 2:
            # Target joint1 position
            target_j1 = 1.0 if self.direction == 1 else -1.0
            target_angles = [target_j1, -1.0, 0.3, 0.7]
            
            rospy.loginfo(f"Scanning: Sweep {self.sweep_count+1}/2, direction: {'Right' if self.direction == 1 else 'Left'}")
            
            # Start moving to target
            # Note: Using small steps to allow checking for camera_data during movement
            steps = 20
            start_j1 = self.data.joint_states[0]
            for i in range(steps):
                if rospy.is_shutdown(): return 'preempted'
                
                if self.data.camera_data is not None:
                    rospy.loginfo("Object detected during SCAN!")
                    return 'detected'
                
                current_target_j1 = start_j1 + (target_j1 - start_j1) * (float(i+1)/steps)
                move_manipulator([current_target_j1, -1.0, 0.3, 0.7], path_time=0.1)
                
            # Completed one side of the sweep
            self.direction *= -1
            if self.direction == 1: # Returned to start side (or completed a full cycle)
                self.sweep_count += 1
                
        if self.sweep_count >= 2:
            rospy.loginfo("SCAN finished 2 sweeps.")
            return 'finished'
            
        return 'preempted'

class Measure(smach.State):
    def __init__(self, data):
        smach.State.__init__(self, outcomes=['done', 'lost', 'preempted'])
        self.data = data

    def execute(self, userdata):
        rospy.loginfo("Entering State: MEASURE")
        
        # Centering logic (P-control)
        kp_x = 0.0005
        kp_y = 0.0005

        controller.mode = "search"
        controller.start_scan()
        '''
        while not rospy.is_shutdown():
            if self.data.camera_data is None:
                rospy.logwarn("Object lost during MEASURE")
                return 'lost'
            
            x_pos = self.data.camera_data[0]
            y_pos = self.data.camera_data[1]
            
            error_x = 320 - x_pos
            error_y = 240 - y_pos
            
            if abs(error_x) < 20 and abs(error_y) < 20:
                rospy.loginfo("Object centered.")
                break
                
            # Adjust joint1 (horizontal) and joint4 (vertical)
            new_j1 = self.data.joint_states[0] + (error_x * kp_x)
            new_j4 = self.data.joint_states[3] - (error_y * kp_y) # Y-axis inverted in image?
            
            move_manipulator([new_j1, -1.0, 0.3, new_j4], path_time=0.2)
            rospy.sleep(0.1)
        '''
            
        # Call auxiliary functions
        loc = project_location(self.data.camera_data)
        send_message(f"Object at {loc}")
        
        return 'done'

class Idle(smach.State):
    def __init__(self, data):
        smach.State.__init__(self, outcomes=['start_scan', 'grab', 'preempted'])
        self.data = data

    def execute(self, userdata):
        rospy.loginfo("Entering State: IDLE")
        self.data.idle_trigger = False

        # Dear Daniel, here the robot waits for commands from the pi5
        
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.data.idle_trigger == "start":
                return 'start_scan'
            elif self.data.idle_trigger == "grab":
                return 'grab'
            rate.sleep()
            
        return 'preempted'

class Grab(smach.State):
    def __init__(self, data):
        smach.State.__init__(self, outcomes=['finished', 'preempted'])
        self.data = data
        self.high_cmd_pub = rospy.Publisher('/high_cmd', HighCmd, queue_size=1)

    def execute(self, userdata):
        rospy.loginfo("Entering State: GRAB")
        
        # 1. Sit the robot
        sit_cmd = HighCmd()
        sit_cmd.head = [0xFE, 0xEF]
        sit_cmd.levelFlag = 0xee
        sit_cmd.mode = 5 # SIT
        
        rospy.loginfo("Sending SIT command...")
        self.high_cmd_pub.publish(sit_cmd)
        rospy.sleep(3.0) # Wait for sit animation
        
        # 2. Perform Pickup
        rospy.loginfo("Performing arm pickup sequence...")
        # Dear Daniel, please put your pickup code here.
        controller.mode = "pickup"
        controller.start_pickup()
        
        rospy.loginfo("Pickup sequence complete.")
        return 'finished'

def main():
    global controller
    rospy.init_node('pi4_state_machine')
    controller = Controller()
    
    data = StateMachineData()
    rospy.Subscriber('/camera_data', Float32MultiArray, data.update_camera)
    
    # Create SMACH state machine
    sm = smach.StateMachine(outcomes=['shutdown'])
    
    with sm:
        smach.StateMachine.add('SCAN', Scan(data), 
                                transitions={'detected':'MEASURE',
                                             'finished':'IDLE',
                                             'preempted':'shutdown'})
        
        smach.StateMachine.add('MEASURE', Measure(data),
                                transitions={'done':'SCAN', # Should this go to IDLE when done, not SCAN?
                                             'lost':'SCAN',
                                             'preempted':'shutdown'})
        
        smach.StateMachine.add('IDLE', Idle(data),
                                transitions={'start_scan':'SCAN',
                                             'grab':'GRAB',
                                             'preempted':'shutdown'})
        
        smach.StateMachine.add('GRAB', Grab(data),
                                transitions={'finished':'IDLE',
                                             'preempted':'shutdown'})

    # Execute SMACH plan
    outcome = sm.execute()
    rospy.loginfo(f"State Machine Finished with outcome: {outcome}")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
