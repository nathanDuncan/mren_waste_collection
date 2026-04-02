#!/usr/bin/env python3

import rospy
import math
import smach
import smach_ros
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Twist, Quaternion
try:
    from unitree_legged_msgs.msg import HighCmd
except ImportError:
    HighCmd = None

import socket
import threading
import goal_position_pb2
import states_pb2
from open_manipulator_msgs.msg import JointPosition, KinematicsPose
from open_manipulator_msgs.srv import SetJointPosition, SetKinematicsPose
from master_control_script import Controller

PI5_IP = "192.168.12.188"
PI5_GOAL_PORT = 25001
PI5_STATE_PORT = 25002

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
        if HighCmd is None:
            if not self.debug:
                rospy.logwarn("unitree_legged_msgs not found. Forcing debug mode.")
            self.debug = True
        
        if self.debug:
            rospy.loginfo("🛠 DEBUG MODE ENABLED: Unitree commands will be logged but not published.")
        
        # Joint state tracking
        self.joint_states = [0.0, 0.0, 0.0, 0.0] # j1, j2, j3, j4
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        
        # Message handling
        self.idle_trigger = False
        
        # Setup UDP socket for idle commands
        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.cmd_sock.bind(("0.0.0.0", 25002))
            self.cmd_sock.settimeout(0.5)
            rospy.loginfo("UDP Socket bound to port 25002 for idle commands")
        except Exception as e:
            rospy.logerr(f"Failed to bind socket: {e}")

        self.listen_thread = threading.Thread(target=self.socket_listener)
        self.listen_thread.daemon = True
        self.listen_thread.start()

    def socket_listener(self):
        """Thread to listen for Protobuf commands on port 25002."""
        while not rospy.is_shutdown():
            try:
                data, addr = self.cmd_sock.recvfrom(1024)
                if addr[0] != PI5_IP and PI5_IP != "192.168.12.188": # Optional IP filtering if needed
                     # rospy.logdebug(f"Received from {addr[0]}")
                     pass
                
                msg = states_pb2.States()
                msg.ParseFromString(data)
                
                if msg.action in ["sit", "stand"]:
                    rospy.loginfo(f"Received command via Protobuf: {msg.action}")
                    self.idle_trigger = msg.action
            except socket.timeout:
                continue
            except Exception as e:
                rospy.logerr(f"Socket listener error: {e}")
                rospy.sleep(1.0)

    def send_state(self, action):
        """Sends a state message using the already bound cmd_sock."""
        try:
            msg = states_pb2.States()
            msg.action = action
            serialized_data = msg.SerializeToString()
            self.cmd_sock.sendto(serialized_data, (PI5_IP, PI5_STATE_PORT))
            rospy.loginfo(f"Message sent to pi5: {action}")
        except Exception as e:
            rospy.logerr(f"Failed to send state: {e}")

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
        """No longer used for ROS subscriber, but kept for logic reference if needed."""
        if msg.data == "start" or msg.data == "grab" or msg.data == "sit" or msg.data == "stand":
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
    location = [controller.can_global_pos[0], controller.can_global_pos[1], 0.0]
    rospy.loginfo(f"[DEBUG] location: {location}")
    return location 

def send_message(location):
    """Sends the location to Pi5 using goal_position.proto."""
    rospy.loginfo(f"Sending goal to Pi5: {location}")
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        goal = goal_position_pb2.GoalPosition()
        goal.x = location[0]
        goal.y = location[1]
        goal.theta = location[2]
        
        serialized_data = goal.SerializeToString()
        sock.sendto(serialized_data, (PI5_IP, PI5_GOAL_PORT))
        rospy.loginfo(f"Goal successfully sent to {PI5_IP}:{PI5_GOAL_PORT}")
        sock.close()
    except Exception as e:
        rospy.logerr(f"Failed to send goal to Pi5: {e}")

# --- States ---

class Scan(smach.State):
    def __init__(self, data):
        smach.State.__init__(self, outcomes=['detected', 'finished', 'preempted'])
        self.data = data
        self.sweep_count = 0

    def execute(self, userdata):
        rospy.loginfo("Entering State: SCAN")
        
        # 1. Initial entrance: Move to Home and hold for 3 seconds
        rospy.loginfo("SCAN: Moving to Home position...")
        move_manipulator(JOINT_HOME, path_time=2.0)
        rospy.loginfo("SCAN: Holding at Home for 3 seconds...")
        
        start_hold = rospy.Time.now()
        while (rospy.Time.now() - start_hold).to_sec() < 3.0:
            if rospy.is_shutdown(): return 'preempted'
            if self.data.camera_data is not None:
                rospy.loginfo("Object detected during initial hold!")
                return 'detected'
            rospy.sleep(0.1)
            
        self.sweep_count = 0
        while not rospy.is_shutdown() and self.sweep_count < 2:
            rospy.loginfo(f"SCAN: Starting sweep cycle {self.sweep_count+1}/2 (Home -> Left -> Right -> Home)")
            
            # Phase 1: Home (0.0) -> Left (-1.0) in 3 seconds
            if self.move_with_detection(0.0, -1.0, 3.0) == 'detected': return 'detected'
            
            # Phase 2: Left (-1.0) -> Right (1.0) in 6 seconds
            if self.move_with_detection(-1.0, 1.0, 6.0) == 'detected': return 'detected'
            
            # Phase 3: Right (1.0) -> Home (0.0) in 3 seconds
            if self.move_with_detection(1.0, 0.0, 3.0) == 'detected': return 'detected'
            
            self.sweep_count += 1
                
        if self.sweep_count >= 2:
            rospy.loginfo("SCAN finished 2 sweep cycles.")
            return 'finished'
            
        return 'preempted'

    def move_with_detection(self, start_j1, end_j1, duration):
        """Helper to move joint1 while checking for camera data."""
        steps = int(duration / 0.1)
        for i in range(steps):
            if rospy.is_shutdown(): return 'preempted'
            
            if self.data.camera_data is not None:
                rospy.loginfo("Object detected during SCAN move!")
                return 'detected'
            
            current_target_j1 = start_j1 + (end_j1 - start_j1) * (float(i+1)/steps)
            move_manipulator([current_target_j1, -1.0, 0.3, 0.7], path_time=0.1)
        return 'continue'

class Measure(smach.State):
    def __init__(self, data):
        smach.State.__init__(self, outcomes=['done', 'lost', 'preempted'])
        self.data = data

    def execute(self, userdata):
        rospy.loginfo("Entering State: MEASURE")
        
        # Centering logic (P-control)
        kp_x = 0.0006
        kp_y = 0.0008

        # controller.mode = "search"
        # controller.start_scan()
        
        while not rospy.is_shutdown():
            if self.data.camera_data is None:
                rospy.logwarn("Object lost during MEASURE")
                return 'lost'
            
            x_pos = self.data.camera_data[0]
            y_pos = self.data.camera_data[1]
            
            error_x = 320 - x_pos
            error_y = 240 - y_pos
            rospy.loginfo(f"Error: {error_x}, {error_y}")
            
            if abs(error_x) < 15 and abs(error_y) < 15:
                rospy.loginfo("Object centered.")
                rospy.sleep(0.5)
                break
                
            # Adjust joint1 (horizontal) and joint4 (vertical)
            new_j1 = self.data.joint_states[0] + (error_x * kp_x)
            new_j4 = self.data.joint_states[3] - (error_y * kp_y)
            
            move_manipulator([new_j1, -1.0, 0.3, new_j4], path_time=0.1)
            rospy.sleep(0.1)
        
            
        # Call auxiliary functions
        loc = project_location(self.data.camera_data)
        send_message(loc)
        
        return 'done'

class Idle(smach.State):
    def __init__(self, data):
        smach.State.__init__(self, outcomes=['start_scan', 'grab', 'preempted'])
        self.data = data

        # pi4_state_machine.py - line 210
# Ensure this is the ONLY subscriber to /camera_data
# rospy.Subscriber('/camera_data', Float32MultiArray, data.update_camera)

    def execute(self, userdata):
        rospy.loginfo("Entering State: IDLE")
        self.data.idle_trigger = False

        # Dear Daniel, here the robot waits for commands from the pi5
        
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.data.idle_trigger == "stand":
                return 'start_scan'
            elif self.data.idle_trigger == "sit":
                return 'grab'
            rate.sleep()
            
        return 'preempted'

class Grab(smach.State):
    def __init__(self, data):
        smach.State.__init__(self, outcomes=['finished', 'preempted'])
        self.data = data
        if HighCmd is not None:
            self.high_cmd_pub = rospy.Publisher('/high_cmd', HighCmd, queue_size=1)
        else:
            self.high_cmd_pub = None

    def execute(self, userdata):
        rospy.loginfo("Entering State: GRAB")
        
        rospy.sleep(2.0)
        
        # 2. Perform Pickup
        rospy.loginfo("Performing arm pickup sequence...")
        # Dear Daniel, please put your pickup code here.
        controller.mode = "pickup"
        controller.start_pickup()
        
        rospy.loginfo("Pickup sequence complete.")
        
        # 3. Inform Pi5 that the can has been picked up
        self.data.send_state("can picked up")

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
                                transitions={'done':'IDLE', # Should this go to IDLE when done, not SCAN?
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
