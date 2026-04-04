#!/usr/bin/env python3

import rospy
import json
from open_manipulator_msgs.msg import KinematicsPose, JointPosition
from open_manipulator_msgs.srv import SetKinematicsPose, SetJointPosition
from std_msgs.msg import Float32MultiArray, String
from sensor_msgs.msg import JointState
import copy
import math
import numpy as np
from scipy.spatial.transform import Rotation as R


# Subscribe to camera node for D (depth to can)
# Subscribe to EE pose for quaternion, then convert to unit vector

TIME = 1.0
JOINT_SPACE_TIME = 3.0
DROP_THRESHOLD = 0.00
H = 480
W = 640

class Controller:
    """
    Access point to control the OpenManipulator X arm using joint/task space commands.
    """

    def __init__(self):
        # rospy.init_node("manipulator_controller")

        # TODO: Get rid of instance variables that do not need to be class level
        # Manipulator States
        self.current_pose = None
        self.current_joint_state = None

        # TODO: I think dont need these to be class variables. Variables for Mapping Calculations
        self.unit_vector = None
        self.height = None

        # Can Pos in pixels
        self.cx = None
        self.cy = None
        
        # Can physical params
        self.can_height = None
        self.can_width = None
        self.can_angle = None

        self.can_dist = None

        # Can global position
        self.can_global_pos = None
        self.can_global_x_pos = None
        self.can_global_y_pos = None

        # Status Flags for manipulator
        self.centered = 0
        self.status = None
        self.object_found = False
        self.mode = None

        
        # Timer for timed interval print statements
        self.timer = rospy.Timer(rospy.Duration(3.0), self.timer_callback)

        # Subscriptions
        rospy.Subscriber(
            "/gripper/kinematics_pose", KinematicsPose, self._pose_callback
        )
        rospy.Subscriber("/joint_states", JointState, self._joint_state_callback)
        rospy.Subscriber(
            "/camera_data", Float32MultiArray, callback=self._camera_subscriber_callback
        )

        rospy.wait_for_service("/goal_joint_space_path_from_present")
        rospy.wait_for_service("/goal_joint_space_path")
        rospy.wait_for_service("/goal_task_space_path_from_present_position_only")
        rospy.wait_for_service("/goal_tool_control")


        self._delta_joint_srv = rospy.ServiceProxy(
            "/goal_joint_space_path_from_present", SetJointPosition
        )

        self._abs_joint_srv = rospy.ServiceProxy(
            "/goal_joint_space_path", SetJointPosition
        )

        self._delta_task_srv = rospy.ServiceProxy(
            "/goal_task_space_path_from_present_position_only", SetKinematicsPose
        )
        self._abs_task_srv = rospy.ServiceProxy(
            "/goal_task_space_path", SetKinematicsPose
        )
        self._gripper_srv = rospy.ServiceProxy("/goal_tool_control", SetJointPosition)


    # Callbacks
    def _pose_callback(self, msg: KinematicsPose):
        self.current_pose = msg.pose
        if self.object_found:
            self.project_object()

    def timer_callback(self, event):
        pass
        # if self.mode == "search":
        #     print(self.status)
        #     if self.status == "Centered":
        #         self.centered += 1
        #     else:
        #         self.centered = 0
        #     print(f"Target positioned at: {self.can_global_pos}")

    def _joint_state_callback(self, msg: JointState):
        target_joints = ["joint1", "joint2", "joint3", "joint4"]
        current_angles = []
        for joint_name in target_joints:
            if joint_name in msg.name:
                index = msg.name.index(joint_name)
                current_angles.append(msg.position[index])

        if len(current_angles) == 4:
            self.current_joint_state = current_angles

    def _camera_subscriber_callback(self, msg):
        if len(msg.data) >= 6 and msg.data[0] != -1.0:
            # receiver.py mapping: [x_pos, y_pos, dist_meters, width_cm, length_cm, angle]
            self.object_found = True
            self.cx = msg.data[0]
            self.cy = msg.data[1]
            self.can_dist = msg.data[2]
            self.can_width = msg.data[3]
            self.can_height = msg.data[4]
            self.can_angle = msg.data[5]
        else:
            self.object_found = False
            self.cx = None
            self.cy = None
    
    def project_object(self):
        # 1. Get End Effector (EE) global position and rotation
        ee_global_pos = np.array([self.current_pose.position.x, self.current_pose.position.y, self.current_pose.position.z])
        quaternion = [self.current_pose.orientation.x, self.current_pose.orientation.y, self.current_pose.orientation.z, self.current_pose.orientation.w]
        ee_rotation = R.from_quat(quaternion)

        # 2. Define the Camera's local tilt (30 degrees down towards the floor)
        camera_tilt_angle = np.radians(30)
        ee_to_camera_R = np.array([
            [np.cos(camera_tilt_angle), 0.0, np.sin(camera_tilt_angle)],
            [0.0, 1.0, 0.0],
            [-np.sin(camera_tilt_angle), 0.0, np.cos(camera_tilt_angle)]
        ])

        # 3. TODO: Define the Camera's local position offset from the EE
        # (3cm back, 4cm up)
        ee_to_camera_T = np.array([-0.025, 0.0, 0.03])

        # --- CALCULATE GLOBAL CAMERA DIRECTION ---
        local_forward = np.array([1.0, 0.0, 0.0])
        # First apply the camera's own tilt, then apply the EE's global rotation
        camera_local_direction = ee_to_camera_R @ local_forward
        camera_global_direction = ee_rotation.apply(camera_local_direction)

        # --- CALCULATE GLOBAL CAMERA POSITION ---
        # Rotate the local offset into the global frame, then add to EE's global position
        camera_global_offset = ee_rotation.apply(ee_to_camera_T)
        camera_global_pos = ee_global_pos + camera_global_offset

        # --- CALCULATE TARGET POSITION (The Can) ---
        # Start at the camera's lens, and move forward along its line of sight by the depth distance
        self.can_global_pos = camera_global_pos + (camera_global_direction * self.can_dist)
        phi = math.atan2(self.can_global_pos[1], self.can_global_pos[0])
        alpha = phi + self.current_joint_state[0]
        u = [math.cos(alpha), math.sin(alpha)]
        # print("Object located at: ", self.can_global_pos)
        self.can_global_pos[0] = self.can_global_pos[0] - 0.25*u[0]
        self.can_global_pos[1] = self.can_global_pos[1] - 0.25*u[1]
        self.can_global_pos[2] = alpha
        # print("Moving to position: ", self.can_global_pos)


    # Helper functions to move in joint/task space absolute/delta
    def move_joint_space_absolute(self, joint_angles, path_time: float = TIME):
        """Command the arm to move to specific absolute joint angles."""
        msg = JointPosition()
        msg.joint_name = ["joint1", "joint2", "joint3", "joint4"]
        msg.position = joint_angles

        try:
            resp = self._abs_joint_srv(
                planning_group="arm", joint_position=msg, path_time=path_time
            )
            rospy.loginfo(f"Absolute joint move planned: {resp.is_planned}")
            rospy.sleep(path_time)
            return resp.is_planned
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False
    
    def move_joint_space_delta(self, joint_deltas, path_time: float = TIME):
        """Shift joints by a relative amount (Emulates teleop 'y'/'h' keys)"""
        msg = JointPosition()
        msg.joint_name = ["joint1", "joint2", "joint3", "joint4"]
        msg.position = joint_deltas

        try:
            resp = self._delta_joint_srv(
                planning_group="arm", joint_position=msg, path_time=path_time
            )
            rospy.sleep(path_time + 1.0)
            return resp.is_planned
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False
    
    def move_task_space_delta(self, dx=0.0, dy=0.0, dz=0.0, dt=1.0):
        """Move a small Cartesian increment (Emulates teleop 'w'/'a'/'s'/'d' keys)"""
        msg = KinematicsPose()
        msg.pose.position.x = dx
        msg.pose.position.y = dy
        msg.pose.position.z = dz

        try:
            # We must use "gripper" as planning group for this specific service
            resp = self._delta_task_srv(
                planning_group="gripper", kinematics_pose=msg, path_time=dt
            )

            if resp.is_planned:
                rospy.sleep(dt)
                if self.current_pose:
                    print(
                        f"Move successful. Current Z is roughly: {self.current_pose.position.z:.3f}"
                    )
                    rospy.sleep(1.0)
                return True
            else:
                rospy.logwarn("Kinematic limit reached! Ignoring command.")
                return False

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False

    def move_task_space_absolute(self, pose, path_time: float = TIME):
        """Command the arm to move to specific absolute joint angles."""
        msg = KinematicsPose()
        msg.pose = pose

        try:
            resp = self._abs_task_srv(
                planning_group="arm", kinematics_pose=msg, path_time=path_time
            )
            if resp.is_planned:
                rospy.sleep(path_time)
                if self.current_pose:
                    print(
                        f"Move successful. Current Z is roughly: {self.current_pose.position.z:.3f}"
                    )
                    rospy.sleep(1.0)
                return True
            else:
                rospy.logwarn("Kinematic limit reached! Ignoring command.")
                return False

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False
        
    # Directional movement
    def move_left(self, amt = 1, t = TIME):
        print("Moving Left (Joint Space)")
        return self.move_joint_space_delta(
            joint_deltas=[0.05*amt, 0.0, 0.0, 0.0], path_time=t*amt
        )

    def move_right(self, amt = 1, t = TIME):
        print("Moving Right (Joint Space)")
        return self.move_joint_space_delta(
            joint_deltas=[-0.05*amt, 0.0, 0.0, 0.0], path_time=t*amt
        )

    def move_up_joint_space(self, amt = 1, t = TIME):
        print("Moving Up (Joint Space)")
        return self.move_joint_space_delta(
            joint_deltas=[0.0, 0.0, 0.0, -0.05*amt], path_time=t*amt
        )

    def move_down_joint_space(self, amt = 1, t = TIME):
        print("Moving Down (Joint Space)")
        return self.move_joint_space_delta(
            joint_deltas=[0.0, 0.0, 0.0, 0.05*amt], path_time=t*amt
        )
    
    def move_up_task_space(self, amt = 1, t = TIME):
        return self.move_task_space_delta(dx=0.0, dy=0.0, dz=0.003*amt, dt=t*amt / 2)

    def move_down_task_space(self, amt = 1, t = TIME):
        return self.move_task_space_delta(dx=0.0, dy=0.0, dz=-0.005*amt, dt=t*amt/3)

    def move_forward_task_space(self, amt = 1, t = TIME):
        return self.move_task_space_delta(dx=0.01*amt, dy=0.0, dz=0.0, dt=t*amt)

    def move_backward_task_space(self, amt = 1, t = TIME):
        return self.move_task_space_delta(dx=-0.01*amt, dy=0.0, dz=0.0, dt=t*amt)
    
    # Gripper Functions
    def close_gripper(self):
        msg = JointPosition()
        msg.joint_name = ["gripper"]
        msg.position = [-0.010]
        try:
            self._gripper_srv(
                planning_group="gripper", joint_position=msg, path_time=0.5
            )
            rospy.sleep(1.0)
        except:
            pass

    def open_gripper(self):
        msg = JointPosition()
        msg.joint_name = ["gripper"]
        msg.position = [0.020]
        try:
            self._gripper_srv(
                planning_group="gripper", joint_position=msg, path_time=0.5
            )
            rospy.sleep(1.0)
        except:
            pass
    
    # Preset Joint Positions
    def move_home(self):
        return self.move_joint_space_absolute([0.0, -1.0, 0.3, 0.7], JOINT_SPACE_TIME)

    def move_stable(self):
        return self.move_joint_space_absolute(
            [0.0, -1.61, 1.320, 0.371], JOINT_SPACE_TIME
        )

    def move_to_dropoff(self):
        return self.move_joint_space_absolute(
            [0.0, -1.159, -0.711, -1.320], JOINT_SPACE_TIME
        )

    def move_to_start(self):
        return self.move_joint_space_absolute(
            [0.0, 0.276, -0.120, 1.20], JOINT_SPACE_TIME
        )

    # Recovery methods when no object detected
    def search(self):
        print("Searching")
        inc = 1
        while(not self.object_found):
            for j in range(4):
                if(self.object_found):
                    break
                if j == 0:
                    self.move_down_joint_space(inc)
                elif j == 1:
                    self.move_right(inc)
                    inc += 1
                elif j == 2:
                    self.move_up_joint_space(inc)
                elif j == 3:
                    self.move_left(inc)
                    inc += 1
                
            if(self.object_found):
                break
                
        print("Object found after searching")

    def wiggle(self):
        #TODO: Create a wiggle function that makes the arm move in an increasingly large circular trajectory until an object is found
        lost = self.cx == None
        dist = 1
        while(lost):
            print("WIGGLING")
            for j in range(5):
                lost = self.cx == None
                if not lost: break
                if j == 0:
                    self.move_joint_space_delta([0.0, 0.0, 0.0, -0.07], TIME)
                elif j == 1:
                    self.move_forward_task_space(dist)
                elif j == 2:
                    self.move_right(dist)
                    dist += 1
                elif j == 3:
                    self.move_backward_task_space(dist)
                elif j == 4:
                    self.move_left(dist)
                    dist += 1
                rospy.sleep(3*TIME)
            if not lost: break
            # prev start joint 4 was 1.328
        while(self.current_joint_state[3] < 1.20+3*-0.05):
            self.move_joint_space_delta([0.0, 0.0, 0.0, 0.03])

    # Main methods
    def start_scan(self):
        rospy.sleep(3.0)
        y_threshold_percentage = 0.50
        y_threshold_tolerance = 0.03
        # Since camera is not perfectly centered
        x_threshold_percentage = 0.53
        x_threshold_tolerance = 0.03

        lower_y = (y_threshold_percentage * H) - (y_threshold_tolerance * H)
        upper_y = (y_threshold_percentage * H) + (y_threshold_tolerance * H)
        lower_x = (x_threshold_percentage * W) - (x_threshold_tolerance * W)
        upper_x = (x_threshold_percentage * W) + (x_threshold_tolerance * W)

        # Set a loop rate (e.g., 10 Hz / 10 times a second)
        rate = rospy.Rate(10)
        
        rospy.loginfo("Starting scan. Press Ctrl+C to cleanly exit.")

        # This naturally breaks when you press Ctrl+C
        while not rospy.is_shutdown() and self.centered < 3:
            
            if not self.object_found:
                self.status = "No Object Detected"
                self.search()

            else:
                amt_x = abs(self.cx - y_threshold_percentage*W)/(W/2)
                amt_y = abs(self.cy - x_threshold_percentage*H)/(H/2)
                k_P = 4
                amt_x *= k_P
                amt_y *= k_P

                if self.cx < lower_x:
                    self.move_left(amt_x)
                    # self.status = "Moving Left"

                elif self.cx > upper_x:
                    self.move_right(amt_x)
                    # self.status = "Moving Right"

                elif self.cy < lower_y:
                    self.move_up_joint_space(amt_y)
                    # self.status = "Moving Up"

                elif self.cy > upper_y:
                    self.move_down_joint_space(amt_y)
                    # self.status = "Moving Down"
                
                else:
                    self.status = "Centered"
                
            # Crucial: Yield the CPU so ROS can process callbacks and shutdown signals
            rate.sleep()
            
        print("\nShutdown signal received. Exiting scan loop.")
    
    def _pick_up_and_drop_off(self):
        self.close_gripper()
        self.open_gripper()
        print("moving down")

        height = 0.13 + self.current_pose.position.z
        print(height)

        # TODO: Sometimes this IK solve fails and gripper just skips this, but should fix
        target_pose = self.current_pose
        target_pose.position.z -= height
        print("Trying to reach pose: ", target_pose)
        move_success = self.move_task_space_delta(
                dx=0.0, dy=0.0, dz=-height, dt=5 * TIME
            )
        back_count = 0
        while move_success == False:
            # Mark the location we need the arm to go to
            print("Initial task space move failed, trying local area")
            self.move_backward_task_space()
            back_count += 1
            move_success = self.move_task_space_delta(
                dx=0.0, dy=0.0, dz=-height, dt=5 * TIME
            )
        print("moved down")
        # TODO: maybe if was prev not y centered, move as far forward as possible
        print("Trying to recenter by ", back_count, "increments")
        # for a in range(back_count):
        #     move_success = self.move_forward()
        #     if move_success: print("Moved forward successfully one increment")
        
        if(back_count != 0):
            while(True):
                move_success = self.move_forward_task_space()
                if not move_success: break
            #TODO: Change this to absolute pose as in current but absolute joint4 cause inconsistent with wiggle, 
            # also make this based on ee pose (further) out = more tilt
            print("Dist: ", np.linalg.norm([self.current_pose.position.x, self.current_pose.position.y]))
            if(np.linalg.norm([self.current_pose.position.x, self.current_pose.position.y]) > 0.230):
                self.move_joint_space_delta([0.0, 0.0, 0.0, -0.7], TIME)
            # TODO: Also this
                for i in range(6):
                    self.move_forward_task_space()

        self.close_gripper()
        self.move_home()
        self.move_to_dropoff()
        self.open_gripper()
        self.move_stable()
        return

    def start_pickup(self):
        y_move_success = True
    # joint_angles = [0.0037, 0.1537, 0.1012, 1.2547]
        safe_to_pickup = True

        try:
            while not rospy.is_shutdown() and safe_to_pickup:
                start_flag = input("Type 's' to start (Ctrl+C to quit): ")
                if start_flag != "s":
                    continue

                # Setup
                self.move_to_start()
                rospy.sleep(0.2)
                self.open_gripper()

                prev_move = None
                lost_object = False
                # Enter active tracking loop
                while not rospy.is_shutdown():
                    if self.cx is None:
                        lost_object = True
                    else:
                        lost_object = False

                    if lost_object and prev_move != "down":
                        # The robot freezes in this case
                        print("no visible object and prev move not down")
                        self.wiggle()
                        continue
                    elif lost_object and prev_move == "down":
                        print("lost object, Moving down")
                        if self.current_pose.position.z > DROP_THRESHOLD:
                            self.move_down_task_space()
                        y_centered = True
                        prev_move = "down"

                    y_threshold_percentage = 0.40
                    y_threshold_tolerance = 0.05
                    # Since camera is not perfectly centered
                    x_threshold_percentage = 0.53
                    x_threshold_tolerance = 0.05

                    lower_y = (y_threshold_percentage * H) - (y_threshold_tolerance * H)
                    upper_y = (y_threshold_percentage * H) + (y_threshold_tolerance * H)
                    lower_x = (x_threshold_percentage * W) - (x_threshold_tolerance * W)
                    upper_x = (x_threshold_percentage * W) + (x_threshold_tolerance * W)

                    if lost_object:
                        y_centered = True
                    else:
                        y_centered = lower_y < self.cy < upper_y

                    # Drop condition, TODO: we only have y centered rn
                    print("Checking for drop condition")
                    print(
                        self.current_pose.position.z <= DROP_THRESHOLD + 0.003,
                        y_centered,
                    )
                    if (
                        self.current_pose.position.z <= DROP_THRESHOLD + 0.003
                        and y_centered
                    ):
                        # TODO: Add bounding box check cause cant move straight down in every config

                        print("DROPPING")
                        print("Height: ", self.current_pose.position.z)
                        break

                    # Tracking Movements
                    move_success = True

                    if lost_object:
                        print("lost object")
                        continue
                        
                    amt_x = abs(self.cx - y_threshold_percentage*W)/(W/2)
                    amt_y = abs(self.cy - x_threshold_percentage*H)/(H/2)
                    k_P = 4
                    amt_x *= k_P
                    amt_y *= k_P

                    if self.current_pose.position.z < DROP_THRESHOLD:
                        move_success = self.move_up_task_space()
                        rospy.sleep(1.0)
                    elif self.cx > upper_x:
                        self.move_right(amt_x)
                        print("object visible and moving right")
                        prev_move = None

                    elif self.cx < lower_x:
                        self.move_left(amt_x)
                        print("object visible and moving left")
                        prev_move = None

                    elif self.cy < lower_y:
                        y_move_success = self.move_forward_task_space()
                        print("object visible and moving fwd")
                        prev_move = None
                    elif self.cy > upper_y:
                        y_move_success = self.move_backward_task_space()
                        print("object visible and moving bwd")
                        prev_move = None
                    elif self.current_pose.position.z > DROP_THRESHOLD:
                        move_success = self.move_down_task_space()
                        print("object visible and moving down")
                        prev_move = "down"
                    else:
                        print("None of the conditions are fulfilled for movement")

                    # If the center of the can is out of range, check if edge of can is still grabbable. Assume can is ideally angled
                    if not y_move_success and self.cy is not None:
                        if (
                            self.cy - self.can_height / 2 < upper_y
                            or self.cy + self.can_height / 2 > lower_y
                        ):
                            print(
                                "Can can still be grabbed even if not y-centered. Moving down"
                            )

                            self.move_down_task_space()
                            prev_move = "down"

                self._pick_up_and_drop_off()

        except KeyboardInterrupt:
            rospy.loginfo("Shutting down cleanly (Ctrl+C)")
        finally:
            rospy.signal_shutdown("User interrupted")

if __name__ == "__main__":
    # rospy.init_node('pi4_state_machine')
    # controller = Controller()
    # controller.move_home()
    # while True:
    #     controller.open_gripper()
        # rospy.sleep(1.0)
        # controller.close_gripper()

    # rospy.sleep(1.0)
    # print("Can Depth: ", controller.can_dist)
    # controller.mode = "search"
    # # controller.start_scan()
    # controller.mode = "pickup"
    # controller.start_pickup()
    print("please work")
