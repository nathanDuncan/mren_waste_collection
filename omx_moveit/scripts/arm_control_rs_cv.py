#!/usr/bin/env python3

import rospy
import json
from open_manipulator_msgs.msg import KinematicsPose, JointPosition
from open_manipulator_msgs.srv import SetKinematicsPose, SetJointPosition
from std_msgs.msg import Int32MultiArray, String
from sensor_msgs.msg import JointState
import copy
import math
import numpy as np

TIME = 1.0
JOINT_SPACE_TIME = 2.5
# DROP_THRESHOLD = -0.010
DROP_THRESHOLD = 0.0
H = 480
W = 640


class Controller:
    """
    Access point to control the OpenManipulator X arm using joint/task space commands.
    """

    def __init__(self):
        rospy.init_node("manipulator_controller")

        # Initialize class variables
        self.current_pose = None
        self.current_joint_state = None
        self.cx = None
        self.cy = None
        self.can_height = None
        self.can_width = None
        self.can_angle = None

        # Subscriptions
        rospy.Subscriber(
            "/gripper/kinematics_pose", KinematicsPose, self._pose_callback
        )
        rospy.Subscriber("/joint_states", JointState, self._joint_state_callback)
        rospy.Subscriber(
            "/camera_data", String, callback=self._camera_subscriber_callback
        )

        # Wait for services
        rospy.wait_for_service("/goal_joint_space_path")
        rospy.wait_for_service("/goal_joint_space_path_from_present")
        rospy.wait_for_service("/goal_task_space_path_from_present_position_only")
        rospy.wait_for_service("/goal_tool_control")

        # Service proxies
        self._abs_joint_srv = rospy.ServiceProxy(
            "/goal_joint_space_path", SetJointPosition
        )
        self._delta_joint_srv = rospy.ServiceProxy(
            "/goal_joint_space_path_from_present", SetJointPosition
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
        try:
            # Decode the JSON string back into a Python list
            detected_objects = json.loads(msg.data)

            # Make sure we actually detected something before grabbing coordinates
            if len(detected_objects) > 0:
                # Grab the first object in the list
                self.cx = detected_objects[0]["x_pos"]
                self.cy = detected_objects[0]["y_pos"]
                # print(f"Camera data updated: cx={self.cx}, cy={self.cy}")
                self.can_height = detected_objects[0]["height"]
                self.can_width = detected_objects[0]["width"]
                self.can_angle = detected_objects[0]["angle"]

                # TODO: Calculate can bottom and top points if the can is not aligned
                can_bottom_y = self.cy - (self.can_height / 2) * math.sin(
                    math.radians(self.can_angle)
                )
                can_bottom_x = self.cx + (self.can_width / 2) * math.cos(
                    math.radians(self.can_angle)
                )
            else:
                # Optional: handle the case where no objects are in view
                self.cx = None
                # self.cy = None

        except json.JSONDecodeError as e:
            rospy.logwarn(f"Failed to parse JSON from camera: {e}")

    # Fundamental movement functions
    def move_joint_space_absolute(self, joint_angles, path_time: float):
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

    def move_joint_space_delta(self, joint_deltas, path_time: float):
        """Shift joints by a relative amount (Emulates teleop 'y'/'h' keys)"""
        msg = JointPosition()
        msg.joint_name = ["joint1", "joint2", "joint3", "joint4"]
        msg.position = joint_deltas

        try:
            resp = self._delta_joint_srv(
                planning_group="arm", joint_position=msg, path_time=path_time
            )
            rospy.sleep(path_time)
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

    def move_task_space_absolute(self, pose, path_time: float):
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

    # Abstracted Directional Movements
    def move_up(self):
        return self.move_task_space_delta(dx=0.0, dy=0.0, dz=0.003, dt=TIME / 2)

    def move_down(self):
        return self.move_task_space_delta(dx=0.0, dy=0.0, dz=-0.005, dt=TIME)

    def move_forward(self):
        return self.move_task_space_delta(dx=0.01, dy=0.0, dz=0.0, dt=TIME)

    def move_backward(self):
        return self.move_task_space_delta(dx=-0.01, dy=0.0, dz=0.0, dt=TIME)

    def move_right(self):
        print("Moving Right (Joint Space)")
        return self.move_joint_space_delta(
            joint_deltas=[0.05, 0.0, 0.0, 0.0], path_time=TIME
        )

    def move_left(self):
        print("Moving Left (Joint Space)")
        return self.move_joint_space_delta(
            joint_deltas=[-0.05, 0.0, 0.0, 0.0], path_time=TIME
        )

    # Gripper Functions
    def close_gripper_flipped(self):
        msg = JointPosition()
        msg.joint_name = ["gripper"]
        msg.position = [0.0080]
        try:
            self._gripper_srv(
                planning_group="gripper", joint_position=msg, path_time=0.5
            )
            rospy.sleep(1.0)
        except:
            pass

    def open_gripper_flipped(self):
        msg = JointPosition()
        msg.joint_name = ["gripper"]
        msg.position = [-0.0220]
        try:
            self._gripper_srv(
                planning_group="gripper", joint_position=msg, path_time=0.5
            )
            rospy.sleep(1.0)
        except:
            pass

    # Presets
    def move_home(self):
        return self.move_joint_space_absolute([0.0, -1.0, 0.3, 0.7], JOINT_SPACE_TIME)

    def move_stable(self):
        return self.move_joint_space_absolute(
            [0.0, -1.61, 1.320, 0.371], JOINT_SPACE_TIME
        )

    def move_to_dropoff_2(self):
        return self.move_joint_space_absolute(
            [0.0, -1.159, -0.711, -1.320], JOINT_SPACE_TIME
        )

    def pick_up_and_drop_off(self):
        self.close_gripper_flipped()
        self.open_gripper_flipped()
        print("moving down")

        height = 0.12 + controller.current_pose.position.z
        print(height)

        # TODO: Sometimes this IK solve fails and gripper just skips this, but should fix
        move_success = False
        target_pose = self.current_pose
        target_pose.position.z -= height
        print("Trying to reach pose: ", target_pose)
        while move_success == False:
            # Mark the location we need the arm to go to
            move_success = self.move_task_space_delta(
                dx=0.0, dy=0.0, dz=-height, dt=5 * TIME
            )

            if not move_success:
                print("Initial task space move failed, trying from home")
                # Or reposition to home, then move directly? Hopes that task space will be easier to solve from home
                self.move_home()
                self.move_task_space_absolute(target_pose)
                # Wiggle function
                # self.move_joint_space_delta([0, 0, 0, 0.05])
                pass
        print("moved down")
        self.close_gripper_flipped()
        self.move_home()
        self.move_to_dropoff_2()
        self.open_gripper_flipped()
        self.move_stable()
        return


if __name__ == "__main__":
    controller = Controller()
    rospy.sleep(1.0)

    y_move_success = True
    joint_angles = [0.0037, 0.1537, 0.1012, 1.2547]
    joint_angles = [0.0061, 0.276, -0.120, 1.328]
    safe_to_pickup = True

    try:
        while not rospy.is_shutdown() and safe_to_pickup:
            start_flag = input("Type 's' to start (Ctrl+C to quit): ")
            if start_flag != "s":
                continue

            # Setup
            controller.move_joint_space_absolute(joint_angles, JOINT_SPACE_TIME)
            rospy.sleep(0.2)
            controller.open_gripper_flipped()

            prev_move = None
            lost_object = False
            # Enter active tracking loop
            while not rospy.is_shutdown():
                if controller.cx is None and prev_move != "down":
                    print("no visible object and prev move not down")
                    continue
                elif controller.cx is None and prev_move == "down":
                    print("lost object, Moving down")
                    lost_object = True
                    if controller.current_pose.position.z > DROP_THRESHOLD:
                        controller.move_down()
                    y_centered = True
                    prev_move = "down"

                y_threshold_percentage = 0.40
                y_threshold_tolerance = 0.10
                x_threshold_percentage = 0.50
                x_threshold_tolerance = 0.10

                lower_y = (y_threshold_percentage * H) - (y_threshold_tolerance * H)
                upper_y = (y_threshold_percentage * H) + (y_threshold_tolerance * H)
                lower_x = (x_threshold_percentage * W) - (x_threshold_tolerance * W)
                upper_x = (x_threshold_percentage * W) + (x_threshold_tolerance * W)

                if lost_object:
                    y_centered = True
                else:
                    y_centered = lower_y < controller.cy < upper_y

                # Drop condition, TODO: we only have y centered rn
                print("Checking for drop condition")
                print(
                    controller.current_pose.position.z <= DROP_THRESHOLD + 0.003,
                    y_centered,
                )
                if (
                    controller.current_pose.position.z <= DROP_THRESHOLD + 0.003
                    and y_centered
                ):
                    # TODO: Add bounding box check cause cant move straight down in every config

                    print("DROPPING")
                    print("Height: ", controller.current_pose.position.z)
                    break

                # Tracking Movements
                move_success = True

                if lost_object:
                    continue

                if controller.current_pose.position.z < DROP_THRESHOLD:
                    move_success = controller.move_up()
                    rospy.sleep(1.0)
                elif controller.cx < lower_x:
                    controller.move_right()
                    print("object visible and moving right")
                    prev_move = None
                elif controller.cx > upper_x:
                    controller.move_left()
                    print("object visible and moving left")
                    prev_move = None
                elif controller.cy < lower_y:
                    y_move_success = controller.move_forward()
                    print("object visible and moving fwd")
                    prev_move = None
                elif controller.cy > upper_y:
                    y_move_success = controller.move_backward()
                    print("object visible and moving bwd")
                    prev_move = None
                elif controller.current_pose.position.z > DROP_THRESHOLD:
                    move_success = controller.move_down()
                    print("object visible and moving down")
                    prev_move = "down"

                # If the center of the can is out of range, check if edge of can is still grabbable. Assume can is ideally angled
                if not y_move_success:
                    if (
                        controller.cy - controller.can_height / 2 < upper_y
                        or controller.cy + controller.can_height / 2 > lower_y
                    ):
                        print(
                            "Can can still be grabbed even if not y-centered. Moving down"
                        )
                        y_move_success = controller.move_down()
                        prev_move = "down"

                # Recovery
                # if not move_success:
                #     rospy.sleep(0.5)
                #     while controller.current_pose.position.z < DROP_THRESHOLD:
                #         controller.move_up()

            # TODO: Add bounding box check cause cant move straight down in every config

            # if controller.cx is not None:
            controller.pick_up_and_drop_off()

    except KeyboardInterrupt:
        rospy.loginfo("Shutting down cleanly (Ctrl+C)")
    finally:
        rospy.signal_shutdown("User interrupted")