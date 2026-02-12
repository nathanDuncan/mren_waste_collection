#!/usr/bin/env python3

import rospy
from open_manipulator_msgs.msg import KinematicsPose, JointPosition
from open_manipulator_msgs.srv import SetKinematicsPose, SetJointPosition
from std_msgs.msg import Int32MultiArray, Float32MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion
import copy

# JointState is needed as that is the type of message that is received from the arm's sensors
# It is read and transformed into a JointPosition object as that is needed for the mutator method.

TIME = 1.0
JOINT_SPACE_TIME = 2.5
H = 480
W = 640


class Controller:
    """
    Access point to control the OpenManipulator X arm using joint/task space commands.

    Attributes:
        current_pose (KinematicsPose): The current kinematic position of the end effector.
        current_joint_state (JointState): The current joint state of the robot.
        cx (int): The x-position of object of interest's centroid represented in pixels.
        cy (int): The y-position of object of interest's centroid represented in pixels.


    """

    def __init__(self):
        # Start ROS node
        rospy.init_node("manipulator_controller")

        # Initialize class variables
        self.current_pose = None
        self.current_joint_state = None
        self.cx = None
        self.cy = None

        # Subscriptions to ROS topics
        rospy.Subscriber(
            "/gripper/kinematics_pose", KinematicsPose, self._pose_callback
        )
        rospy.Subscriber("/joint_states", JointState, self._joint_state_callback)
        rospy.Subscriber(
            "/camera_data", Float32MultiArray, callback=self._camera_subscriber_callback
        )

        # Wait for services
        rospy.wait_for_service("/goal_task_space_path")
        rospy.wait_for_service("/goal_joint_space_path")
        rospy.wait_for_service("/goal_tool_control")

        # Service proxies
        self._ik_service = rospy.ServiceProxy(
            "/goal_task_space_path", SetKinematicsPose
        )
        self._fk_service = rospy.ServiceProxy(
            "/goal_joint_space_path", SetJointPosition
        )
        self._gripper_control_service = rospy.ServiceProxy(
            "/goal_tool_control", SetJointPosition
        )

    # Callbacks to obtain information from ROS messages
    def _pose_callback(self, msg: KinematicsPose):
        """
        Callback to read kinematics pose of end effector.

        :param msg: KinematicsPose object containing pose information.
        :type msg: KinematicsPose
        """
        self.current_pose = msg.pose

    def _joint_state_callback(self, msg: JointState):
        """
        Callback to read joint state of the manipulator.

        :param msg: Object containing joint state information.
        :type msg: JointState
        """
        # This slices the position message to only give values for joint1, joint2, joint3, joint4
        self.current_joint_state = msg.position[2:]

    def _camera_subscriber_callback(self, msg):
        """
        Docstring for _camera_subscriber_callback

        :param msg: Object containing data from the camera subscriber, ordered by [centroid_x_position, centroid_y_position].
        :type msg: list[int]
        """
        self.cx = msg.data[0]
        self.cy = msg.data[1]

    # Fundamental movement functions
    def move_joint_space(self, joint_angles, path_time: float):
        """
        Command the arm to move to specific joint angles.

        :param joint_angles: List of joint angles for each motor to go to.
        :type joint_angles: list[float]

        :param path_time: Path time of the specified movement.
        :type path_time: float
        """
        joint_msg = JointPosition()
        joint_msg.joint_name = ["joint1", "joint2", "joint3", "joint4"]
        joint_msg.position = joint_angles

        try:
            resp = self._fk_service(
                planning_group="arm", joint_position=joint_msg, path_time=path_time
            )
            rospy.loginfo(f"Joint space move planned: {resp.is_planned}")
            rospy.sleep(path_time)
            return resp.is_planned
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False

    def move_increment(self, dx=0, dy=0, dz=0, dt=1.0):
        """Move a small increment from current pose"""
        if self.current_pose is None:
            rospy.logwarn("Current pose not received yet")
            return False

        # Create new target pose by offsetting current
        target_pose = copy.deepcopy(self.current_pose)
        target_pose.position.x += dx
        target_pose.position.y += dy
        target_pose.position.z += dz
        print("Current pose:")
        print(self.current_pose)

        # Build kinematics pose object

        kinematics_msg = KinematicsPose()
        kinematics_msg.pose = target_pose
        kinematics_msg.max_accelerations_scaling_factor = 1.0
        kinematics_msg.max_velocity_scaling_factor = 1.0
        kinematics_msg.tolerance = 0.0

        try:
            resp = self._ik_service(
                planning_group="arm",
                end_effector_name="gripper",
                kinematics_pose=kinematics_msg,
                path_time=dt,
            )
            rospy.loginfo(f"Task space move planned: {resp.is_planned}")
            rospy.sleep(dt)
            return resp.is_planned
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False

    # Abstracted movement functions
    def move_up(self):
        dx, dy, dz = directions["up"]
        controller.move_increment(dx, dy, dz, dt=TIME)

    def move_down(self):
        dx, dy, dz = directions["down"]
        controller.move_increment(dx, dy, dz, dt=TIME)

    def move_right(self):

        # Move joint space but use current joint position +- only in joint1
        new_joint_angles = list(self.current_joint_state)
        print("New joint angles: ", new_joint_angles)
        print("joint1: ", new_joint_angles[0])
        print("Moving Right")
        # Move base by X radians (0.09 is roughly 5 deg)
        new_joint_angles[0] = new_joint_angles[0] + 0.09
        self.move_joint_space(new_joint_angles, TIME)

    def move_left(self):
        # Move joint space but use current joint position +- only in joint1
        new_joint_angles = list(self.current_joint_state)
        print("New joint angles: ", new_joint_angles)
        print("joint1: ", new_joint_angles[0])
        print("Moving Left")
        # Move base by X radians (0.09 is roughly 5 deg)
        new_joint_angles[0] = new_joint_angles[0] - 0.09
        self.move_joint_space(new_joint_angles, TIME)

    def move_forward(self):
        dx, dy, dz = directions["forward"]
        controller.move_increment(dx, dy, dz, dt=TIME)

    def move_backward(self):
        dx, dy, dz = directions["backward"]
        controller.move_increment(dx, dy, dz, dt=TIME)

    # Gripper control functions
    def open_gripper(self):
        joint_msg = JointPosition()
        joint_msg.joint_name = ["gripper"]
        joint_msg.position = [0.0075]

        try:
            resp = self._gripper_control_service(
                planning_group="gripper", joint_position=joint_msg, path_time=0.5
            )
            rospy.loginfo(f"Joint space move planned: {resp.is_planned}")
            rospy.sleep(1.0)
            return resp.is_planned
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False

    def close_gripper(self):
        joint_msg = JointPosition()
        joint_msg.joint_name = ["gripper"]
        joint_msg.position = [-0.0075]

        try:
            resp = self._gripper_control_service(
                planning_group="gripper", joint_position=joint_msg, path_time=0.5
            )
            rospy.loginfo(f"Joint space move planned: {resp.is_planned}")
            rospy.sleep(1.0)
            return resp.is_planned
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False

    # TODO: Right now it seems open and close grip are flipped on the robot so this is temp
    # This closes the gripper
    def close_gripper_flipped(self):
        joint_msg = JointPosition()
        joint_msg.joint_name = ["gripper"]
        joint_msg.position = [0.0080]

        try:
            resp = self._gripper_control_service(
                planning_group="gripper", joint_position=joint_msg, path_time=0.5
            )
            rospy.loginfo(f"Joint space move planned: {resp.is_planned}")
            rospy.sleep(1.0)
            return resp.is_planned
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False

    # This opens the gripper
    # TODO: If we want more clearance then edit the joint limits in Dynamixel Wizard 2.0
    def open_gripper_flipped(self):
        joint_msg = JointPosition()
        joint_msg.joint_name = ["gripper"]
        joint_msg.position = [-0.0100]

        try:
            resp = self._gripper_control_service(
                planning_group="gripper", joint_position=joint_msg, path_time=0.5
            )
            rospy.loginfo(f"Joint space move planned: {resp.is_planned}")
            rospy.sleep(1.0)
            return resp.is_planned
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False

    # Preset joint angle movement functions
    def move_home(self):
        home_joint_angles = [0.0, -1.0, 0.3, 0.7]
        return self.move_joint_space(home_joint_angles, JOINT_SPACE_TIME)

    def move_far_view(self):
        # Long range angles
        stable_joint_angles = [0.0, -1.61, 1.320, 0.371]

        return self.move_joint_space(stable_joint_angles, JOINT_SPACE_TIME)

    def move_short_view(self):

        # position: [-0.006135923322290182, -0.777728259563446, -0.34514567255973816, 1.9527575969696045]
        # Nathan short stable
        stable_joint_angles = [
            -0.006135923322290182,
            -0.777728259563446,
            -0.34514567255973816,
            1.9527575969696045,
        ]
        return self.move_joint_space(stable_joint_angles, JOINT_SPACE_TIME)

    def move_to_dropoff(self):
        dropoff_joint_angles = [-2.8, -0.179, -0.140, 0.320]
        return self.move_joint_space(dropoff_joint_angles, JOINT_SPACE_TIME)

    def move_to_dropoff_2(self):
        dropoff_joint_angles = [0.0, -1.159, -0.711, -1.320]
        return self.move_joint_space(dropoff_joint_angles, JOINT_SPACE_TIME)

    def pick_up_and_drop_off(self):

        # Move straight down
        self.close_gripper_flipped()
        self.open_gripper_flipped()
        print("moving down")
        dx, dy, dz = (0, 0, -0.07)
        controller.move_increment(dx, dy, dz, dt=5 * TIME)
        print("moved down")
        self.close_gripper_flipped()
        self.move_home()
        self.move_to_dropoff_2()
        self.open_gripper_flipped()
        self.move_far_view()
        return



if __name__ == "__main__":
    controller = Controller()
    rospy.sleep(1.0)  # allow subscriber to get initial pose
    # TODO: P control based on centroid dist
    directions = {
        "up": (0, 0, 0.01),
        "down": (0, 0, -0.01),
        "forward": (0.01, 0, 0),
        "backward": (-0.01, 0, 0),
    }
    # Trial start positions
    joint_angles = [0.0, 0.45, -0.12, 1.18]
    prev_controller_cx = 0.0
    prev_controller_cy = 0.0

    

    try:
        while not rospy.is_shutdown():
            # controller.move_stable()
            # controller.move_stable_short()
            start_flag = input("Type 's' to start (Ctrl+C to quit): ")

            if start_flag != "s":
                continue
            controller.move_home()
            controller.move_far_view()

            # If centroid is at bottom of screen, move stable short
            while True:
                
                y_threshold_percentage = 0.075
                y_threshold_percentage = 0.50
                y_threshold_tolerance = 0.20

                x_threshold_percentage = 0.50
                x_threshold_tolerance = 0.20
                lower_y_threshold = (
                    y_threshold_percentage * H - y_threshold_tolerance * H
                )
                upper_y_threshold = (
                    y_threshold_percentage * H + y_threshold_tolerance * H
                )
                lower_x_threshold = (
                    x_threshold_percentage * W - x_threshold_tolerance * W
                )
                upper_x_threshold = (
                    x_threshold_percentage * W + x_threshold_tolerance * W
                )

                # Mock camera data correct
                # switch_flag = input("Type 's' to switch to stable short")
                # if switch_flag == "s":
                #     controller.cx = x_threshold_percentage * W
                #     controller.cy = y_threshold_percentage * H
                # print("CX and CY: ")
                # print(controller.cx, controller.cy)

                # Check if object is at the bottom of the screen
                if (
                    controller.cy > lower_y_threshold
                    and controller.cy < upper_y_threshold
                ):
                    y_centered = True
                else:
                    y_centered = False
                
                if (
                    controller.cx > lower_x_threshold
                    and controller.cx < upper_x_threshold
                ):
                    x_centered = True
                else:
                    x_centered = False
                
                if(y_centered and x_centered):
                    controller.move_short_view()
                    break


    except KeyboardInterrupt:
        rospy.loginfo("Shutting down cleanly (Ctrl+C)")

    finally:
        # Put any cleanup here
        # controller.stop()  # if you have one
        rospy.signal_shutdown("User interrupted")

