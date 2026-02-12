#!/usr/bin/env python3

import rospy
from open_manipulator_msgs.msg import KinematicsPose, JointPosition
from open_manipulator_msgs.srv import SetKinematicsPose, SetJointPosition
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import JointState
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
            "camera_data", Int32MultiArray, callback=self._camera_subscriber_callback
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

    def _camera_subscriber_callback(self, msg: list[int]):
        """
        Docstring for _camera_subscriber_callback

        :param msg: Object containing data from the camera subscriber, ordered by [centroid_x_position, centroid_y_position].
        :type msg: list[int]
        """
        self.cx = msg.data[0]
        self.cy = msg.data[1]

    # Fundamental movement functions
    def move_joint_space(self, joint_angles: list[float], path_time: float):
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

    def move_stable(self):
        # Prior: -1.697
        # Long range angles
        stable_joint_angles = [0.0, -1.61, 1.320, 0.371]

        # Short range: position: [-0.0015339808305725455, -0.9265244007110596, 0.5353593230247498, 1.1443496942520142, -0.009986215531826019]
        return self.move_joint_space(stable_joint_angles, JOINT_SPACE_TIME)

    def move_stable_short(self):
        # Prior: -1.697
        # Temp short stable test
        stable_joint_angles = [
            -0.0015339808305725455,
            -0.9265244007110596,
            0.5353593230247498,
            1.1443496942520142,
        ]

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
        self.move_stable()
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

    go_is_crouching = True
    if go_is_crouching:
        safe_to_pickup = True
    else:
        safe_to_pickup = False
    try:
        while not rospy.is_shutdown() and safe_to_pickup:
            # controller.move_stable()
            # controller.move_stable_short()
            start_flag = input("Type 's' to start (Ctrl+C to quit): ")

            if start_flag != "s":
                continue
            controller.move_joint_space(joint_angles, JOINT_SPACE_TIME)
            rospy.sleep(0.2)
            controller.open_gripper_flipped()

            # TODO: Change this to depth maybe
            while not rospy.is_shutdown() and controller.cx is not None:

                # TODO: Add left and right movement using base motor only
                # TODO: If the centroid disappears, it currently gets stuck in the prev movement. Make it do nothing or go down

                y_threshold_percentage = 0.60
                y_threshold_tolerance = 0.10
                x_threshold_percentage = 0.50
                x_threshold_tolerance = 0.25
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

                # Check if object is centered
                if (
                    controller.cy > lower_y_threshold
                    and controller.cy < upper_y_threshold
                ):
                    y_centered = True
                else:
                    y_centered = False

                # Exit the loop if z position limit is reached AND object is centered
                # TODO: Change -0.1 to based on floor distance away based on raised arm height, rn floor is -0.18 m and EE frame to end of prong ~0.03~0.04 m
                if controller.current_pose.position.z <= -0.025 and y_centered:
                    print("Height: ", controller.current_pose.position.z)
                    break
                # This allows the controller to not get stuck performing the same action if no new centroid positions are being received.
                # TODO: Make some type of 'search' in the local area if not detecting
                # if (
                #     abs(controller.cx - prev_controller_cx) < 3
                #     and abs(controller.cy - prev_controller_cy) < 3
                # ):
                #     pass
                if controller.cy < lower_y_threshold:
                    controller.move_forward()
                elif controller.cy > upper_y_threshold:
                    controller.move_backward()
                # elif controller.cx < lower_x_threshold:
                #     controller.move_right()
                # elif controller.cx > upper_x_threshold:
                #     controller.move_left()

                else:
                    if controller.current_pose.position.z > -0.025:
                        controller.move_down()

                # Update prev states
                if controller.cx is not None and controller.cy is not None:
                    prev_controller_cx = controller.cx
                    prev_controller_cy = controller.cy

                # Dead Reckoning
                # Given last known centroid position and end-effector position, move towards object
                # Perfect Case: Center the object at a certain height, then just go straight down, no calc needed
                # Just add this to pick up and drop off operation tbh

            controller.pick_up_and_drop_off()
            rospy.sleep(5.0)

    except KeyboardInterrupt:
        rospy.loginfo("Shutting down cleanly (Ctrl+C)")

    finally:
        # Put any cleanup here
        # controller.stop()  # if you have one
        rospy.signal_shutdown("User interrupted")
