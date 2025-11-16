#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Vector3, Twist

class TurtleSimController:
    def __init__(self):
        rospy.loginfo("Starting TurtleSim Controller node...")
        
        # --- Controller Gains (Tunable) ---
        # Proportional gain for linear velocity (based on size error)
        # We make this small because e_s can be large (e.g., 150 pixels)
        self.Kp_linear = 0.005
        
        # Proportional gain for angular velocity (based on lateral error)
        # We need a negative sign because:
        # +e_u (ball on right) needs a -angular.z (clockwise turn)
        self.Kp_angular = -0.002
        
        # --- Velocity Limits (Safety) ---
        self.max_linear_vel = 1.0
        self.max_angular_vel = 1.5

        # ROS Publishers
        self.twist_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # ROS Subscriber
        self.error_sub = rospy.Subscriber('/control_errors', Vector3, self.error_callback)
        
        # Initialize a Twist message to zeros
        self.stop_msg = Twist()
        self.stop_msg.linear.x = 0.0
        self.stop_msg.angular.z = 0.0

    def error_callback(self, msg):
        """This function is called every time a new /control_errors message is received."""
        
        e_u = msg.x       # Lateral error
        e_s = msg.y       # Size error
        ball_seen = msg.z # Ball seen flag (1.0 or 0.0)

        cmd_vel_msg = Twist()

        if ball_seen > 0.5:
            # Ball is visible - run the controller
            
            # 1. Calculate Linear (forward/backward) velocity
            linear_vel = self.Kp_linear * e_s
            
            # 2. Calculate Angular (turning) velocity
            angular_vel = self.Kp_angular * e_u
            
            # 3. Saturate (clamp) the velocities to safe limits
            cmd_vel_msg.linear.x = max(-self.max_linear_vel, min(self.max_linear_vel, linear_vel))
            cmd_vel_msg.angular.z = max(-self.max_angular_vel, min(self.max_angular_vel, angular_vel))

            rospy.loginfo_throttle(0.5, f"Ball seen. Errors (u,s): ({e_u:.1f}, {e_s:.1f}) -> Vels (lin,ang): ({cmd_vel_msg.linear.x:.2f}, {cmd_vel_msg.angular.z:.2f})")

        else:
            # Ball is NOT visible - stop the turtle
            cmd_vel_msg = self.stop_msg
            rospy.logwarn_throttle(1.0, "Ball not seen. Stopping.")

        # Publish the final command
        self.twist_pub.publish(cmd_vel_msg)
        
    def spin(self):
        # Keep the node alive. When we shut down, publish one last stop command.
        rospy.on_shutdown(self.shutdown_hook)
        rospy.spin()
        
    def shutdown_hook(self):
        rospy.loginfo("Shutting down controller, sending one last stop command.")
        self.twist_pub.publish(self.stop_msg)

def main():
    rospy.init_node('turtlesim_controller', anonymous=True)
    controller = TurtleSimController()
    controller.spin()

if __name__ == '__main__':
    main()