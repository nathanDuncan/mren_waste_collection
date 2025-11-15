#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys, select, tty, termios

# Define key mappings
key_bindings = {
    'w': ( 0.5,  0.0,  0.0),  # Forward
    's': (-0.5,  0.0,  0.0),  # Backward
    'a': ( 0.0,  0.0,  0.8),  # Turn Left
    'd': ( 0.0,  0.0, -0.8),  # Turn Right
    'q': ( 0.0,  0.5,  0.0),  # Strafe Left
    'e': ( 0.0, -0.5,  0.0),  # Strafe Right
    'x': ( 0.0,  0.0,  0.0),  # Stop
}

# Speed settings
speed = 1.0 # Linear speed multiplier
turn = 1.0  # Angular speed multiplier

# Help message
msg = """
Control Unitree Go1!
---------------------------
Moving around:
       w
  q    s    e
       d

w/s : move forward/backward
a/d : turn left/right
q/e : strafe left/right

x : stop all movement

CTRL-C to quit
"""

def get_key(settings):
    """Get a single keypress from the user."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1) # 0.1s timeout
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = '' # No key pressed
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    """Main function to run the teleop node."""
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('go1_teleop_key')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

    twist = Twist()
    
    # Initialize velocities to zero
    target_linear_vel = 0.0
    target_angular_vel = 0.0
    target_strafe_vel = 0.0
    
    print(msg)

    try:
        while not rospy.is_shutdown():
            key = get_key(settings)
            
            if key in key_bindings:
                target_linear_vel = key_bindings[key][0]
                target_strafe_vel = key_bindings[key][1]
                target_angular_vel = key_bindings[key][2]
                
                # Use ROS log info for debugging
                rospy.loginfo(f"Key: '{key}', Vels: Lin={target_linear_vel:.2f}, Str={target_strafe_vel:.2f}, Ang={target_angular_vel:.2f}")

            elif key == '':
                # *********** THE SAFETY FIX ***********
                # If no key is pressed, stop the robot.
                target_linear_vel = 0.0
                target_angular_vel = 0.0
                target_strafe_vel = 0.0
            
            elif key == '\x03': # CTRL-C
                break

            else:
                # If an unmapped key is pressed, stop
                target_linear_vel = 0.0
                target_strafe_vel = 0.0
                target_angular_vel = 0.0
                rospy.logwarn(f"Unmapped key pressed: {key}")


            # Populate the Twist message
            # Note: walk_ros maps linear.x to forward, linear.y to side, angular.z to rotation
            twist.linear.x = target_linear_vel * speed
            twist.linear.y = target_strafe_vel * speed
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = target_angular_vel * turn
            
            pub.publish(twist)

    except Exception as e:
        rospy.logerr(f"An error occurred: {e}")

    finally:
        # Stop the robot when quitting
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)
        
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == "__main__":
    main()