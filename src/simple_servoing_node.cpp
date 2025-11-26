/**
 * @file simple_servoing_node.cpp
 * @brief Visual servoing node for the Unitree Go1 used in Fall Sketch Model Presentation.
 * * This node subscribes to control errors and publishes high-level commands
 * to align the robot with a target.
 */
#include <ros/ros.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <geometry_msgs/Quaternion.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"

using namespace UNITREE_LEGGED_SDK;

/**
 * @class SimpleServoing
 * @brief Main controller class for visual tracking.
 * * Implements a Proportional (P) controller to adjust Yaw and Pitch
 * based on visual error inputs.
 */
class SimpleServoing
{
public:
    /**
     * @brief Constructor. Initializes ROS handles and control gains.
     */
    SimpleServoing()
    {
        // --- Tunable Gains ---
        // Yaw Gain: How fast to turn per pixel of horizontal error
        kp_yaw_ = 0.003; 
        
        // Pitch Gain: How much to tilt head/body per pixel of vertical error
        kp_pitch_ = 0.002;

        // --- Safety Limits ---
        max_yaw_rate_ = 1.0; // rad/s
        max_pitch_ = 0.7;    // rad (approx 28 degrees)
        
        // --- Deadbands ---
        deadband_u_ = 40.0; // Pixels. Ignore lateral error if within this range.

        // Initialize state
        error_u_ = 0.0;
        error_v_ = 0.0;
        target_visible_ = false;

        // ROS
        high_cmd_pub_ = nh_.advertise<unitree_legged_msgs::HighCmd>("high_cmd", 1);
        error_sub_ = nh_.subscribe("/control_errors", 1, &SimpleServoing::errorCallback, this);

        // Control loop timer (50 Hz)
        timer_ = nh_.createTimer(ros::Duration(0.02), &SimpleServoing::controlLoop, this);
    }

    /**
     * @brief Callback for error updates.
     * @param msg The quaternion message containing error data (x=u, y=v).
     */
    void errorCallback(const geometry_msgs::Quaternion::ConstPtr& msg)
    {
        // Map the Quaternion fields back to our error definition
        // x = lateral error (u)
        // y = vertical error (v)
        // z = size/depth error (s) -- Unused for now
        // w = detection flag
        
        error_u_ = msg->x;
        error_v_ = msg->y;
        
        // Check if object is detected (w > 0.5)
        target_visible_ = (msg->w > 0.5);
    }

    /**
     * @brief Main control loop (50Hz).
     * * Calculates the required yaw/pitch velocity and publishes the HighCmd.
     * @param event ROS Timer event.
     */
    void controlLoop(const ros::TimerEvent&)
    {
        unitree_legged_msgs::HighCmd cmd;
        
        // Default Header & Settings
        cmd.head[0] = 0xFE;
        cmd.head[1] = 0xEF;
        cmd.levelFlag = HIGHLEVEL;
        cmd.reserve = 0;

        if (target_visible_)
        {
            // --- Visual Servoing Logic ---
            
            // 1. Yaw Control (Lateral)
            double yaw_cmd = 0.0;
            
            // Apply Deadband: Only turn if error is significant
            if (std::abs(error_u_) > deadband_u_) {
                // Error u is (center - image_center). 
                // If object is to the Right (positive u), turn Right (negative yaw).
                yaw_cmd = -kp_yaw_ * error_u_;
            }

            // 2. Pitch Control (Vertical)
            // Error v is (center - image_center).
            // If object is Low (positive v), Pitch Down (positive pitch).
            double pitch_cmd = kp_pitch_ * error_v_;

            // --- Saturation (Safety) ---
            if (yaw_cmd > max_yaw_rate_) yaw_cmd = max_yaw_rate_;
            if (yaw_cmd < -max_yaw_rate_) yaw_cmd = -max_yaw_rate_;
            
            if (pitch_cmd > max_pitch_) pitch_cmd = max_pitch_;
            if (pitch_cmd < -max_pitch_) pitch_cmd = -max_pitch_;

            // --- Populate Command ---
            cmd.mode = 2;      // Walk mode (required for yaw velocity)
            cmd.gaitType = 1;  // 0=Idle, 1=Trot, 2=Trot Running
            
            cmd.velocity[0] = 0.0; // No forward movement
            cmd.velocity[1] = 0.0; // No side movement
            cmd.yawSpeed = yaw_cmd;
            
            cmd.euler[0] = 0.0;       // Roll
            cmd.euler[1] = pitch_cmd; // Pitch
            cmd.euler[2] = 0.0;       // Yaw (Body orientation, not rate)

            // Important for walking mode
            cmd.bodyHeight = 0.0;
            cmd.footRaiseHeight = 0.08; 
        }
        else
        {
            // --- Lost Target / Idle ---
            // Stop everything and stand still
            cmd.mode = 1; // Forced Stand
            cmd.velocity[0] = 0.0;
            cmd.velocity[1] = 0.0;
            cmd.yawSpeed = 0.0;
            cmd.euler[0] = 0.0;
            cmd.euler[1] = 0.0;
            cmd.euler[2] = 0.0;
            cmd.bodyHeight = 0.0;
        }

        high_cmd_pub_.publish(cmd);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher high_cmd_pub_;
    ros::Subscriber error_sub_;
    ros::Timer timer_;

    double error_u_;
    double error_v_;
    bool target_visible_;

    double kp_yaw_;       ///< Proportional gain for Yaw control.
    double kp_pitch_;     ///< Proportional gain for Pitch control.
    double max_yaw_rate_;
    double max_pitch_;
    
    double deadband_u_; // New deadband variable
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_servoing_node");
    SimpleServoing servo;
    ros::spin();
    return 0;
}