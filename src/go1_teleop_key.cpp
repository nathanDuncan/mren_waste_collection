/**
 * @file go1_teleop_key.cpp
 * @brief Keyboard teleoperation node for Unitree Go1.
 */
#include <ros/ros.h>
#include <unitree_legged_msgs/HighCmd.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h" // <-- 1. ADDED SDK INCLUDE
#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h> // For non-blocking read

// 2. ADDED SDK NAMESPACE
using namespace UNITREE_LEGGED_SDK;

// Define keycodes (lowercase)
#define KEYCODE_W 0x77
#define KEYCODE_S 0x73
#define KEYCODE_A 0x61
#define KEYCODE_D 0x64
#define KEYCODE_Q 0x71
#define KEYCODE_E 0x65
#define KEYCODE_X 0x78 // For quit

// Define keycodes (uppercase)
#define KEYCODE_W_UPPER 0x57
#define KEYCODE_S_UPPER 0x53
#define KEYCODE_A_UPPER 0x41
#define KEYCODE_D_UPPER 0x44
#define KEYCODE_Q_UPPER 0x51
#define KEYCODE_E_UPPER 0x45
#define KEYCODE_X_UPPER 0x58


/**
 * @brief A non-blocking keyboard reader class.
 * Based on the teleop_turtle_key.cpp KeyboardReader, but modified
 * to be non-blocking using select().
 */
class KeyboardReader
{
public:
  KeyboardReader() : kfd(0)
  {
    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    struct termios raw;
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
  }

  /**
   * @brief Reads a key, non-blocking.
   * @param c The character read.
   * @return 1 if a key was pressed, 0 if not.
   */
  int readOne(char * c)
  {
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(kfd, &fds);
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000; // 100ms = 10Hz timeout

    int rc = select(kfd + 1, &fds, NULL, NULL, &tv);

    if (rc < 0)
    {
      throw std::runtime_error("select failed");
    }
    if (rc == 0)
    {
      return 0; // Timeout, no key pressed
    }

    if (read(kfd, c, 1) < 0)
    {
      throw std::runtime_error("read failed");
    }
    return 1; // Key was pressed
  }

  void shutdown()
  {
    tcsetattr(kfd, TCSANOW, &cooked);
  }

private:
  int kfd;
  struct termios cooked;
};

KeyboardReader input;

/**
 * @class TeleopGo1
 * @brief Translates keyboard inputs into HighCmd messages.
 */
class TeleopGo1
{
public:
  TeleopGo1();
  void keyLoop();
  void shutdown();

private:
  ros::NodeHandle nh_;
  ros::Publisher high_cmd_pub_;
  
  double linear_vel_fwd_;
  double linear_vel_side_;
  double angular_vel_yaw_;
  
  double l_scale_; // Forward/backward scale
  double s_scale_; // Side/strafe scale
  double a_scale_; // Angular/yaw scale
};

TeleopGo1::TeleopGo1() :
  linear_vel_fwd_(0),
  linear_vel_side_(0),
  angular_vel_yaw_(0),
  l_scale_(0.5),  // Default max forward speed (m/s)
  s_scale_(0.3),  // Default max strafe speed (m/s)
  a_scale_(0.8)   // Default max yaw speed (rad/s)
{
  nh_.param("scale_linear", l_scale_, l_scale_);
  nh_.param("scale_strafe", s_scale_, s_scale_);
  nh_.param("scale_angular", a_scale_, a_scale_);

  high_cmd_pub_ = nh_.advertise<unitree_legged_msgs::HighCmd>("high_cmd", 10);
}

void quit(int sig)
{
  (void)sig;
  input.shutdown();
  ros::shutdown();
  exit(0);
}

void TeleopGo1::shutdown()
{
  input.shutdown();
}

/**
* @brief Reads keyboard state and publishes velocity commands.
* * Handles the logic for switching between 'Stand' and 'Walk' modes
* based on input presence.
*/
void TeleopGo1::keyLoop()
{
  char c;
  bool dirty = false;

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use 'w,a,s,d' to move (forward, turn, back, turn).");
  puts("Use 'q,e' to strafe (left, right).");
  puts("Press 'x' to quit.");

  // Note: ros::Rate is removed. We use the select() timeout as our loop rate.

  while (ros::ok())
  {
    // Reset velocities to zero every loop (dead-man's switch)
    linear_vel_fwd_ = 0.0;
    linear_vel_side_ = 0.0;
    angular_vel_yaw_ = 0.0;
    
    // Default to "Stand" mode
    int target_mode = 1;
    int target_gait = 0;

    // Check for key press
    if (input.readOne(&c))
    {
      // --- DEBUGGING LINE ---
      ROS_INFO("Key pressed: 0x%02X", c); 

      switch(c)
      {
        case KEYCODE_W:
        case KEYCODE_W_UPPER:
          linear_vel_fwd_ = 1.0;
          break;
        case KEYCODE_S:
        case KEYCODE_S_UPPER:
          linear_vel_fwd_ = -1.0;
          break;
        case KEYCODE_A:
        case KEYCODE_A_UPPER:
          angular_vel_yaw_ = 1.0;
          break;
        case KEYCODE_D:
        case KEYCODE_D_UPPER:
          angular_vel_yaw_ = -1.0;
          break;
        case KEYCODE_Q:
        case KEYCODE_Q_UPPER:
          linear_vel_side_ = 1.0;
          break;
        case KEYCODE_E:
        case KEYCODE_E_UPPER:
          linear_vel_side_ = -1.0;
          break;
        case KEYCODE_X:
        case KEYCODE_X_UPPER:
          ROS_INFO("Quitting...");
          return;
      }
    }

    // --- THIS IS THE FIX ---
    // Populate the HighCmd message
    unitree_legged_msgs::HighCmd high_cmd_ros;
    
    // If any velocity is non-zero, switch to "Walk" mode
    if (linear_vel_fwd_ != 0 || linear_vel_side_ != 0 || angular_vel_yaw_ != 0)
    {
      target_mode = 2;
      target_gait = 2; // Trot gait
      high_cmd_ros.footRaiseHeight = 0.1f; // Set foot raise height for walking
      high_cmd_ros.bodyHeight = 0.0f; // Keep body level
    }
    else
    {
      target_mode = 1; // Stand mode
      target_gait = 0;
      high_cmd_ros.footRaiseHeight = 0.0f;
      high_cmd_ros.bodyHeight = 0.0f;
    }
    // --- END OF FIX ---


    high_cmd_ros.head[0] = 0xFE;
    high_cmd_ros.head[1] = 0xEF;
    high_cmd_ros.levelFlag = HIGHLEVEL; // <-- 3. CORRECTED FLAG
    high_cmd_ros.mode = target_mode;
    high_cmd_ros.gaitType = target_gait;
    high_cmd_ros.speedLevel = 0;
    
    high_cmd_ros.euler[0] = 0;
    high_cmd_ros.euler[1] = 0;
    high_cmd_ros.euler[2] = 0;
    high_cmd_ros.reserve = 0;

    // Set velocities
    high_cmd_ros.velocity[0] = linear_vel_fwd_ * l_scale_; // Forward
    high_cmd_ros.velocity[1] = linear_vel_side_ * s_scale_; // Side
    high_cmd_ros.yawSpeed = angular_vel_yaw_ * a_scale_;     // Yaw

    // Publish the command
    high_cmd_pub_.publish(high_cmd_ros);

    ros::spinOnce();
    // loop_rate.sleep(); // <-- This is correctly removed
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "go1_teleop_key");
  TeleopGo1 teleop_go1;

  signal(SIGINT, quit);

  try
  {
    teleop_go1.keyLoop();
  }
  catch (const std::runtime_error &e)
  {
    ROS_FATAL("Error in keyLoop: %s", e.what());
    teleop_go1.shutdown();
  }
  
  quit(0);
  return(0);
}