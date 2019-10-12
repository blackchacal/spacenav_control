#ifndef SPACENAV_CONTROL_CONTROLLER_H
#define SPACENAV_CONTROL_CONTROLLER_H

#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <string.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <urdf/model.h>
#include <boost/algorithm/string.hpp>  // Must be installed on linux distro

#define LOG_TAG "SPACENAV_CONTROL"

namespace spacenav
{
enum class Modes
{
  PosRel = 0,
  PosAbs = 1,
  Nav3D = 2
};

enum class Sensitivity
{
  Low = 0,
  Medium = 1,
  High = 2
};

enum class JoyStates
{
  X_NEGATIVE = 0,
  X_ZERO = 1,
  X_POSITIVE = 2,
  Y_NEGATIVE = 3,
  Y_ZERO = 4,
  Y_POSITIVE = 5,
  Z_NEGATIVE = 6,
  Z_ZERO = 7,
  Z_POSITIVE = 8
};

class Controller
{
private:
  // Constants
  const float LOW_SENSITIVITY = 0.01;
  const float MEDIUM_SENSITIVITY = 0.1;
  const float HIGH_SENSITIVITY = 1;
  const int JOINT_ABSOLUTE_ANGLE_INC = 15;

  // Vars
  Modes mode;
  std::string robot_name;
  std::map<std::string, urdf::JointSharedPtr> robot_joints;
  std::vector<std::string> joint_names;
  ros::Publisher joint_position_pub;
  int total_joints;
  int selected_joint;
  float joint_angle_rad;
  float joint_angle_deg;
  float sensitivity_factor;

  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> accelerations;
  std::vector<double> effort;
  trajectory_msgs::JointTrajectory joint_msg;
  trajectory_msgs::JointTrajectoryPoint point;

  bool mode_button_state = false, prev_mode_button_state = false;
  bool joint_button_state = false, prev_joint_button_state = false;
  JoyStates joy_x_state = JoyStates::X_ZERO, prev_joy_x_state = JoyStates::X_ZERO;
  JoyStates joy_y_state = JoyStates::Y_ZERO, prev_joy_y_state = JoyStates::Y_ZERO;
  JoyStates joy_z_state = JoyStates::Z_ZERO, prev_joy_z_state = JoyStates::Z_ZERO;
  bool only_nav_mode = false;

  // Methods
  void changeMode(int msg_mode_button_state);
  void changeJoint(int msg_joint_button_state);
  void changeJointAbsoluteValue(float msg_yaxis, float msg_zaxis);
  void changeJointRelativeValue(float msg_yaxis);
  void publishNewJointState(int joint, float value, float duration = 5.0);

public:
  Controller(ros::NodeHandle nh, std::string robot_name, std::map<std::string, urdf::JointSharedPtr> robot_joints,
             std::string controller_topic, Sensitivity sensitivity);
  void getSpacenavDataCallback(const sensor_msgs::Joy::ConstPtr &msg);
};
}  // namespace spacenav

#endif