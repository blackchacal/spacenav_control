#ifndef SPACENAV_CONTROL_CONTROLLER_H
#define SPACENAV_CONTROL_CONTROLLER_H

#include <fstream>
#include <string>
#include <list>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <cmath>
#include <Eigen/Dense>

#include <urdf/model.h>
#include <boost/algorithm/string.hpp>  // Must be installed on linux distro

#define LOG_TAG "SPACENAV_CONTROL"

namespace spacenav
{
enum class Modes
{
  JointPosRel,
  JointPosAbs,
  TaskPosRel,
  GetPoints,
  Nav3D
};

enum class Sensitivity
{
  Low,
  Medium,
  High
};

enum class JoyStates
{
  X_NEGATIVE,
  X_ZERO,
  X_POSITIVE,
  Y_NEGATIVE,
  Y_ZERO,
  Y_POSITIVE,
  Z_NEGATIVE,
  Z_ZERO,
  Z_POSITIVE
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
  std::list<char> active_modes;
  std::string robot_name;
  std::map<std::string, urdf::JointSharedPtr> robot_joints;
  std::vector<std::string> joint_names;
  int total_joints;
  int selected_joint;
  float joint_angle_rad;
  float joint_angle_deg;
  float sensitivity_factor;
  bool robot;
  geometry_msgs::Pose current_pose;

  // Files
  std::fstream pts_fh; // File handler for space points data file

  // Publishers/Subscribers
  ros::Publisher joint_position_pub;
  ros::Publisher cartesian_pose_pub;
  ros::Subscriber robot_state_sub;
  trajectory_msgs::JointTrajectory joint_msg;
  trajectory_msgs::JointTrajectoryPoint point;
  geometry_msgs::Pose pose_msg;

  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> accelerations;
  std::vector<double> effort;

  bool mode_button_state = false, prev_mode_button_state = false;
  bool option_button_state = false, prev_option_button_state = false;
  JoyStates joy_x_state = JoyStates::X_ZERO, prev_joy_x_state = JoyStates::X_ZERO;
  JoyStates joy_y_state = JoyStates::Y_ZERO, prev_joy_y_state = JoyStates::Y_ZERO;
  JoyStates joy_z_state = JoyStates::Z_ZERO, prev_joy_z_state = JoyStates::Z_ZERO;
  bool only_nav_mode = false;

  // Methods
  void changeMode(int msg_mode_button_state);
  void changeJoint(int msg_option_button_state);
  void changeJointAbsoluteValue(float msg_yaxis, float msg_zaxis);
  void changeJointRelativeValue(float msg_yaxis);
  void changePoseRelativeValue(const sensor_msgs::Joy::ConstPtr &msg);
  void publishNewJointState(int joint, float value, float duration = 5.0);
  void publishNewCartesianPose(const geometry_msgs::Pose pose);
  void setupPublishersAndSubscribers(ros::NodeHandle nh, std::string controller_topic, std::string controller_topic_type, 
                                    std::string robot_state_topic, std::string robot_state_topic_type);
  void savePoints(const sensor_msgs::Joy::ConstPtr &msg);
  bool checkIfModeIsActive(char mode);
  void setJointPosRelMode(void);
  void setJointPosAbsMode(void);
  void setTaskPosRelMode(void);
  void setGetPointsMode(void);
  void setNav3DMode(void);

  // Callback methods
  void getRobotStatePoseCallback(const geometry_msgs::PoseConstPtr &msg);
  void getRobotStatePoseStampedCallback(const geometry_msgs::PoseStampedConstPtr &msg);
  void getRobotStatePoseCovCallback(const geometry_msgs::PoseWithCovarianceConstPtr &msg);
  void getRobotStatePoseCovStampedCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);
public:
  Controller(ros::NodeHandle nh, std::string robot_name, 
            std::map<std::string, urdf::JointSharedPtr> robot_joints,
            std::string controller_topic, std::string controller_topic_type, 
            std::string robot_state_topic, std::string robot_state_topic_type, 
            Sensitivity sensitivity, bool is_real, std::string active_modes);
  // Callback methods
  void getSpacenavDataCallback(const sensor_msgs::Joy::ConstPtr &msg);
};
}  // namespace spacenav

#endif