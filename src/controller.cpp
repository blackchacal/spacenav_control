#include <spacenav_control/controller.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/AccelWithCovariance.h>
#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>

using namespace spacenav;

Controller::Controller(ros::NodeHandle nh, std::string robot_name,
                      std::map<std::string, urdf::JointSharedPtr> robot_joints, 
                      std::string controller_topic, std::string controller_topic_type,
                      std::string robot_state_topic, std::string robot_state_topic_type,
                      Sensitivity sensitivity, bool is_robot)
{
  // Define is the controller is simulation or real robot
  robot = is_robot;

  // Set initial mode to Nav3D
  mode = Modes::Nav3D;
  ROS_INFO_NAMED(LOG_TAG, "%s: Start on Nav3D Mode.", LOG_TAG);

  this->robot_name = (!robot_name.empty()) ? robot_name : "robot";
  boost::to_upper(this->robot_name);

  if (robot_joints.empty() || controller_topic.empty())
  {
    ROS_INFO_NAMED(LOG_TAG, "%s: Only Nav3D mode allowed!", LOG_TAG);
    only_nav_mode = true;
  }
  else
  {
    ROS_INFO_NAMED(LOG_TAG, "%s: Controlling '%s' robot.", LOG_TAG, this->robot_name.c_str());
    this->robot_joints = robot_joints;

    // Extract joint names (only revolute joints) from urdf model joints
    for (std::map<std::string, urdf::JointSharedPtr>::iterator it = robot_joints.begin(); it != robot_joints.end();
         ++it)
    {
      if (it->second->type == urdf::Joint::REVOLUTE)
      {
        joint_names.push_back(it->first);
      }
    }
    total_joints = joint_names.size();

    // If there is no revolute joints, set to nav only mode and exit.
    if (total_joints == 0)
    {
      ROS_INFO_NAMED(LOG_TAG, "%s: Only Nav3D mode allowed!", LOG_TAG);
      only_nav_mode = true;
      return;
    }

    // Setup the publishers and subscribers to get robot state and to set new robot state
    setupPublishersAndSubscribers(nh, controller_topic, controller_topic_type, 
                                  robot_state_topic, robot_state_topic_type);

    // Set default values
    switch (sensitivity)
    {
      case Sensitivity::Low:
        this->sensitivity_factor = LOW_SENSITIVITY;
        ROS_INFO_NAMED(LOG_TAG, "%s: Sensitivity set to LOW.", LOG_TAG);
        break;
      case Sensitivity::High:
        this->sensitivity_factor = HIGH_SENSITIVITY;
        ROS_INFO_NAMED(LOG_TAG, "%s: Sensitivity set to HIGH.", LOG_TAG);
        break;
      default:
        this->sensitivity_factor = MEDIUM_SENSITIVITY;
        ROS_INFO_NAMED(LOG_TAG, "%s: Sensitivity set to MEDIUM.", LOG_TAG);
        break;
    }

    selected_joint = 0;
    joint_angle_deg = 0;
    joint_angle_rad = 0;
    positions.resize(total_joints, 0);
    positions[3] = -1.54;
    positions[5] = 1.54;
    velocities.resize(total_joints, 0);
    accelerations.resize(total_joints, 0);
    effort.resize(total_joints, 0);
  }
}

void Controller::changeMode(int msg_mode_button_state)
{
  mode_button_state = msg_mode_button_state;
  if (!only_nav_mode && mode_button_state != prev_mode_button_state && !mode_button_state)
  {
    switch (mode)
    {
      case Modes::JointPosRel:
        mode = Modes::JointPosAbs;
        joint_angle_deg = 0;
        ROS_INFO_NAMED(LOG_TAG, "%s: Set Joint Position Absolute Mode.", LOG_TAG);
        ROS_INFO_NAMED(LOG_TAG, "%s: Joint %d (%s) selected.", LOG_TAG, selected_joint + 1,
                       joint_names[selected_joint].c_str());
        break;
      case Modes::JointPosAbs:
        mode = Modes::TaskPosRel;
        ROS_INFO_NAMED(LOG_TAG, "%s: Set Task Position Relative Mode.", LOG_TAG);
        break;
      case Modes::TaskPosRel:
        mode = Modes::GetPoints;
        ROS_INFO_NAMED(LOG_TAG, "%s: Set Get Points in Space Mode.", LOG_TAG);
        {
          // Open file to append point data
          std::string path;
          path = "/home/rtonet/ROS/tese/src/panda_3dbioprint_debug_tools";
          pts_fh.open(path + "/data/segmentation_points.dat", std::fstream::out);
          pts_fh << "px py pz ox oy oz ow" << "\n";
        }
        break;
      case Modes::GetPoints:
        mode = Modes::Nav3D;
        ROS_INFO_NAMED(LOG_TAG, "%s: Set Nav3D Mode.", LOG_TAG);
        pts_fh.close(); // Close file
        break;
      case Modes::Nav3D:
        mode = Modes::JointPosRel;
        joint_angle_deg = 0;
        ROS_INFO_NAMED(LOG_TAG, "%s: Set Joint Position Relative Mode.", LOG_TAG);
        ROS_INFO_NAMED(LOG_TAG, "%s: Joint %d (%s) selected.", LOG_TAG, selected_joint + 1,
                       joint_names[selected_joint].c_str());
        break;
      default:
        break;
    }
  }
  prev_mode_button_state = mode_button_state;
}

void Controller::changeJoint(int msg_option_button_state)
{
  option_button_state = msg_option_button_state;
  if (!only_nav_mode && option_button_state != prev_option_button_state && !option_button_state)
  {
    if (++selected_joint == total_joints)
      selected_joint = 0;

    joint_angle_deg = 0;
    ROS_INFO_NAMED(LOG_TAG, "%s: Joint %d (%s) selected.", LOG_TAG, selected_joint + 1,
                   joint_names[selected_joint].c_str());
  }
  prev_option_button_state = option_button_state;
}

void Controller::changeJointAbsoluteValue(float msg_yaxis, float msg_zaxis)
{
  float joint_lower_limit = (robot_joints[joint_names[selected_joint]].get()->limits.get()->lower * 180) / M_PI;
  float joint_upper_limit = (robot_joints[joint_names[selected_joint]].get()->limits.get()->upper * 180) / M_PI;

  if (msg_yaxis > 0.5)
    joy_y_state = JoyStates::Y_POSITIVE;
  if (msg_yaxis < -0.5)
    joy_y_state = JoyStates::Y_NEGATIVE;
  if (msg_yaxis == 0.0)
    joy_y_state = JoyStates::Y_ZERO;
  if (msg_zaxis > 0.5)
    joy_z_state = JoyStates::Z_POSITIVE;
  if (msg_zaxis == 0.0)
    joy_z_state = JoyStates::Z_ZERO;

  // Check joy y state change to update the absolute angle in degrees
  if (joy_y_state != prev_joy_y_state && joy_y_state == JoyStates::Y_ZERO)
  {
    if (prev_joy_y_state == JoyStates::Y_POSITIVE)  // Joy Y is positive
    {
      if (joint_angle_deg == joint_upper_limit)
        // Ajust limit value to keep absolute angles multiple of increment
        joint_angle_deg =
            (ceil(joint_upper_limit / JOINT_ABSOLUTE_ANGLE_INC) * JOINT_ABSOLUTE_ANGLE_INC) - JOINT_ABSOLUTE_ANGLE_INC;
      else
        joint_angle_deg -= JOINT_ABSOLUTE_ANGLE_INC;

      if (joint_angle_deg < joint_lower_limit)
        joint_angle_deg = joint_lower_limit;

      ROS_INFO_NAMED(LOG_TAG, "%s: Joint %d (%s), position: %f.2", LOG_TAG, selected_joint + 1,
                     joint_names[selected_joint].c_str(), joint_angle_deg);
    }
    else  // Joy Y is negative
    {
      if (joint_angle_deg == joint_lower_limit)
        // Ajust limit value to keep absolute angles multiple of increment
        joint_angle_deg =
            (floor(joint_lower_limit / JOINT_ABSOLUTE_ANGLE_INC) * JOINT_ABSOLUTE_ANGLE_INC) + JOINT_ABSOLUTE_ANGLE_INC;
      else
        joint_angle_deg += JOINT_ABSOLUTE_ANGLE_INC;

      if (joint_angle_deg > joint_upper_limit)
        joint_angle_deg = joint_upper_limit;

      ROS_INFO_NAMED(LOG_TAG, "%s: Joint %d (%s), position: %f.2", LOG_TAG, selected_joint + 1,
                     joint_names[selected_joint].c_str(), joint_angle_deg);
    }
  }

  // Check if the joy z axes is negative (joy pressed down) to send the angle to the robot joint
  if (joy_z_state != prev_joy_z_state && joy_z_state == JoyStates::Z_ZERO)
  {
    joint_angle_rad = (joint_angle_deg * M_PI) / 180;
    publishNewJointState(selected_joint, joint_angle_rad);
  }

  prev_joy_y_state = joy_y_state;
  prev_joy_z_state = joy_z_state;
}

void Controller::changeJointRelativeValue(float msg_yaxis)
{
  float joint_lower_limit = (robot_joints[joint_names[selected_joint]].get()->limits.get()->lower * 180) / M_PI;
  float joint_upper_limit = (robot_joints[joint_names[selected_joint]].get()->limits.get()->upper * 180) / M_PI;

  if (msg_yaxis > 0.5)
  {
    joint_angle_deg -= sensitivity_factor;
    if (joint_angle_deg < joint_lower_limit)
      joint_angle_deg = joint_lower_limit;

    joint_angle_rad = (joint_angle_deg * M_PI) / 180;
    ROS_INFO_NAMED(LOG_TAG, "%s: Joint %d (%s), position: %f.2", LOG_TAG, selected_joint + 1,
                   joint_names[selected_joint].c_str(), joint_angle_deg);
    publishNewJointState(selected_joint, joint_angle_rad);
  }
  else if (msg_yaxis < -0.5)
  {
    joint_angle_deg += sensitivity_factor;
    if (joint_angle_deg > joint_upper_limit)
      joint_angle_deg = joint_upper_limit;

    joint_angle_rad = (joint_angle_deg * M_PI) / 180;
    ROS_INFO_NAMED(LOG_TAG, "%s: Joint %d (%s), position: %f.2", LOG_TAG, selected_joint + 1,
                   joint_names[selected_joint].c_str(), joint_angle_deg);
    publishNewJointState(selected_joint, joint_angle_rad);
  }
}

void Controller::changePoseRelativeValue(const sensor_msgs::Joy::ConstPtr &msg)
{
  float posx = msg->axes[0];
  float posy = msg->axes[1];
  float posz = msg->axes[2];
  float orientx = msg->axes[3];
  float orienty = msg->axes[4];
  float orientz = msg->axes[5];
  float orient_sensitivity = sensitivity_factor * 10;

  geometry_msgs::Pose new_pose;
  new_pose.position.x = current_pose.position.x + sensitivity_factor * (-posx);
  new_pose.position.y = current_pose.position.y + sensitivity_factor * (-posy);
  new_pose.position.z = current_pose.position.z + sensitivity_factor * posz;

  Eigen::Quaterniond current_orientation(current_pose.orientation.w, 
                                        current_pose.orientation.x,
                                        current_pose.orientation.y,
                                        current_pose.orientation.z);

  Eigen::Matrix3d Rc = current_orientation.matrix(); // Rotation matrix for current pose
  Eigen::Matrix3d Rs; // Rotation matrix for spacenav
  Eigen::Matrix3d Rsx; // Rotation matrix for spacenav x-axis rotation
  Eigen::Matrix3d Rsy; // Rotation matrix for spacenav y-axis rotation
  Eigen::Matrix3d Rsz; // Rotation matrix for spacenav z-axis rotation

  if (orientx >= 0 && orientx < 0.001 && 
      orienty >= 0 && orienty < 0.001 && 
      orientz >= 0 && orientz < 0.001) 
  {
    Rs << 1, 0, 0,
          0, 1, 0,
          0, 0, 1;     
  }
  else 
  {
    Rsx << 1,                 0,                  0,
          0, cos(orient_sensitivity * orientx), -sin(orient_sensitivity * orientx),
          0, sin(orient_sensitivity * orientx),  cos(orient_sensitivity * orientx);

    Rsy <<  cos(-orient_sensitivity * orienty),  0, sin(-orient_sensitivity * orienty),
                            0, 1,                 0,
          -sin(-orient_sensitivity * orienty), 0, cos(-orient_sensitivity * orienty);

    Rsz << cos(-orient_sensitivity * orientz), -sin(-orient_sensitivity * orientz), 0,
          sin(-orient_sensitivity * orientz),  cos(-orient_sensitivity * orientz), 0,
                          0,                  0, 1;

    Rs = Rsx * Rsy * Rsz;
  }

  Eigen::Matrix3d Rf = Rc * Rs; // In relation to end-effector
  // Eigen::Matrix3d Rf = Rs * Rc; // In relation to base

  Eigen::Quaterniond new_orientation(Rf);

  new_pose.orientation.x = new_orientation.x();
  new_pose.orientation.y = new_orientation.y();
  new_pose.orientation.z = new_orientation.z();
  new_pose.orientation.w = new_orientation.w();

  if (posx || posy || posz || orientx || orienty || orientz)
    publishNewCartesianPose(new_pose);
}

void Controller::publishNewJointState(int joint, float value, float duration)
{
  positions[joint] = value;

  joint_msg.header.stamp = ros::Time::now();
  joint_msg.joint_names = joint_names;
  point.positions = positions;
  point.velocities = velocities;
  point.accelerations = accelerations;
  point.effort = effort;
  point.time_from_start = ros::Duration(duration);
  joint_msg.points.resize(1);
  joint_msg.points[0] = point;

  joint_position_pub.publish(joint_msg);
}

void Controller::publishNewCartesianPose(const geometry_msgs::Pose pose)
{
  pose_msg.position = pose.position;
  pose_msg.orientation = pose.orientation;
  cartesian_pose_pub.publish(pose_msg);
}

void Controller::setupPublishersAndSubscribers(ros::NodeHandle nh, std::string controller_topic, std::string controller_topic_type, 
                                    std::string robot_state_topic, std::string robot_state_topic_type)
{
  // Setup topic advertising to send commands to joints
  if (controller_topic_type.compare("trajectory_msgs::JointTrajectory") == 0)
    joint_position_pub = nh.advertise<trajectory_msgs::JointTrajectory>(controller_topic, 10);
  else if (controller_topic_type.compare("trajectory_msgs::JointTrajectoryPoint") == 0)
    joint_position_pub = nh.advertise<trajectory_msgs::JointTrajectoryPoint>(controller_topic, 10);
  else if (controller_topic_type.compare("geometry_msgs::Accel") == 0)
    cartesian_pose_pub = nh.advertise<geometry_msgs::Accel>(controller_topic, 10);
  else if (controller_topic_type.compare("geometry_msgs::AccelStamped") == 0)
    cartesian_pose_pub = nh.advertise<geometry_msgs::AccelStamped>(controller_topic, 10);
  else if (controller_topic_type.compare("geometry_msgs::AccelWithCovariance") == 0)
    cartesian_pose_pub = nh.advertise<geometry_msgs::AccelWithCovariance>(controller_topic, 10);
  else if (controller_topic_type.compare("geometry_msgs::AccelWithCovarianceStamped") == 0)
    cartesian_pose_pub = nh.advertise<geometry_msgs::AccelWithCovarianceStamped>(controller_topic, 10);
  else if (controller_topic_type.compare("geometry_msgs::Point32") == 0)
    cartesian_pose_pub = nh.advertise<geometry_msgs::Point32>(controller_topic, 10);
  else if (controller_topic_type.compare("geometry_msgs::Point") == 0)
    cartesian_pose_pub = nh.advertise<geometry_msgs::Point>(controller_topic, 10);
  else if (controller_topic_type.compare("geometry_msgs::PointStamped") == 0)
    cartesian_pose_pub = nh.advertise<geometry_msgs::PointStamped>(controller_topic, 10);
  else if (controller_topic_type.compare("geometry_msgs::Pose") == 0)
    cartesian_pose_pub = nh.advertise<geometry_msgs::Pose>(controller_topic, 10);
  else if (controller_topic_type.compare("geometry_msgs::PoseArray") == 0)
    cartesian_pose_pub = nh.advertise<geometry_msgs::PoseArray>(controller_topic, 10);
  else if (controller_topic_type.compare("geometry_msgs::PoseStamped") == 0)
    cartesian_pose_pub = nh.advertise<geometry_msgs::PoseStamped>(controller_topic, 10);
  else if (controller_topic_type.compare("geometry_msgs::PoseWithCovariance") == 0)
    cartesian_pose_pub = nh.advertise<geometry_msgs::PoseWithCovariance>(controller_topic, 10);
  else if (controller_topic_type.compare("geometry_msgs::PoseWithCovarianceStamped") == 0)
    cartesian_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(controller_topic, 10);
  else if (controller_topic_type.compare("geometry_msgs::Quaternion") == 0)
    cartesian_pose_pub = nh.advertise<geometry_msgs::Quaternion>(controller_topic, 10);
  else if (controller_topic_type.compare("geometry_msgs::QuaternionStamped") == 0)
    cartesian_pose_pub = nh.advertise<geometry_msgs::QuaternionStamped>(controller_topic, 10);
  else if (controller_topic_type.compare("geometry_msgs::Twist") == 0)
    cartesian_pose_pub = nh.advertise<geometry_msgs::Twist>(controller_topic, 10);
  else if (controller_topic_type.compare("geometry_msgs::TwistStamped") == 0)
    cartesian_pose_pub = nh.advertise<geometry_msgs::TwistStamped>(controller_topic, 10);
  else if (controller_topic_type.compare("geometry_msgs::TwistWithCovariance") == 0)
    cartesian_pose_pub = nh.advertise<geometry_msgs::TwistWithCovariance>(controller_topic, 10);
  else if (controller_topic_type.compare("geometry_msgs::TwistWithCovarianceStamped") == 0)
    cartesian_pose_pub = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>(controller_topic, 10);
  else if (controller_topic_type.compare("geometry_msgs::Wrench") == 0)
    cartesian_pose_pub = nh.advertise<geometry_msgs::Wrench>(controller_topic, 10);
  else if (controller_topic_type.compare("geometry_msgs::WrenchStamped") == 0)
    cartesian_pose_pub = nh.advertise<geometry_msgs::WrenchStamped>(controller_topic, 10);

  if (robot_state_topic_type.compare("geometry_msgs::Pose") == 0)
    robot_state_sub = nh.subscribe(robot_state_topic, 10, &Controller::getRobotStatePoseCallback, this);
  else if (robot_state_topic_type.compare("geometry_msgs::PoseStamped") == 0)
    robot_state_sub = nh.subscribe(robot_state_topic, 10, &Controller::getRobotStatePoseStampedCallback, this);
  else if (robot_state_topic_type.compare("geometry_msgs::PoseWithCovariance") == 0)
    robot_state_sub = nh.subscribe(robot_state_topic, 10, &Controller::getRobotStatePoseCovCallback, this);
  else if (robot_state_topic_type.compare("geometry_msgs::PoseWithCovarianceStamped") == 0)
    robot_state_sub = nh.subscribe(robot_state_topic, 10, &Controller::getRobotStatePoseCovStampedCallback, this);
}

void Controller::getSpacenavDataCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
  /**
   * msg->axes = [x, y, z, rot_x, rot_y, rot_z]
   * msg->buttons = [left_button, right_button]
   *
   * left and right is defined assuming the 3DConnexion space mouse logo turned to user.
   */

  // Change mode on left button click
  changeMode(msg->buttons[0]);

  // Change joint on right button click if on JointPosRel or JointPosAbs modes
  if (mode == Modes::JointPosAbs || mode == Modes::JointPosRel)
    changeJoint(msg->buttons[1]);

  switch (mode)
  {
    case Modes::JointPosAbs:
      changeJointAbsoluteValue(msg->axes[1], msg->axes[2]);
      break;
    case Modes::JointPosRel:
      changeJointRelativeValue(msg->axes[1]);
      break;
    case Modes::TaskPosRel:
      changePoseRelativeValue(msg);
      break;
    case Modes::GetPoints:
      if (robot)
      {
        savePoints(msg);
      }
      else
      {
        changePoseRelativeValue(msg);
        savePoints(msg);
      }
      break;
    default:
      break;
  }
}

void Controller::savePoints(const sensor_msgs::Joy::ConstPtr &msg)
{
  option_button_state = msg->buttons[1];
  if (!only_nav_mode && option_button_state != prev_option_button_state && !option_button_state)
  {
    if (pts_fh.is_open())
    {
      pts_fh << current_pose.position.x << " " << current_pose.position.y << " " << current_pose.position.z \
      << " " << current_pose.orientation.x << " " << current_pose.orientation.y << " " << current_pose.orientation.z << " " << current_pose.orientation.w << "\n";
      ROS_INFO_NAMED(LOG_TAG, "%s: Stored pose: p(%.4f, %.4f, %.4f) o(%.4f, %.4f, %.4f, %.4f).", LOG_TAG, current_pose.position.x, current_pose.position.y, current_pose.position.z, \
      current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);
    }
    else
    {
      ROS_ERROR("The files were not properly opened!");
    }
  }
  prev_option_button_state = option_button_state;
}

void Controller::getRobotStatePoseCallback(const geometry_msgs::PoseConstPtr &msg)
{
  // Save robot current pose
  current_pose.position = msg->position;
  current_pose.orientation = msg->orientation;
}

void Controller::getRobotStatePoseStampedCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  // Save robot current pose
  current_pose.position = msg->pose.position;
  current_pose.orientation = msg->pose.orientation;
}

void Controller::getRobotStatePoseCovCallback(const geometry_msgs::PoseWithCovarianceConstPtr &msg)
{
  // Save robot current pose
  current_pose.position = msg->pose.position;
  current_pose.orientation = msg->pose.orientation;
}

void Controller::getRobotStatePoseCovStampedCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{
  // Save robot current pose
  current_pose.position = msg->pose.pose.position;
  current_pose.orientation = msg->pose.pose.orientation;
}