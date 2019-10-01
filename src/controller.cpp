#include <spacenav_control/controller.h>

using namespace spacenav;

Controller::Controller(ros::NodeHandle nh, std::string joint_names_str, std::string controller_topic,
                       Sensitivity sensitivity)
{
  // Set initial mode to Nav3D
  mode = Modes::Nav3D;
  ROS_INFO("SPACENAV_CONTROL: Start on Nav3D Mode.");

  if (joint_names_str.empty() || controller_topic.empty())
  {
    ROS_INFO("Only Nav3D mode allowed!");
    only_nav_mode = true;
  }
  {
    // Separate joint names by ',' to a string array
    boost::split(joint_names, joint_names_str, [](char c) { return c == ','; });
    total_joints = joint_names.size();

    joint_position_pub = nh.advertise<trajectory_msgs::JointTrajectory>(controller_topic, 10);

    // Set default values
    switch (sensitivity)
    {
      case Sensitivity::Low:
        this->sensitivity = LOW_SENSITIVITY;
        ROS_INFO("SPACENAV_CONTROL: Sensitivity set to LOW.");
        break;
      case Sensitivity::High:
        this->sensitivity = HIGH_SENSITIVITY;
        ROS_INFO("SPACENAV_CONTROL: Sensitivity set to HIGH.");
        break;
      default:
        this->sensitivity = MEDIUM_SENSITIVITY;
        ROS_INFO("SPACENAV_CONTROL: Sensitivity set to MEDIUM.");
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
      case Modes::PosRel:
        mode = Modes::PosAbs;
        joint_angle_deg = 0;
        ROS_INFO("SPACENAV_CONTROL: Set Joint Position Absolute Mode.");
        ROS_INFO("SPACENAV_CONTROL: Joint %d selected.", selected_joint + 1);
        break;
      case Modes::PosAbs:
        mode = Modes::Nav3D;
        ROS_INFO("SPACENAV_CONTROL: Set Nav3D Mode.");
        break;
      case Modes::Nav3D:
        mode = Modes::PosRel;
        joint_angle_deg = 0;
        ROS_INFO("SPACENAV_CONTROL: Set Joint Position Relative Mode.");
        ROS_INFO("SPACENAV_CONTROL: Joint %d selected.", selected_joint + 1);
        break;
      default:
        break;
    }
  }
  prev_mode_button_state = mode_button_state;
}

void Controller::changeJoint(int msg_joint_button_state)
{
  joint_button_state = msg_joint_button_state;
  if (!only_nav_mode && joint_button_state != prev_joint_button_state && !joint_button_state)
  {
    if (++selected_joint == total_joints)
      selected_joint = 0;

    joint_angle_deg = 0;
    ROS_INFO("SPACENAV_CONTROL: Joint %d selected.", selected_joint + 1);
  }
  prev_joint_button_state = joint_button_state;
}

void Controller::changeJointAbsoluteValue(float msg_yaxis)
{
  if (msg_yaxis > 0.5)
  {
    if (joint_angle_deg > -180)
      joint_angle_deg -= 30;

    joint_angle_rad = (joint_angle_deg * M_PI) / 180;
    ROS_INFO("SPACENAV_CONTROL: Joint %d, position: %f.2", selected_joint + 1, joint_angle_deg);
  }
  else if (msg_yaxis < -0.5)
  {
    if (joint_angle_deg < 180)
      joint_angle_deg += 30;

    joint_angle_rad = (joint_angle_deg * M_PI) / 180;
    ROS_INFO("SPACENAV_CONTROL: Joint %d, position: %f.2", selected_joint + 1, joint_angle_deg);
  }
}

void Controller::changeJointRelativeValue(float msg_yaxis)
{
  if (msg_yaxis > 0.5)
  {
    if (joint_angle_deg > -180)
      joint_angle_deg -= sensitivity;

    joint_angle_rad = (joint_angle_deg * M_PI) / 180;
    ROS_INFO("SPACENAV_CONTROL: Joint %d, position: %f.2", selected_joint + 1, joint_angle_deg);
    publishNewJointState(selected_joint, joint_angle_rad);
  }
  else if (msg_yaxis < -0.5)
  {
    if (joint_angle_deg < 180)
      joint_angle_deg += sensitivity;

    joint_angle_rad = (joint_angle_deg * M_PI) / 180;
    ROS_INFO("SPACENAV_CONTROL: Joint %d, position: %f.2", selected_joint + 1, joint_angle_deg);
    publishNewJointState(selected_joint, joint_angle_rad);
  }
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

  // Change joint on right button click if on PosRel or PosAbs modes
  if (mode == Modes::PosAbs || mode == Modes::PosRel)
    changeJoint(msg->buttons[1]);

  switch (mode)
  {
    case Modes::PosAbs:
      changeJointAbsoluteValue(msg->axes[1]);
      break;
    case Modes::PosRel:
      changeJointRelativeValue(msg->axes[1]);
      break;
    default:
      break;
  }
}