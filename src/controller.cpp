#include <spacenav_control/controller.h>

using namespace spacenav;

Controller::Controller(ros::NodeHandle nh, std::string robot_name,
                       std::map<std::string, urdf::JointSharedPtr> robot_joints, std::string controller_topic,
                       Sensitivity sensitivity)
{
  // Set initial mode to Nav3D
  mode = Modes::Nav3D;
  ROS_INFO("SPACENAV_CONTROL: Start on Nav3D Mode.");

  this->robot_name = (!robot_name.empty()) ? robot_name : "robot";

  if (robot_joints.empty() || controller_topic.empty())
  {
    ROS_INFO("Only Nav3D mode allowed!");
    only_nav_mode = true;
  }
  else
  {
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
      ROS_INFO("Only Nav3D mode allowed!");
      only_nav_mode = true;
      return;
    }

    // Setup topic advertising to send commands to joints
    joint_position_pub = nh.advertise<trajectory_msgs::JointTrajectory>(controller_topic, 10);

    // Set default values
    switch (sensitivity)
    {
      case Sensitivity::Low:
        this->sensitivity_factor = LOW_SENSITIVITY;
        ROS_INFO("SPACENAV_CONTROL: Sensitivity set to LOW.");
        break;
      case Sensitivity::High:
        this->sensitivity_factor = HIGH_SENSITIVITY;
        ROS_INFO("SPACENAV_CONTROL: Sensitivity set to HIGH.");
        break;
      default:
        this->sensitivity_factor = MEDIUM_SENSITIVITY;
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
        ROS_INFO("SPACENAV_CONTROL: Joint %d (%s) selected.", selected_joint + 1, joint_names[selected_joint].c_str());
        break;
      case Modes::PosAbs:
        mode = Modes::Nav3D;
        ROS_INFO("SPACENAV_CONTROL: Set Nav3D Mode.");
        break;
      case Modes::Nav3D:
        mode = Modes::PosRel;
        joint_angle_deg = 0;
        ROS_INFO("SPACENAV_CONTROL: Set Joint Position Relative Mode.");
        ROS_INFO("SPACENAV_CONTROL: Joint %d (%s) selected.", selected_joint + 1, joint_names[selected_joint].c_str());
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
    ROS_INFO("SPACENAV_CONTROL: Joint %d (%s) selected.", selected_joint + 1, joint_names[selected_joint].c_str());
  }
  prev_joint_button_state = joint_button_state;
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

      ROS_INFO("SPACENAV_CONTROL: Joint %d (%s), position: %f.2", selected_joint + 1,
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

      ROS_INFO("SPACENAV_CONTROL: Joint %d (%s), position: %f.2", selected_joint + 1,
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
    ROS_INFO("SPACENAV_CONTROL: Joint %d (%s), position: %f.2", selected_joint + 1, joint_names[selected_joint].c_str(),
             joint_angle_deg);
    publishNewJointState(selected_joint, joint_angle_rad);
  }
  else if (msg_yaxis < -0.5)
  {
    joint_angle_deg += sensitivity_factor;
    if (joint_angle_deg > joint_upper_limit)
      joint_angle_deg = joint_upper_limit;

    joint_angle_rad = (joint_angle_deg * M_PI) / 180;
    ROS_INFO("SPACENAV_CONTROL: Joint %d (%s), position: %f.2", selected_joint + 1, joint_names[selected_joint].c_str(),
             joint_angle_deg);
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
      changeJointAbsoluteValue(msg->axes[1], msg->axes[2]);
      break;
    case Modes::PosRel:
      changeJointRelativeValue(msg->axes[1]);
      break;
    default:
      break;
  }
}