#include <ros/ros.h>
#include <spacenav_control/controller.h>
#include <stdlib.h>
#include <string.h>
#include <urdf/model.h>

const std::string spacenav_topic = "/spacenav/joy";
const std::string controller_topic_param = "/spacenav_control/controller_topic";
const std::string controller_topic_type_param = "/spacenav_control/controller_topic_type";
const std::string robot_state_topic_param = "/spacenav_control/robot_state_topic";
const std::string robot_state_topic_type_param = "/spacenav_control/robot_state_topic_type";
const std::string markers_topic_param = "/spacenav_control/markers_topic";
const std::string robot_description_param = "/robot_description";

std::string controller_topic, controller_topic_type, 
            robot_state_topic, robot_state_topic_type,
            markers_topic, robot_description;
std::string robot_name;
std::map<std::string, urdf::JointSharedPtr> robot_joints;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "spacenav_control_node");
  ros::NodeHandle nh;

  spacenav::Sensitivity sensitivity;
  bool is_robot = false;
  std::string active_modes = "3";

  // Process command-line arguments
  int opt;
  while ((opt = getopt(argc, argv, ":s:r:m:")) != -1)
  {
    switch (opt)
    {
      case 's':
        sensitivity = static_cast<spacenav::Sensitivity>(std::stoi(optarg));
        break;
      case 'r':
        is_robot = static_cast<bool>(std::stoi(optarg));
        break;
      case 'm':
        active_modes = optarg;
        break;
      default:
        break;
    }
  }

  if (nh.hasParam(robot_description_param))
  {
    urdf::Model model;
    if (model.initParam(robot_description_param))
    {
      robot_name = model.getName();
      robot_joints = model.joints_;
    }
    else
    {
      ROS_WARN("Unable to read the robot model from URDF.");
    }
  }
  else 
  {
    ROS_WARN("The 'robot_description_param' parameter is undefined.");
  }

  if (nh.hasParam(controller_topic_param))
  {
    nh.getParam(controller_topic_param, controller_topic);
  } 
  else 
  {
    ROS_WARN("The 'controller_topic_param' parameter is undefined.");
  }

  if (nh.hasParam(controller_topic_type_param))
  {
    nh.getParam(controller_topic_type_param, controller_topic_type);
  } 
  else 
  {
    ROS_WARN("The 'controller_topic_type_param' parameter is undefined.");
  }

  if (nh.hasParam(robot_state_topic_param))
  {
    nh.getParam(robot_state_topic_param, robot_state_topic);
  }
  else 
  {
    ROS_WARN("The 'controller_topic_param' parameter is undefined.");
  }

  if (nh.hasParam(robot_state_topic_type_param))
  {
    nh.getParam(robot_state_topic_type_param, robot_state_topic_type);
  }
  else 
  {
    ROS_WARN("The 'controller_topic_type_param' parameter is undefined.");
  }

  if (nh.hasParam(markers_topic_param))
  {
    nh.getParam(markers_topic_param, markers_topic);
  }
  else 
  {
    ROS_WARN("The 'markers_topic_param' parameter is undefined.");
  }

  spacenav::Controller spnav(nh, robot_name, robot_joints, 
                            controller_topic, controller_topic_type,
                            robot_state_topic, robot_state_topic_type, markers_topic,
                            sensitivity, is_robot, active_modes);

  // Subscribe to /spacenav/joy topic which is published by spacenav_node
  // with spacenav's six degrees of freedom and buttons
  ros::Subscriber sub = nh.subscribe(spacenav_topic, 10, &spacenav::Controller::getSpacenavDataCallback, &spnav);

  ros::Rate r(10);  // 10 Hz
  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
