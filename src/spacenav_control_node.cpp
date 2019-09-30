#include <ros/ros.h>
#include <spacenav_control/controller.h>
#include <stdlib.h>
#include <string.h>

const std::string spacenav_topic = "/spacenav/joy";
const std::string joint_names_param = "/spacenav_control/joint_names";
const std::string controller_topic_param = "/spacenav_control/controller_topic";

std::string joint_names_str, controller_topic;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "spacenav_control_node");
  ros::NodeHandle nh;

  spacenav::Sensitivity sensitivity;

  // Process command-line arguments
  int opt;
  while ((opt = getopt(argc, argv, ":s:")) != -1)
  {
    switch (opt)
    {
      case 's':
        sensitivity = static_cast<spacenav::Sensitivity>(std::stoi(optarg));
        break;
      default:
        break;
    }
  }

  if (nh.hasParam(joint_names_param) && nh.hasParam(controller_topic_param))
  {
    nh.getParam(joint_names_param, joint_names_str);
    nh.getParam(controller_topic_param, controller_topic);
  }
  else
  {
    joint_names_str = "";
    controller_topic = "";
  }
  spacenav::Controller spnav(nh, joint_names_str, controller_topic, sensitivity);

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
