#include "pulse_force_estimation/yu_bias_comp_class.hpp"

int main(int argc, char** argv)
{
  std::string robot_base_name;
  std::string robot_tool_name;
  std::string sensor_base_name;

  // Initialize the node and declare the node handler.
  ros::init(argc, argv, "yu_bias_comp_node");
  ros::NodeHandle nh("~");

  // Start an async spinner to avoid move group interface object complaining
  // about time sync with UR5e.
  // https://github.com/ros-planning/moveit/issues/1187
  // Use more than one thread to avoid MoveIt's move group interface's plan()
  // function freezing.
  // https://github.com/jacknlliu/development-issues/issues/44
  ros::AsyncSpinner spinner(0);
  spinner.start();

  if(nh.hasParam("robot_base_name"))
  {
    nh.getParam("robot_base_name", robot_base_name);
  }
  else
  {
    robot_base_name = "base_link";
  }

  if(nh.hasParam("robot_tool_name"))
  {
    nh.getParam("robot_tool_name", robot_tool_name);
  }
  else
  {
    robot_tool_name = "tool0";
  }

  if(nh.hasParam("sensor_base_name"))
  {
    nh.getParam("sensor_base_name", sensor_base_name);
  }
  else
  {
    // See CAD drawing of NET-FT sensor which clearly marks sensor origin at
    // frame `netft_link1`.
    sensor_base_name = "netft_link1";
  }

  YuBiasCompClass y(nh, robot_base_name, robot_tool_name, sensor_base_name);

  ros::waitForShutdown();

  return 0;
}
