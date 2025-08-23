#include "pulse_force_control/gubbi_standalone_force_control_class.hpp"

int main(int argc, char** argv)
{
  // Initialize the node and declare the node handler.
  ros::init(argc, argv, "gubbi_standalone_force_control_node");
  ros::NodeHandle nh("~");
  ROS_DEBUG("gubbi_standalone_force_control_node - main");

  // Start an async spinner to avoid move group interface object complaining
  // about time sync with UR5e.
  // https://github.com/ros-planning/moveit/issues/1187
  // Use more than one thread to avoid MoveIt's move group interface's plan()
  // function freezing.
  // https://github.com/jacknlliu/development-issues/issues/44
  ros::AsyncSpinner spinner(0);
  spinner.start();

  pfc::GubbiStandaloneForceControlClass y(nh);

  // Wait for the node to shutdown.
  ros::waitForShutdown();

  return 0;
}
