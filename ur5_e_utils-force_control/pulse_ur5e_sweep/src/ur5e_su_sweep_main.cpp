#include "pulse_ur5e_sweep/ur5e_su_sweep_class.hpp"

int main(int argc, char** argv)
{
  // Initialize the node and declare the node handler.
  ros::init(argc, argv, "ur5e_su_sweep_node");
  ros::NodeHandle nh;

  double scan_dist = .03;

  // Start an async spinner to avoid move group interface object complaining
  // about time sync with UR5e.
  // https://github.com/ros-planning/moveit/issues/1187
  // Use more than one thread to avoid MoveIt's move group interface's plan()
  // function freezing.
  // https://github.com/jacknlliu/development-issues/issues/44
  ros::AsyncSpinner spinner(0);
  spinner.start();

  // Instantiate the UR5e Base Class object.
  Ur5eSuSweepClass u(nh, scan_dist, 3, 3);

  // Wait for the node to shutdown.
  ros::waitForShutdown();

  return 0;
}
