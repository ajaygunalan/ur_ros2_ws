#include "pulse_force_control/simple_z_control_class.hpp"

int main(int argc, char** argv)
{
  // Initialize the node and declare the node handler.
  ros::init(argc, argv, "simple_z_control_node");
  ros::NodeHandle nh("~");

  std::string action_server_name = "/scaled_pos_joint_traj_controller/follow_joint_trajectory";

  // Start an async spinner to avoid move group interface object complaining about time sync with UR5e.
  // https://github.com/ros-planning/moveit/issues/1187
  // Use more than one thread to avoid MoveIt's move group interface's plan() function freezing.
  // https://github.com/jacknlliu/development-issues/issues/44
  ros::AsyncSpinner spinner(0);
  spinner.start();

  // Instantiate the force control object.
  SimpleZControlClass s(nh, action_server_name, "p42v_link1", 0.016, (M_PI / 10.0));

  // Wait for the node to shutdown.
  ros::waitForShutdown();

  return 0;
}

