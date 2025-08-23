#include "pulse_ur5e_sweep/ur5e_gubbi_sweep_class.hpp"

int main(int argc, char** argv)
{
  std::string ee_name;
  double delta_t;
  std::string action_server_name;

  // Initialize the node and declare the node handler.
  ros::init(argc, argv, "ur5e_gubbi_sweep_node");
  ros::NodeHandle nh;

  // Start an async spinner to avoid move group interface object complaining
  // about time sync with UR5e.
  // https://github.com/ros-planning/moveit/issues/1187
  // Use more than one thread to avoid MoveIt's move group interface's plan()
  // function freezing.
  // https://github.com/jacknlliu/development-issues/issues/44
  ros::AsyncSpinner spinner(0);
  spinner.start();

  if(nh.hasParam("/action_server_name"))
  {
    nh.getParam("/action_server_name", action_server_name);
  }
  else
  {
    ROS_WARN_STREAM("Could not find parameter '/action_server_name', using default value...");
    action_server_name = "/scaled_pos_joint_traj_controller/follow_joint_trajectory";
  }

  if(nh.hasParam("/end_effector_name"))
  {
    nh.getParam("/end_effector_name", ee_name);
  }
  else
  {
    ROS_WARN_STREAM("Could not find parameter '/end_effector_name', using default value...");
    ee_name = "tool0";
  }

  if(nh.hasParam("delta_t"))
  {
    nh.getParam("delta_t", delta_t);
  }
  else
  {
    ROS_WARN_STREAM("Could not find parameter 'delta_t', using default value...");
    delta_t = 0.1;
  }

  // Instantiate the UR5e object.
  Ur5eGubbiSweepClass u(nh, action_server_name, ee_name, delta_t);

  // Wait for the node to shutdown.
  ros::waitForShutdown();

  return 0;
}
