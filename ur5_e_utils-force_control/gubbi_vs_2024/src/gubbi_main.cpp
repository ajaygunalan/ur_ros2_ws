#include "gubbi_vs_2024/gubbi_class.hpp"

int main(int argc, char** argv)
{
  std::string action_server_name;
  std::string ee_name;
  double delta_t;
  double joint_omega_max;
  double joint_alpha_max;

  // Initialize the node and declare the node handler.
  ros::init(argc, argv, "visual_servoing_node");
  ros::NodeHandle nh("~");
  ROS_INFO("Node namespace: %s", nh.getNamespace().c_str());

  if(nh.hasParam("action_server_name"))
  {
    nh.getParam("action_server_name", action_server_name);
  }
  else
  {
    ROS_WARN("Could not find parameter 'action_server_name', using default value...");
    action_server_name = "/scaled_pos_joint_traj_controller/follow_joint_trajectory";
  }

  if(nh.hasParam("probe_frame_name"))
  {
    nh.getParam("probe_frame_name", ee_name);
  }
  else
  {
    ROS_WARN("Could not find parameter 'probe_frame_name', using default value...");
    ee_name = "p42v_link1";
  }

  if(nh.hasParam("ctrl_period"))
  {
    nh.getParam("ctrl_period", delta_t);
  }
  else
  {
    ROS_WARN("Could not find parameter 'ctrl_period', using default value...");
    delta_t = 0.032;
  }

  if(nh.hasParam("joint_omega_max"))
  {
    nh.getParam("joint_omega_max", joint_omega_max);
  }
  else
  {
    ROS_WARN("Could not find parameter 'joint_omega_max', using default value...");
    joint_omega_max = (M_PI / 5.0);
  }

  if(nh.hasParam("joint_alpha_max"))
  {
    nh.getParam("joint_alpha_max", joint_alpha_max);
  }
  else
  {
    ROS_WARN("Could not find parameter 'joint_alpha_max', using default value...");
    joint_alpha_max = (M_PI / 2.0);
  }

  // Start an async spinner to avoid move group interface object complaining about time sync with UR5e.
  // https://github.com/ros-planning/moveit/issues/1187
  // Use more than one thread to avoid MoveIt's move group interface's plan() function freezing.
  // https://github.com/jacknlliu/development-issues/issues/44
  ros::AsyncSpinner spinner(0);
  spinner.start();

  GubbiVisualServoingClass gvs(nh, action_server_name, ee_name, delta_t, joint_omega_max, joint_alpha_max);

  ros::waitForShutdown();

  return 0;
}
