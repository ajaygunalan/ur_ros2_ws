/**
 *
 * \file visual_servoing_main.cpp
 *
 * \brief Main executable for visual servoing node
 *
 * \author Mardava Gubbi <mgubbi1@jhu.edu>
 *
 */
#include "gubbi_icra_2021/gubbi_class.h"

int main(int argc, char** argv)
{
  // Initialize the node and declare the node handler.
  ros::init(argc, argv, "visual_servoing_node");

  double delta_t;
  double joint_omega_max;
  double search_r_dot;
  double search_omega;
  int search_max_count;
  int wait_max_count;
  ros::NodeHandle nh;
  std::string action_server_name;
  std::string ee_name;
  std::string tgt_loc_ns;

  // Start an async spinner for planning, even though that might not be required
  // for a simple action client class.
  ros::AsyncSpinner spinner(2);
  spinner.start();

  if(nh.hasParam("/visual_servoing/action_server_name"))
  {
    ROS_DEBUG_STREAM("Fetching parameter '/visual_servoing/action_server_name'...");
    nh.getParam("/visual_servoing/action_server_name", action_server_name);
  }
  else
  {
    ROS_WARN_STREAM("Setting action server name to "
                    "'/scaled_pos_joint_traj_controller/follow_joint_trajectory'...");
    action_server_name = "/scaled_pos_joint_traj_controller/follow_joint_trajectory";
  }

  if(nh.hasParam("/visual_servoing/ee_name"))
  {
    ROS_DEBUG_STREAM("Fetching parameter '/visual_servoing/ee_name'...");
    nh.getParam("/visual_servoing/ee_name", ee_name);
  }
  else
  {
    ROS_DEBUG_STREAM("Setting end effector name to 'p42v_link1'...");
    ee_name = "p42v_link1";
  }

  if(nh.hasParam("/visual_servoing/delta_t"))
  {
    ROS_DEBUG_STREAM("Fetching parameter '/visual_servoing/delta_t'...");
    nh.getParam("/visual_servoing/delta_t", delta_t);
  }
  else
  {
    ROS_DEBUG_STREAM("Setting control system time step to 0.1 s...");
    delta_t = 0.1;
  }

  if(nh.hasParam("/visual_servoing/joint_omega_max"))
  {
    ROS_DEBUG_STREAM("Fetching parameter '/visual_servoing/joint_omega_max'...");
    nh.getParam("/visual_servoing/joint_omega_max", joint_omega_max);
  }
  else
  {
    ROS_DEBUG_STREAM("Setting maximum joint angular velocity to 0.1 rad/s...");
    joint_omega_max = 0.1;
  }

  if(nh.hasParam("/visual_servoing/search_r_dot"))
  {
    ROS_DEBUG_STREAM("Fetching parameter '/visual_servoing/search_r_dot'...");
    nh.getParam("/visual_servoing/search_r_dot", search_r_dot);
  }
  else
  {
    ROS_DEBUG_STREAM("Setting search spiral increase rate to 5 mm/s...");
    search_r_dot = 0.005;
  }

  if(nh.hasParam("/visual_servoing/search_omega"))
  {
    ROS_DEBUG_STREAM("Fetching parameter '/visual_servoing/search_omega'...");
    nh.getParam("/visual_servoing/search_omega", search_omega);
  }
  else
  {
    ROS_DEBUG_STREAM("Setting search spiral angular velocity to 0.1 rad/s...");
    search_omega = 0.1;
  }

  if(nh.hasParam("/visual_servoing/wait_max_count"))
  {
    ROS_DEBUG_STREAM("Fetching parameter '/visual_servoing/wait_max_count'...");
    nh.getParam("/visual_servoing/wait_max_count", wait_max_count);
  }
  else
  {
    // Assume a laser pulse repetition frequency of 10 Hz
    // (Opotek PHOCUS Mobile laser).
    ROS_DEBUG_STREAM("Setting maximum wait time prior to search initiation to 1 s...");
    wait_max_count = 10;
  }

  if(nh.hasParam("/visual_servoing/search_max_count"))
  {
    ROS_DEBUG_STREAM("Fetching parameter '/visual_servoing/search_max_count'...");
    nh.getParam("/visual_servoing/search_max_count", search_max_count);
  }
  else
  {
    // Assume a laser pulse repetition frequency of 10 Hz
    // (Opotek PHOCUS Mobile laser).
    ROS_DEBUG_STREAM("Setting maximum search time prior to search termination to 5 s...");

    search_max_count = 50;
  }

  // A single invalid pose counter (see Ur5eGubbiVisualServoingClass) is used
  // for both waiting and searching, so search_max_count must be the
  // cumulative sum of the desired wait and search times to work correctly.
  search_max_count += wait_max_count;

  if(nh.hasParam("/visual_servoing/tgt_loc_ns"))
  {
    ROS_DEBUG_STREAM("Fetching parameter '/visual_servoing/tgt_loc_ns'...");
    nh.getParam("/visual_servoing/tgt_loc_ns", tgt_loc_ns);
  }
  else
  {
    ROS_DEBUG_STREAM("Setting target localization namespace to '/deep_learning'...");
    tgt_loc_ns = "/deep_learning";
  }

  ROS_DEBUG("Testing visual servoing system...");
  Ur5eGubbiVisualServoingClass ur5e_gubbi_vs(
    nh, action_server_name, ee_name, delta_t, joint_omega_max, search_r_dot, search_omega, wait_max_count,
    search_max_count, tgt_loc_ns);

  ros::waitForShutdown();

  return 0;
}
