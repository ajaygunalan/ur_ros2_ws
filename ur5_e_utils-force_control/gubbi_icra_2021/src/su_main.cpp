#include "gubbi_icra_2021/su_class.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "su_visual_servoing_node");
  ros::NodeHandle nh("~");
  ros::AsyncSpinner spinner(2);
  std::string action_server_name;
  std::string ee_name;
  std::string save_path;
  int needle_filter_threshold;
  int moment_filter_threshold;
  int moment_kernel_size;
  int needle_kernel_size;
  bool save_img;
  bool debug_segmentation;
  double delta_t;
  double joint_omega_max;
  double unknown_loc_vy;
  double find_tip_vx;
  double max_aligning_wz;

  spinner.start();

  ROS_INFO_STREAM("Fetching action client topic...");
  ROS_DEBUG_STREAM("Debug printing enabled...");

  if(nh.hasParam("/action_server_name"))
  {
    nh.getParam("/action_server_name", action_server_name);
  }
  else
  {
    ROS_WARN_STREAM(
      "Could not find parameter '/action_server_name', using default value...");
    action_server_name =
      "/scaled_pos_joint_traj_controller/follow_joint_trajectory";
  }

  if(nh.hasParam("/end_effector_name"))
  {
    nh.getParam("/end_effector_name", ee_name);
  }
  else
  {
    ROS_WARN_STREAM(
      "Could not find parameter '/end_effector_name', using default value...");
    ee_name = "tool0";
  }

  ROS_INFO_STREAM("Fetching time-step for motion commands...");

  if(nh.hasParam("delta_t"))
  {
    nh.getParam("delta_t", delta_t);
  }
  else
  {
    ROS_WARN_STREAM(
      "Could not find parameter 'delta_t', using default value...");
    delta_t = 0.1;
  }

  if(nh.hasParam("joint_omega_max"))
  {
    nh.getParam("joint_omega_max", joint_omega_max);
  }
  else
  {
    ROS_WARN_STREAM(
      "Could not find parameter 'joint_omega_max', using default value...");
    joint_omega_max = 0.1;
  }

  if(nh.hasParam("needle_filter_threshold"))
  {
    nh.getParam("needle_filter_threshold", needle_filter_threshold);
    ROS_INFO_STREAM("Needle filter threshold = " << needle_filter_threshold);
  }
  else
  {
    ROS_WARN_STREAM(
      "Could not find parameter 'needle filter threshold', using default...");
    needle_filter_threshold = 125;
  }

  if(nh.hasParam("moment_filter_threshold"))
  {
    nh.getParam("moment_filter_threshold", moment_filter_threshold);
    ROS_INFO_STREAM("Moment filter threshold = " << moment_filter_threshold);
  }
  else
  {
    ROS_WARN_STREAM(
      "Could not find parameter 'moment_filter_threshold', using default...");
    moment_filter_threshold = 125;
  }

  if(nh.hasParam("moment_kernel_size"))
  {
    nh.getParam("moment_kernel_size", moment_kernel_size);
  }
  else
  {
    ROS_WARN_STREAM(
      "Could not find parameter 'moment_kernel_size', using default...");
    moment_kernel_size = 9;  // s parameter in moment filter
  }

  if(nh.hasParam("needle_kernel_size"))
  {
    nh.getParam("needle_kernel_size", needle_kernel_size);
  }
  else
  {
    ROS_WARN_STREAM(
      "Could not find parameter 'needle_kernel_size', using default...");
    needle_kernel_size = 29;  // as reported in Mathiassen et al.
  }

  if(nh.hasParam("save_img"))
  {
    nh.getParam("save_img", save_img);
  }
  else
  {
    ROS_WARN_STREAM("Could not find parameter 'save_img', using default...");
    save_img = false;
  }

  if(nh.hasParam("debug_segmentation"))
  {
    nh.getParam("debug_segmentation", debug_segmentation);
  }
  else
  {
    ROS_WARN_STREAM(
      "Could not find parameter 'debug_segmentation', using default...");
    debug_segmentation = false;
  }

  if(nh.hasParam("unknown_loc_vy"))
  {
    nh.getParam("unknown_loc_vy", unknown_loc_vy);
  }
  else
  {
    ROS_WARN_STREAM(
      "Could not find parameter 'unknown_loc_vy', using default...");
    unknown_loc_vy = 0.002;
  }

  if(nh.hasParam("find_tip_vx"))
  {
    nh.getParam("find_tip_vx", find_tip_vx);
  }
  else
  {
    ROS_WARN_STREAM("Could not find parameter 'find_tip_vx', using default...");
    find_tip_vx = 0.002;
  }

  if(nh.hasParam("max_aligning_wz"))
  {
    nh.getParam("max_aligning_wz", max_aligning_wz);
  }
  else
  {
    ROS_WARN_STREAM(
      "Could not find parameter 'max_aligning_wz', using default...");
    max_aligning_wz = 0.002;
  }

  if(nh.hasParam("save_path"))
  {
    nh.getParam("save_path", save_path);
  }
  else
  {
    ROS_WARN_STREAM("Could not find parameter 'save_path', using default...");
    save_path = "";
  }
  ROS_INFO_STREAM("Instantiating Visual Servoing Node...");
  Ur5eSuVisualServoingClass vs(
    nh, action_server_name, ee_name, delta_t, joint_omega_max,
    needle_filter_threshold, moment_filter_threshold, needle_kernel_size,
    moment_kernel_size, save_img, save_path, debug_segmentation, unknown_loc_vy,
    find_tip_vx, max_aligning_wz);
  ros::waitForShutdown();

  return 0;
}

