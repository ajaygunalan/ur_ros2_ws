#include "gubbi_icra_2021/mathiassen_class.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mathiassen_visual_servoing_node");
  ros::NodeHandle nh("~");
  ros::AsyncSpinner spinner(2);
  std::string action_server_name;
  std::string ee_name;
  int needle_filter_threshold;
  int moment_filter_threshold;
  int moment_kernel_size;
  int needle_kernel_size;
  bool save_img;
  bool debug_segmentation;
  bool force_control;
  double r_lambda;
  double r0;
  double k;
  double needle_rad;
  double delta_t;
  double joint_omega_max;

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

  if(!nh.getParam("save_img", save_img))
  {
    ROS_WARN_STREAM("Could not find parameter 'save_img', using default...");
    save_img = false;
  }
  else
  {
    // No operation
  }

  if(!nh.getParam("r_lambda", r_lambda))
  {
    ROS_WARN_STREAM("Could not find parameter 'r_lambda', using default...");
    r_lambda = 5;
  }
  else
  {
    // No operation
  }

  if(!nh.getParam("k", k))
  {
    ROS_WARN_STREAM("Could not find parameter 'k', using default...");
    k = 0.4;
  }
  else
  {
    // No operation
  }

  if(!nh.getParam("needle_rad", needle_rad))
  {
    ROS_WARN_STREAM("Could not find parameter 'needle_rad', using default...");
    needle_rad = 200;
  }
  else
  {
    // No operation
  }

  if(!nh.getParam("r0", r0))
  {
    ROS_WARN_STREAM("Could not find parameter 'r0', using default...");
    r0 = needle_rad;
  }
  else
  {
    // No operation
  }

  if(!nh.getParam("debug_segmentation", debug_segmentation))
  {
    ROS_WARN_STREAM(
      "Could not find parameter 'debug_segmentation', using default...");
    debug_segmentation = false;
  }
  else
  {
    // No operation
  }

  if(!nh.getParam("force_control", force_control))
  {
    ROS_WARN_STREAM(
      "Could not find parameter 'force_control', using default...");
    force_control = false;
  }
  else
  {
    // No operation
  }

  ROS_INFO_STREAM("Instantiating Visual Servoing Node...");
  Ur5eMathiassenVisualServoingClass vs(
    nh, action_server_name, ee_name, delta_t, joint_omega_max,
    needle_filter_threshold, moment_filter_threshold, needle_kernel_size,
    moment_kernel_size, save_img, r_lambda, r0, k, needle_rad,
    debug_segmentation, force_control);
  ros::waitForShutdown();

  return 0;
}

