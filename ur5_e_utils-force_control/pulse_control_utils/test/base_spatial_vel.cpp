#include "pulse_control_utils/ur5e_base_class.hpp"
#include "pulse_common_utils/eigen_print_utils.hpp"

int main(int argc, char** argv)
{
  std::string ee_name;
  double delta_t;
  double ee_vel;
  double joint_omega_max;
  Eigen::MatrixXd j_body;
  Eigen::MatrixXd j_spatial;
  Eigen::VectorXd v(6);
  int i0;
  int num_steps;
  std::string action_server_name;
  std::vector<double> q;

  ros::init(argc, argv, "spatial_vel_test_node");
  ros::NodeHandle nh("~");
  ros::AsyncSpinner spinner(2);

  spinner.start();
  ROS_DEBUG_STREAM("Debug environment activated.");
  ROS_INFO_STREAM("Fetching action client topic...");

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

  if(nh.hasParam("end_effector_vel"))
  {
    nh.getParam("end_effector_vel", ee_vel);
  }
  else
  {
    ROS_WARN_STREAM(
      "Could not find parameter 'end_effector_vel', using default value...");
    ee_vel = 0.01;
  }

  if(nh.hasParam("num_steps"))
  {
    nh.getParam("num_steps", num_steps);
  }
  else
  {
    ROS_WARN_STREAM(
      "Could not find parameter 'num_steps', using default value...");
    num_steps = 250;
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

  if(argc == 2)
  {
    if(strcmp(argv[1], "x") == 0)
    {
      v << ee_vel, 0.0, 0.0, 0.0, 0.0, 0.0;
    }
    else if(strcmp(argv[1], "y") == 0)
    {
      v << 0.0, ee_vel, 0.0, 0.0, 0.0, 0.0;
    }
    else if(strcmp(argv[1], "z") == 0)
    {
      v << 0.0, 0.0, ee_vel, 0.0, 0.0, 0.0;
    }
    else if(strcmp(argv[1], "wx") == 0)
    {
      v << 0.0, 0.0, 0.0, ee_vel, 0.0, 0.0;
    }
    else if(strcmp(argv[1], "wy") == 0)
    {
      v << 0.0, 0.0, 0.0, 0.0, ee_vel, 0.0;
    }
    else
    {
      // Test rotation along the z-axis by default.
      v << 0.0, 0.0, 0.0, 0.0, 0.0, ee_vel;
    }
  }
  else
  {
    // Test rotation along the z-axis by default.
    v << 0.0, 0.0, 0.0, 0.0, 0.0, ee_vel;
  }

  ROS_INFO("Starting test...");
  Ur5eBaseClass u(nh, action_server_name, ee_name, delta_t, joint_omega_max);

  // Go back to home position
  ROS_INFO_STREAM("Setting goal to home position...");
  q.push_back(0.0);
  q.push_back(-M_PI / 2.0);
  q.push_back(0.0);
  q.push_back(-M_PI / 2.0);
  q.push_back(0.0);
  q.push_back(0.0);

  u.setJointPosition(q);
  ROS_INFO_STREAM("Sending home position goal...");
  u.sendGoalAndWait();
  q.clear();

  ROS_INFO_STREAM("Setting initial position...");
  q.push_back(0.0);
  q.push_back(-1.75);
  q.push_back(-1.75);
  q.push_back(-1.17);
  q.push_back(M_PI / 2.0);
  q.push_back(M_PI / 4.0);

  u.setJointPosition(q);
  ROS_INFO_STREAM("Sending initial position goal...");
  u.sendGoalAndWait();
  q.clear();

  ros::Duration(0.5).sleep();
  const Eigen::Isometry3d fwd_kin_before = u.getGlobalLinkTransform();
  printTransform(fwd_kin_before);

  for(i0 = 0; i0 < num_steps; ++i0)
  {
    u.setSpatialCartesianVelocity(v);

    ROS_INFO_STREAM(
      "[" << (i0 + 1) << "/" << num_steps
          << "] Setting spatial velocity of frame " << ee_name << " to "
          << std::endl
          << "[" << v(0) << ", " << v(1) << ", " << v(2) << ", " << v(3) << ", "
          << v(4) << ", " << v(5) << "] <m/rad>/s...");
    ROS_INFO_STREAM(
      "[" << (i0 + 1) << "/" << num_steps << "] Sending joint vector goal...");
    u.sendGoalAndWait();
    ros::Duration(0.5).sleep();
  }

  ROS_INFO_STREAM("Joint trajectories executed.");
  printTransform(fwd_kin_before);

  ros::Duration(0.5).sleep();
  const Eigen::Isometry3d fwd_kin_after = u.getGlobalLinkTransform();
  printTransform(fwd_kin_after);

  // Compute and print transform between before and after robot movement.
  Eigen::Vector3d t_obs =
    fwd_kin_after.translation() - fwd_kin_before.translation();
  ROS_INFO_STREAM("Observed difference:" << std::endl << t_obs);

  Eigen::Vector3d t_ref(v(0), v(1), v(2));
  t_ref *= num_steps * delta_t;
  ROS_INFO_STREAM("Expected difference:" << std::endl << t_ref);

  Eigen::Vector3d t_err = t_ref - t_obs;
  ROS_INFO("Translational error: %.3f mm", t_err.norm() * 1e3);

  ros::shutdown();

  return 0;
}
