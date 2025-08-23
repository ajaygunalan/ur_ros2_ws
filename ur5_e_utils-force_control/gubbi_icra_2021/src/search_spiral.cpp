/**
 *
 * \file search_spiral.cpp
 *
 * \brief Class for spiral search pattern
 *
 * \author Mardava Gubbi <mgubbi1@jhu.edu>
 *
 */
#include "gubbi_icra_2021/search_spiral.h"

/*
 * \brief Construct an object of the type Ur5eSearchClass.
 */
Ur5eSearchSpiralClass::Ur5eSearchSpiralClass(
  ros::NodeHandle& nh, std::string action_server_name, std::string search_ns,
  std::string path_planning_ns, std::string end_effector_ns,
  std::string joint_ns)
  : Ur5eSearchClass(
    nh, action_server_name, search_ns, path_planning_ns, end_effector_ns,
    joint_ns)
{
  // Fetch the parameters from the parameter server.
  std::string search_omega_param_name = search_ns + "/omega";
  std::string search_r_dot_param_name = search_ns + "/r_dot";

  if(!this->nh.getParam(search_omega_param_name, this->search_omega))
  {
    ROS_WARN_STREAM(
      ""
      << "Could not find parameter '" << search_omega_param_name
      << "', using default value...");
    this->search_omega = M_PI / 10.0;
  }
  else
  {
    // No operation
  }

  if(!this->nh.getParam(search_r_dot_param_name, this->search_r_dot))
  {
    ROS_WARN_STREAM(
      ""
      << "Could not find parameter '" << search_r_dot_param_name
      << "', using default value...");
    this->search_r_dot = 0.01;
  }
  else
  {
    // No operation
  }
}

/*
 * \brief Plan the updated iteration of the search path.
 */
bool Ur5eSearchSpiralClass::planSearchPath(void)
{
  bool path_planned = true;
  double t;
  Eigen::Isometry3d x_goal_ee, x_goal_base;
  int i, j, n;
  ros::Time t_now = ros::Time::now();

  // Message containing information of each intermediate point in the joint
  // trajectory.
  trajectory_msgs::JointTrajectoryPoint jtp_msg;

  // Clear the points vector to ensure that the previous trajectory is not
  // retransmitted to the robot.
  this->follow_joint_traj_goal_msg.trajectory.points.clear();

  for(i = 0; i < NUM_ROBOT_JOINTS; ++i)
  {
    jtp_msg.positions.push_back(0.0);
  }

  // Initialize the current goal of the search pattern in the tool0 frame to
  // the identity transformation to prevent rotation of the end effector frame
  // with respect to the base link frame.
  x_goal_ee.setIdentity();

  // Use the floor function to compute the number of time steps instead of the
  // ceiling to allow for situations where path_planning_total_t is a perfect
  // multiple of path_planning_delta_t.
  n = std::floor(this->path_planning_total_t / this->path_planning_delta_t) + 1;

  for(i = 0; i < n; ++i)
  {
    // Compute the time to span 'n - 1' equal intervals across a span of width
    // 'path_planning_total_t', with both end points included.
    if(i < n - 1)
    {
      t = (this->search_iteration * this->path_planning_total_t)
        + (i * this->path_planning_delta_t);
    }
    else
    {
      t = (this->search_iteration + 1) * this->path_planning_total_t;
    }

    x_goal_ee(0, 3) = this->search_r_dot * t * cos(this->search_omega * t);
    x_goal_ee(1, 3) = this->search_r_dot * t * sin(this->search_omega * t);

    // Transform the goal from the 'tool0' frame to the 'base_link' frame.
    x_goal_base = this->search_center * x_goal_ee;

    // Convert the end effector pose to the corresponding joint vector.
    if(Ur5eBaseClass::toJointVector(x_goal_base, this->joint_q_goal))
    {
      for(j = 0; j < NUM_ROBOT_JOINTS; ++j)
      {
        jtp_msg.positions[j] = this->joint_q_goal(j);
      }

      jtp_msg.time_from_start = ros::Duration(
        t - (this->search_iteration * this->path_planning_total_t));
      this->follow_joint_traj_goal_msg.trajectory.points.push_back(jtp_msg);
    }
    else
    {
      ROS_ERROR_STREAM(
        "Could not obtain joint vector corresponding to transformation:"
        << std::endl
        << eigenIsometry3dToString(x_goal_base));
      path_planned = false;

      break;
    }
  }

  this->follow_joint_traj_goal_msg.trajectory.header.seq++;
  this->follow_joint_traj_goal_msg.trajectory.header.stamp = ros::Time::now();
  this->search_iteration++;

  return path_planned;
}
