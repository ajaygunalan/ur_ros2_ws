/**
 * \file ur5e_pos_control_class.cpp
 *
 * \brief Position control built on top of UR5e base class.
 *
 * \author Mardava Gubbi <mgubbi1@jhu.edu>
 */
#include "pulse_control_utils/ur5e_pos_control_class.hpp"

/*
 * \brief Constructor for Ur5ePosControlClass
 */
Ur5ePosControlClass::Ur5ePosControlClass(
  ros::NodeHandle& nh, std::string action_server_name, std::string ee_name,
  double delta_t, double ee_vel_max, double k_p, double k_i, double k_d,
  double joint_omega_max, double joint_alpha_max, double t_err_max,
  int n_steps_max, std::string robot_ns)
  : Ur5eBaseClass(
    nh, action_server_name, ee_name, delta_t, joint_omega_max, joint_alpha_max,
    robot_ns)
  , ee_vel_max(ee_vel_max)
  , k_p(k_p)
  , k_i(k_i)
  , k_d(k_d)
  , t_err_max(t_err_max)
  , n_steps_max(n_steps_max)
{
}

/*
 * \brief Move the robot to the desired Cartesian pose in the spatial frame.
 */
bool Ur5ePosControlClass::setSpatialCartesianPose(Eigen::Isometry3d t)
{
  Eigen::Vector3d t_err;
  Eigen::VectorXd v(6);
  int i0, i1;

  ROS_DEBUG_STREAM(
    "Setting spatial cartesian pose:" << std::endl
                                      << eigenIsometry3dToString(t));

  for(i0 = 0; i0 < v.rows(); ++i0)
  {
    v(i0) = 0.0;
  }

  for(i0 = 0; i0 < this->n_steps_max; ++i0)
  {
    const Eigen::Isometry3d fwd_kin_curr = this->getGlobalLinkTransform();
    t_err = t.translation() - fwd_kin_curr.translation();
    ROS_DEBUG(
      "[%3d/%3d] Translational error: %.3f mm", i0, this->n_steps_max,
      t_err.norm() * 1e3);

    if(t_err.norm() >= t_err_max)
    {
      t_err *= this->k_p;

      if(t_err.norm() > this->ee_vel_max)
      {
        t_err *= this->ee_vel_max / t_err.norm();
      }
      else
      {
        // No operation
      }

      for(i1 = 0; i1 < 3; ++i1)
      {
        v(i1) = t_err(i1);
      }

      ROS_DEBUG_STREAM(
        "[" << (i0 + 1) << "/" << this->n_steps_max
            << "] Setting spatial velocity of frame " << this->ee_name << " to "
            << std::endl
            << "[" << v(0) << ", " << v(1) << ", " << v(2) << ", " << v(3)
            << ", " << v(4) << ", " << v(5) << "] <m/rad>/s...");
      this->setSpatialCartesianVelocity(v);
      this->sendGoalAndWait();
    }
    else
    {
      break;
    }
  }

  return (t_err.norm() < this->t_err_max);
}
