/**
 * \file ur5e_pos_control_class.hpp
 *
 * \brief Position control built on top of UR5e base class.
 *
 * \author Mardava Gubbi <mgubbi1@jhu.edu>
 */
#ifndef PULSE_UR5E_POS_CONTROL_CLASS_HPP
#define PULSE_UR5E_POS_CONTROL_CLASS_HPP

#include "pulse_common_utils/eigen_string_utils.hpp"
#include "pulse_control_utils/ur5e_base_class.hpp"

class Ur5ePosControlClass : public Ur5eBaseClass
{
protected:
  /*
   * \brief Maximum end effector speed commandable
   */
  double ee_vel_max;

  /*
   * \brief Proportional gain component of PID-based position controller
   */
  double k_p;

  /*
   * \brief Integral gain component of PID-based position controller
   */
  double k_i;

  /*
   * \brief Derivative gain component of PID-based position controller
   */
  double k_d;

  /*
   * \brief Maximum tolerable translational error after PID control termination
   */
  double t_err_max;

  /*
   * \brief Maximum number of separate motion commands to be sent to robot
   */
  int n_steps_max;

public:
  /**
   * \brief Constructor for Ur5ePosControlClass
   *
   * \arg[in] nh NodeHandle object
   * \arg[in] action_server_name Name of action server for robot motion commands
   * \arg[in] ee_name Name of frame corresponding to robot end effector
   * \arg[in] delta_t Time step of individual robot commands
   * \arg[in] ee_vel_max Maximum end effector velocity along any axis
   * \arg[in] k_p Proportional gain of PID controller
   * \arg[in] k_i Integral gain of PID controller
   * \arg[in] k_d Derivative gain of PID controller
   * \arg[in] joint_omega_max Maximum joint angular velocity to be commanded
   * \arg[in] joint_alpha_max Maximum joint angular acceleration to be commanded
   * \arg[in] t_err_max Maximum tolerated error between desired and achieved
   * forward kinematic transformations
   * \arg[in] n_steps_max Maximum number of steps to use for motion planning
   * \arg[in] robot_ns Namespace corresponding to robot related information
   */
  Ur5ePosControlClass(
    ros::NodeHandle& nh, std::string action_server_name, std::string ee_name,
    double delta_t, double ee_vel_max = 0.1, double k_p = 1.0, double k_i = 0.0,
    double k_d = 0.0, double joint_omega_max = 0.1,
    double joint_alpha_max = M_PI / 2.0, double t_err_max = 1e-4,
    int n_steps_max = 100, std::string robot_ns = "/robot");

  /**
   * \brief Move the robot to the desired Cartesian pose in the spatial frame.
   *
   * \arg[in] t The pose in the spatial frame to which the robot is to be moved
   */
  bool setSpatialCartesianPose(Eigen::Isometry3d t);
};

#endif /* PULSE_UR5E_POS_CONTROL_CLASS_HPP */
