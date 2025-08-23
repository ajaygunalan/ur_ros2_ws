/**
 *
 * \file tool_frame_force_control_class.hpp
 *
 * \brief This file contains the base class for force control methods which
 * involve the definition of a desired tool frame such as Yoshikawa et al. [1]
 *
 * For simplicity, we use the following conventions for frames:
 * `B`: robot base frame
 * `P`: probe frame
 *
 * `t_a_b`: Transformation from frame `B` to frame `A`
 * `rot_a_b`: Rotation from frame `B` to frame `A`
 * `p_a_b`: Vector location of origin of frame `B` in frame `A`
 *
 * All computations are carried out in the probe frame `P`. This allows us to
 * prevent this force control algorithm from affecting the elevation
 * displacement of the probe, thus ensuring that the target being tracked during
 * photoacoustic visual servoing remains within the imaging plane of the probe.
 *
 * References:
 * 1. Yoshikawa, Tsuneo, and Akio Sudou. "Dynamic hybrid position/force control of robot manipulators - online
 * estimation of unknown constraint." IEEE Transactions on Robotics and Automation (1993).
 *
 * \author Mardava Gubbi <mgubbi1@jhu.edu>
 *
 */
#ifndef TOOL_FRAME_FORCE_CONTROL_CLASS_HPP
#define TOOL_FRAME_FORCE_CONTROL_CLASS_HPP

// ROS code header files
#include <ros/ros.h>

// ROS message files
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/WrenchStamped.h>

// Moveit header files
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// TF header files
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "pulse_common_utils/eigen_math_utils.hpp"
#include "pulse_common_utils/execution_time.hpp"
#include "pulse_control_utils/ur5e_move_group_if_class.hpp"

namespace pfc
{
/**
 *
 * \brief Class implementing task frame estimation and hybrid position-force
 * control system.
 *
 * \details This class compares the input processed force-torque readings
 * against a desired wrench and generates a tool frame using the rotational x-
 * and y- axes and translational z-axis to which the robot must move to minimize
 * the computed force-torque error. No motion commands are executed in this
 * class. The error is processed through configurable deadbands and saturation
 * filters to account for sensor noise.
 *
 */
class ToolFrameForceControlClass : public mgi::Ur5eMoveGroupInterfaceClass
{
protected:
  /*
   * \brief Threshold for translational z-component of force-torque error
   * determining whether or not the robot end effector is contact with the
   * surface. If the translational z-component of the force-torque error is
   * below the threshold, then the robot is in contact with the surface, and may
   * rotate the end effector to align itself with the local normal. Otherwise,
   * the robot must move along the translational z-dimension of the end effector
   * to establish/reestablish contact with the surface.
   */
  double f_z_deadband;

  /*
   * \brief Time period of control loop.
   */
  double fsm_period;

  /*
   * \brief Maximum angular velocity of end effector along rotational x- and
   * y-dimensions.
   */
  double max_ee_rot_vel;

  /*
   * \brief Maximum translational velocity of end effector along z-dimension.
   */
  double max_ee_trans_vel;

  /*
   * \brief Rotational saturation threshold computed from maximum end effector
   * angular velocity and FSM time period.
   */
  double ee_rot_sat_thresh;

  /*
   * \brief Saturation threshold for z-translation resulting from force control
   * algorithm in each time step.
   */
  double ee_trans_sat_thresh;

  /*
   * \brief Proportional gain for z-translation of desired tool frame `T`
   * relative to current tool frame `P`.
   */
  double z_trans_k_p;

  /*
   * \brief Latest force-torque value
   */
  Eigen::Matrix<double, 6, 1> ft_curr;

  /*
   * \brief Force-torque error
   */
  Eigen::Matrix<double, 6, 1> ft_err;

  /*
   * \brief Commanded force-torque values
   */
  Eigen::Matrix<double, 6, 1> ft_cmd;

  /*
   * \brief Translational component of transform from probe frame `P` to robot
   * base frame `B`.
   */
  Eigen::Vector3d t_curr_b_p;

  /*
   * \brief Force control loop execution timer
   */
  et::PulseExecutionTimer* force_control_timer;

  /**
   * \brief ROS subscriber to the processed force-torque sensor readings in the
   * given frame
   */
  ros::Subscriber ft_proc_tool_sub;

  /**
   * \brief ROS timer to be used to execute control loop
   */
  ros::Timer fsm_timer;

  /*
   * \brief Name of robot base frame `B`
   */
  std::string base_frame_name;

  /*
   * \brief Name of desired tool frame `T` to which the probe must be moved.
   */
  std::string tool_frame_name;

  /*
   * \brief Transform broadcaster for desired tool frame `T`.
   */
  tf::TransformBroadcaster tf_broadcaster;

public:
  /**
   *
   * \brief Construct an object of the type ToolFrameForceControlClass
   *
   * \arg[in] nh NodeHandle object
   * \arg[in] base_frame_name The name of the robot base frame `B`
   * \arg[in] probe_frame_name The name of the probe frame `P`
   * \arg[in] tool_frame_name The name of the tool frame `T`
   *
   * TODO: Come up with a better name for the tool frame `T`.
   *
   */
  ToolFrameForceControlClass(
    ros::NodeHandle& nh, std::string base_frame_name = "base_link", std::string probe_frame_name = "p42v_link1",
    std::string tool_frame_name = "p42v_link1_fc");

  /**
   *
   * \brief Callback function for force control loop
   *
   * \details This function is a pure virtual function to ensure that derived
   * classes differ in their implementations of the force control loop.
   *
   * This function is expected to compute and publish the transform from the
   * desired tool frame `T` to the probe frame `P`.
   *
   * \arg[in] e ROS timer event variable (typically not used in function)
   *
   */
  virtual void forceControlLoopTimerCallback(const ros::TimerEvent& e) = 0;

  /**
   * \brief Callback function for subscriber to processed F/T sensor readings
   *
   * \details This function computes the force torque error
   *
   * \arg[in] msg The most recent processed F/T sensor reading
   */
  virtual void netftProcSensorSubscriberCallback(const geometry_msgs::WrenchStamped& msg);
};
}  // namespace pfc

#endif /* TOOL_FRAME_FORCE_CONTROL_CLASS_HPP */
