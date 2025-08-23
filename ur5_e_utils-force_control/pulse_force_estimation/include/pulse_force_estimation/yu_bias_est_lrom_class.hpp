/**
 *
 * \file yu_bias_est_base_class.hpp
 *
 * \brief Estimate the gravitational forces and sensor bias as described by Yu et al. [1].
 *
 * \details This class implements the Limited Robot Orientation Method (LROM).
 *
 * References:
 * 1. Yu, Yongqiang, Ran Shi, and Yunjiang Lou. "Bias estimation and gravity compensation for wrist-mounted force/torque
 * sensor." IEEE Sensors Journal 22.18 (2021): 17625-17634.
 *
 * \author Mardava Gubbi <mgubbi1@jhu.edu>
 *
 */
#ifndef YU_BIAS_EST_LROM_CLASS_HPP
#define YU_BIAS_EST_LROM_CLASS_HPP

#include "eigen_conversions/eigen_msg.h"
#include "geometry_msgs/WrenchStamped.h"
#include "tf_conversions/tf_eigen.h"

#include "pulse_force_estimation/yu_bias_est_base_class.hpp"
#include "pulse_force_estimation/YuSetBiases.h"

#define LROM_NUM_FT_ROBOT_POSES 32

/**
 *
 * \brief Extend the basic gravitational force and sensor bias estimation class to estimate the parameters in question
 * using the Limited Robot Orientation Method.
 *
 * \details A fixed number of orientations of the robot are generated to ensure that the axes of the robot end effector
 * are parallel to the axes of the robot base frame. These orientations are then used to estimate the gravitational
 * force of the tool, which should remain the same across orientations.
 *
 */
class YuBiasEstLromClass : public ft::YuBiasEstBaseClass
{
protected:
  /*
   * \brief Vertical distance to lift probe above surface for force estimation-related motion planning.
   */
  double probe_lift_displacement;

  /*
   * \brief Estimate the gravitational forces in the robot base frame as described in Section III-A2 of Yu et al.
   */
  void estimateGravitationalForcesRobotBaseFrame(void) override;

  /*
   * \brief Compute the joint angles to be achieved by the robot during the bias estimation procedure.
   */
  void setRobotCmdJointAngles(void) override;

public:
  /**
   *
   * \brief Constructor for YuBiasEstLromClass
   *
   * \arg[in] nh NodeHandle object
   * \arg[in] base_frame_name The name of the robot base frame `B`
   * \arg[in] ee_frame_name The name of the robot end effector frame `E`
   * \arg[in] sensor_frame_name The name of the sensor frame `S`
   *
   */
  YuBiasEstLromClass(
    ros::NodeHandle& nh, std::string base_frame_name = "base_link", std::string ee_frame_name = "tool0",
    std::string sensor_frame_name = "netft_link1");
};

#endif /* YU_BIAS_EST_LROM_CLASS_HPP */
