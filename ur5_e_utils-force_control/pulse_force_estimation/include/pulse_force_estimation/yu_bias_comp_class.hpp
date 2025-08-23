/**
 *
 * \file yu_bias_comp_class.hpp
 *
 * \brief Perform sensor bias and gravity compensation for the force-torque
 * sensor based on the method described by Yu et al. [1].
 *
 * \details This class implements the sensor bias and gravitational force
 * compensation to compute the contact force (see Sec. II in [1]) in the
 * sensor frame. This should be easily transferrable to another force-torque
 * sensor.
 *
 * NOTE: This class follows the notation of `t_d_s` to indicate a transform from
 * the source frame `S` to the destination frame `D`.
 * NOTE: The tool frame `T` in [1] is equivalent to the probe frame `P` in our
 * visual servoing system. We try to stick to `P` in this code.
 *
 * References:
 * 1. Yu, Yongqiang, Ran Shi, and Yunjiang Lou. "Bias estimation and gravity
 * compensation for wrist-mounted force/torque sensor."
 * IEEE Sensors Journal 22.18 (2021): 17625-17634.
 * 2. Murray, Richard M., et al. "A mathematical introduction to robotic
 * manipulation."
 * CRC press, 1994.
 *
 * \author Mardava Gubbi <mgubbi1@jhu.edu>
 */
#ifndef PULSE_YU_BIAS_COMP_CLASS_HPP
#define PULSE_YU_BIAS_COMP_CLASS_HPP

#include <boost/filesystem.hpp>
#include <Eigen/Geometry>
#include <highfive/highfive.hpp>
#include <stdexcept>

#include <ros/ros.h>

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float64.h>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>

#include "netft_utils/Cancel.h"
#include "pulse_common_utils/eigen_math_utils.hpp"
#include "pulse_common_utils/execution_time.hpp"
#include "pulse_force_estimation/YuSetBiases.h"

class YuBiasCompClass
{
protected:
  /**
   * \brief Instantaneous transformation from robot base frame `B` to robot end
   * effector frame `E`.
   */
  Eigen::Isometry3d t_e_b;

  /**
   * \brief Estimate of the position of the origin of the tool gravity frame `G`
   * in the sensor frame `S` (see Eq. (5) in [1]).
   *
   * \details The origin of the tool gravity frame `G` corresponds to the center
   * of gravity of the tool.
   */
  Eigen::Matrix3d p_grav_s_hat;

  /**
   * \brief Estimate of rotation from robot end effector frame `E` to sensor
   * frame `S` (see Eq. (6) in [1]).
   */
  Eigen::Matrix3d rot_s_e;

  /**
   * \brief Estimate of rotation from tool gravity frame `G` to robot base frame
   * `B` (see Eq. (6), Fig. 2 in [1]).
   *
   * \details The tool gravity frame `G` is defined as having its axes parallel
   * to the world frame and origin coincident with the center of mass of the
   * tool. This frame translates as the robot moves, but does not rotate with
   * respect to the world frame. This matrix estimated the constant rotation
   * between the robot base frame and the tool gravity frame, with non-zero
   * components along the x- and y-directions.
   */
  Eigen::Matrix3d rot_b_g;

  /*
   * \brief Adjoint transformation matrix for converting force-torque wrenches
   * from sensor frame `S` to probe frame `P`.
   */
  Eigen::Matrix<double, 6, 6> ft_adjoint_p_s;

  /**
   * \brief Estimate of gravitational force in robot base frame `B`
   * (see Eq. (9) in [1]).
   *
   * \details This force is constant in the base frame, which does not rotate
   * with respect to the world frame.
   */
  Eigen::Vector3d f_grav_b;

  /**
   * \brief Estimate of force component of sensor bias in sensor frame `S`
   *
   * \details This is referred to as $^{s}F_{0}$ in Eq. (2) in [1].
   */
  Eigen::Vector3d f_bias_s;

  /**
   * \brief Estimate of torque component of sensor bias in sensor frame `S`
   *
   * \details This is referred to as $^{s}T_{0}$ in Eq. (3) in [1].
   */
  Eigen::Vector3d t_bias_s;

  /*
   * \brief Execution time estimator for bias compensation function.
   */
  et::PulseExecutionTimer* bias_comp_exe_timer;

  /**
   * \brief ROS NodeHandle object
   */
  ros::NodeHandle nh;

  /**
   * \brief ROS publisher for force-torque sensor readings converted to probe
   * frame `P`.
   */
  ros::Publisher ft_proc_probe_pub;

  /**
   * \brief ROS publisher for force-torque sensor readings in sensor frame `S`
   * after bias and gravity compensation.
   */
  ros::Publisher ft_proc_sensor_pub;

  /**
   * \brief ROS publisher for force magnitudes from raw sensor readings.
   *
   * \details Used for debugging purposes
   */
  ros::Publisher ft_raw_force_norm_pub;

  /**
   * \brief ROS publisher for torque magnitudes from raw sensor readings.
   *
   * \details Used for debugging purposes
   */
  ros::Publisher ft_raw_torque_norm_pub;

  /**
   * \brief ROS publisher for cancel signal to be sent to robot in case of
   * exceedingly high forces.
   */
  ros::Publisher ft_cancel_pub;

  /**
   * \brief ROS service handler required to update the parameters of the bias
   * compensation process.
   */
  ros::ServiceServer bias_update_server;

  /**
   * \brief ROS subscriber to raw force-torque sensor readings in sensor frame
   * `S`.
   */
  ros::Subscriber ft_raw_sensor_sub;

  /**
   * \brief TF listener object required to interface with TF chain.
   */
  tf::TransformListener* tf_listener;

  /**
   * \brief Name of robot base frame `B`
   */
  std::string base_frame_name;

  /**
   * \brief Name of robot end effector frame `E`
   *
   * \details This frame is physically located on the robot (e.g., `/tool0`),
   * not an attached tool.
   */
  std::string ee_frame_name;

  /**
   * \brief Name of sensor base frame `S`
   *
   * \details This frame should be the frame in which raw sensor readings are
   * received (e.g., `/netft_link1` for the ATI Gamma sensor). Refer to a CAD
   * model of the sensor to determine which this is.
   */
  std::string sensor_frame_name;

  /**
   * \brief Name of probe frame `P`
   *
   * \details This is the frame in which photoacoustic images and related data
   * are published (e.g., `/p42v_link1`). This should ideally be fetched from a
   * `MoveGroupInterface` object as the final frame in the TF chain.
   */
  std::string probe_frame_name;

public:
  /**
   * \brief Constructor for YuBiasCompClass
   *
   * \arg[in] nh NodeHandle object
   * \arg[in] base_frame_name The name of the robot base frame `B`
   * \arg[in] ee_frame_name The name of the robot end effector frame `E`
   * \arg[in] sensor_frame_name The name of the sensor frame `S`
   * \arg[in] probe_frame_name The name of the probe frame `P`
   */
  YuBiasCompClass(
    ros::NodeHandle& nh, std::string base_frame_name = "base_link", std::string ee_frame_name = "tool0",
    std::string sensor_frame_name = "netft_link1", std::string probe_frame_name = "p42v_link1");

  /**
   * \brief Destructor for YuBiasCompClass
   */
  ~YuBiasCompClass(void);

  /**
   * \brief Set or update the bias and gravity compensation parameters
   *
   * \details This function must return a boolean value as it is advertised as a
   * service.
   *
   * \arg[in] req The ROS service request object
   * \arg[out] res The ROS service response object
   *
   * \returns true if successful, false otherwise
   */
  bool setBiases(pulse_force_estimation::YuSetBiases::Request& req, pulse_force_estimation::YuSetBiases::Response& res);

  /**
   * \brief Callback function to run when a new raw data point is received
   *
   * \arg[in] raw_msg The message object containing the raw sensor data
   */
  void rawSensorSubscriberCallback(const geometry_msgs::WrenchStamped& raw_msg);
};

#endif /* PULSE_YU_BIAS_COMP_CLASS_HPP */
