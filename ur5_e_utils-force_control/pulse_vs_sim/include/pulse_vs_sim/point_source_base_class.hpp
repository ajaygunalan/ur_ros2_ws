/**
 *
 * \file point_source_base_class.hpp
 *
 * \brief Simulate a point source which is held stationary in the base frame.
 *
 * \author Mardava Gubbi <mgubbi1@jhu.edu>
 *
 */
#ifndef PULSE_POINT_SOURCE_BASE_CLASS_HPP
#define PULSE_POINT_SOURCE_BASE_CLASS_HPP

#include <cmath>
#include <string>
#include <random>

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>

namespace vs_sim
{
/**
 *
 * \brief Move a point source in independent sinusoidal patterns in x-, y-, and z-dimensions
 *
 */
class PointSourceBase
{
protected:
  /**
   * \brief Time period of timer for point source movement control system execution [s]
   */
  double ctrl_delta_t;

  /**
   * \brief Iteration index of control loop
   */
  double ctrl_iter_id;

  /**
   * \brief Transform from base frame to point source frame
   */
  Eigen::Isometry3d pt_src_pose_base_eig;

  /**
   * \brief ROS node handler
   */
  ros::NodeHandle nh;

  /**
   * \brief ROS timer to execute control system for point source movement.
   */
  ros::Timer ctrl_timer;

  /**
   * \brief Names of base and point source frames
   */
  std::string base_frame_name;
  std::string pt_src_frame_name;
  std::string probe_frame_name;

  /**
   * \brief TF broadcaster to publish transformation from point source frame to base frame.
   */
  tf2_ros::TransformBroadcaster tf_broadcaster;

public:
  /**
   *
   * \brief Construct an object of the type `PointSourceBase`
   *
   * \arg[in] nh The ROS node handler
   *
   */
  PointSourceBase(ros::NodeHandle& nh);

  /**
   *
   * \brief Timer callback function to publish transform between point source and base
   *
   * \arg[in] e ROS timer event (unused)
   *
   */
  virtual void ctrlTimerCallback(const ros::TimerEvent& e) = 0;
};
}  // namespace vs_sim

#endif /* PULSE_POINT_SOURCE_BASE_CLASS_HPP */
