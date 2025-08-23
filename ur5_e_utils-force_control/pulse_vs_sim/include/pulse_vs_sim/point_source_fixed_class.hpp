/**
 *
 * \file point_source_fixed_class.hpp
 *
 * \brief Simulate a point source which is held stationary in the base frame.
 *
 * \author Mardava Gubbi <mgubbi1@jhu.edu>
 *
 */
#ifndef PULSE_POINT_SOURCE_FIXED_CLASS_HPP
#define PULSE_POINT_SOURCE_FIXED_CLASS_HPP

#include <cmath>
#include <string>
#include <random>

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>

#include "pulse_vs_sim/point_source_base_class.hpp"

namespace vs_sim
{
/**
 *
 * \brief Move a point source in independent sinusoidal patterns in x-, y-, and z-dimensions
 *
 */
class PointSourceFixed : protected PointSourceBase
{
public:
  /**
   *
   * \brief Construct an object of the type `PointSourceFixed`
   *
   * \arg[in] nh The ROS node handler
   *
   */
  PointSourceFixed(ros::NodeHandle& nh);

  /**
   *
   * \brief Timer callback function to publish transform between point source and base
   *
   * \arg[in] e ROS timer event (unused)
   *
   */
  void ctrlTimerCallback(const ros::TimerEvent& e) override;
};
}  // namespace vs_sim

#endif /* PULSE_POINT_SOURCE_FIXED_CLASS_HPP */
