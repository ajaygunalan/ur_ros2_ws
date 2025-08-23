/**
 *
 * \file fake_invalid_target_pose_pub_class.hpp
 *
 * \brief Publish target pose messages with frame ID "invalid" at a fixed rate.
 *
 * \author Mardava Gubbi <mgubbi1@jhu.edu>
 *
 */
#ifndef PULSE_FAKE_INVALID_TARGET_POSE_PUB_CLASS_HPP
#define PULSE_FAKE_INVALID_TARGET_POSE_PUB_CLASS_HPP

#include <ros/ros.h>

#include "pulse_vs_msgs/Kalman.h"
#include "pulse_vs_msgs/KalmanArray.h"

namespace vs_sim
{
class FakeInvalidTargetPosePublisher
{
protected:
  /**
   * \brief ROS node handler
   */
  ros::NodeHandle nh;

  /**
   * \brief ROS timer to execute fixed rate message publishing
   */
  ros::Timer pub_timer;

  /**
   * \brief Publisher object for the target position topic
   */
  ros::Publisher tgt_pos_pub;

public:
  /**
   *
   * \brief Construct an object of the type `FakeInvalidTargetPosePublisher`.
   *
   * \arg[in] nh The ROS node handler
   *
   */
  FakeInvalidTargetPosePublisher(ros::NodeHandle& nh);

  /**
   *
   * \brief Timer callback function for fixed-rate message publishing
   *
   * \arg[in] ev The timer event (unused)
   *
   */
  void timerCallback(const ros::TimerEvent& ev);
};
}  // namespace vs_sim

#endif /* PULSE_FAKE_INVALID_TARGET_POSE_PUB_CLASS_HPP */
