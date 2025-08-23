/**
 *
 * \file multitrack_lkf_position_class.hpp
 *
 * \brief Multi-hypothesis position-based Linear Kalman Filter for photoacoustic visual servoing system
 *
 * \author Mardava Gubbi <mgubbi1@jhu.edu>
 *
 * \references
 *
 */
#ifndef PULSE_MULTITRACK_LKF_POSITION_CLASS_HPP
#define PULSE_MULTITRACK_LKF_POSITION_CLASS_HPP

#include <cmath>
#include <random>
#include <vector>

#include "pulse_estimation_filters/estimation_filter_class.hpp"
#include "pulse_estimation_filters/lkf_position_class.hpp"
#include "pulse_vs_msgs/Kalman.h"
#include "pulse_vs_msgs/KalmanArray.h"

namespace pulse_est
{
class MultiTrackLkfPosition : protected EstimationFilter
{
protected:
  /**
   * \brief Current number of tracks in existence
   */
  int number_of_tracks;

  /**
   * \brief Current tracks
   */
  std::vector<LkfPosition*> lkf_tracks;

  /**
   * \brief Smoothing filter coefficient for individual LKF tracks
   */
  double lkf_smooth_coeff;

public:
  /**
   *
   * \brief Construct an object of the type `MultiTrackLkfPosition`
   *
   * \arg[in] nh The ROS node handler
   *
   */
  MultiTrackLkfPosition(ros::NodeHandle& nh);

  /**
   *
   * \brief Update function for multi-track Kalman filter
   *
   * \details This function searches through the input message for sources, associates each source to a track, and
   * updates each track accordingly.
   *
   * \arg[in] msg Message containing detections output by photoacoustic point source localization system
   *
   */
  void detectionSubscriberCallback(vision_msgs::Detection2DArray msg) override;
};
}  // namespace pulse_est

#endif /* PULSE_MULTITRACK_LKF_POSITION_CLASS_HPP */
