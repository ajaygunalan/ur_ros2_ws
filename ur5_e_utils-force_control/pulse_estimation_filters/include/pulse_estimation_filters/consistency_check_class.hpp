/**
 *
 * \file consistency_check_class.hpp
 *
 * \brief This file contains the consistency check class, to estimate the location of a point source relative to an
 * ultrasound transducer.
 *
 * \author Mardava Gubbi <mgubbi1@jhu.edu>
 *
 * \references
 * 1. Gubbi and Bell, IEEE ICRA 2021
 *
 */
#ifndef PULSE_CONSISTENCY_CHECK_HPP
#define PULSE_CONSISTENCY_CHECK_HPP

#include <cmath>
#include <random>
#include <vector>

#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <vision_msgs/Detection2DArray.h>

#include "pulse_estimation_filters/estimation_filter_class.hpp"

namespace pulse_est
{
class ConsistencyCheck : protected EstimationFilter
{
protected:
  /**
   * \brief Threshold of distance between current point source location and mean point source location to mark the
   * current position as valid
   */
  double pt_src_mean_dist_thresh;

  /**
   * \brief Buffer of recent point source locations required to perform consistency check
   */
  Eigen::MatrixXd pt_src_pos_base_buff;

  /**
   * \brief Length of point source location buffer
   */
  int buff_length;

  /**
   * \brief Index of current point source location in buffer
   */
  int buff_id;

  /**
   * \brief Buffer of flags indicating validity of corresponding point source locations in `pt_src_pos_base_buff`
   */
  std::vector<bool> pt_src_found_buff;

public:
  /**
   *
   * \brief Construct an object of the type `ConsistencyCheck`
   *
   * \arg[in] nh The ROS node handler
   *
   */
  ConsistencyCheck(ros::NodeHandle& nh);

  /**
   *
   * \brief Callback function for subscriber to network outputs
   *
   * \details This function receives the outputs from the network, extracts the source (if any) with the highest
   * confidence, and performs the consistency check as described in [1].
   *
   * \arg[in] msg Message containing detections output by photoacoustic point source localization system
   *
   */
  void detectionSubscriberCallback(vision_msgs::Detection2DArray msg) override;
};
}  // namespace pulse_est

#endif /* PULSE_CONSISTENCY_CHECK_HPP */
