/**
 *
 * \file estimation_filter_class.hpp
 *
 * \brief This file contains the estimation filter base class to be derived for different applications.
 *
 * \author Mardava Gubbi <mgubbi1@jhu.edu>
 *
 */
#ifndef PULSE_ESTIMATION_FILTER_CLASS_HPP
#define PULSE_ESTIMATION_FILTER_CLASS_HPP

#include <string>

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <vision_msgs/Detection2DArray.h>

#include "pulse_vs_msgs/Kalman.h"
#include "pulse_vs_msgs/KalmanArray.h"

namespace pulse_est
{
class EstimationFilter
{
protected:
  /**
   * \brief Index of source class
   */
  int src_class_id;

  /**
   * \brief ROS node handler
   */
  ros::NodeHandle nh;

  /**
   * \brief Publisher of current estimated target position (and validity)
   */
  ros::Publisher tgt_pos_pub;

  /**
   * \brief Subscriber to topic containing network detections
   */
  ros::Subscriber detection_sub;

  /**
   * \brief Time at which previous network detections were received
   */
  ros::Time t_det_prev;

  /**
   * \brief Name of TF frame corresponding to robot base
   */
  std::string base_frame_name;

  /**
   * \brief Name of TF frame corresponding to ultrasound transducer
   */
  std::string probe_frame_name;

  /**
   * \brief TF related variables to propagate particles
   */
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener* tf_listener;

  /**
   *
   * \brief Extract the source positions in the base frame from the input message.
   *
   * \arg[in] msg The message containing positive elevation source and artifact detections in the probe frame
   * \arg[in] probe_pose_base_eig Eigen::Isometry3d matrix containing homogeneous transform from probe to base frame
   * \arg[out] det_pos_base_mat Matrix to be populated with possible source positions in base frame
   * \arg[out] src_id_vec Vector containing indices of detections in input message corresponding to source class
   * \arg[out] num_src_det Number of detections and negative elevation counterparts in input message corresponding to
   * source class
   *
   * \returns A matrix of three-dimensional source position possibilities in homogeneous form (i.e., appended with
   * unity)
   *
   */
  void extractSourcePositions(
    vision_msgs::Detection2DArray msg, Eigen::Isometry3d probe_pose_base_eig, Eigen::MatrixXd& det_pos_base_mat,
    std::vector<int>& src_id_vec, int& num_src_det);

public:
  /**
   *
   * \brief Construct an object of the type `EstimationFilter`
   *
   * \arg[in] nh The ROS node handler
   * \arg[in] probe_frame_name The name of the frame corresponding to the ultrasound transducer
   *
   */
  EstimationFilter(ros::NodeHandle& nh);

  /**
   *
   * \brief Callback function for subscriber to network outputs
   *
   * \details This function is meant to be overridden by derived classes, containing the implementations of specific
   * filters.
   *
   * \arg[in] msg Message containing detections output by photoacoustic point source localization system
   *
   */
  virtual void detectionSubscriberCallback(vision_msgs::Detection2DArray msg) = 0;
};
}  // namespace pulse_est

#endif /* PULSE_ESTIMATION_FILTER_CLASS_HPP */
