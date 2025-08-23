/**
 *
 * \file estimation_filter_class.cpp
 *
 * \brief This file contains the estimation filter base class to be derived for different applications.
 *
 * \author Mardava Gubbi <mgubbi1@jhu.edu>
 *
 */
#include "pulse_estimation_filters/estimation_filter_class.hpp"

/*
 * \brief Construct an object of the type EstimationFilter
 */
pulse_est::EstimationFilter::EstimationFilter(ros::NodeHandle& nh) : nh(nh)
{
  // Fetch frame names corresponding to robot base and ultrasound transducer.
  if(this->nh.hasParam("base_frame_name"))
  {
    this->nh.getParam("base_frame_name", this->base_frame_name);
    ROS_DEBUG_STREAM("Set 'base_frame_name' to value '" << this->base_frame_name << "'.");
  }
  else
  {
    ROS_WARN("Could not find ROS parameter 'base_frame_name', using default value...");
    this->base_frame_name = "base_link";
  }

  if(this->nh.hasParam("probe_frame_name"))
  {
    this->nh.getParam("probe_frame_name", this->probe_frame_name);
    ROS_DEBUG_STREAM("Set 'probe_frame_name' to value '" << this->probe_frame_name << "'.");
  }
  else
  {
    ROS_WARN("Could not find ROS parameter 'probe_frame_name', using default value...");
    this->probe_frame_name = "probe";
  }

  // Fetch the index of the class corresponding to photoacoustic point sources.
  if(this->nh.hasParam("src_class_id"))
  {
    this->nh.getParam("src_class_id", this->src_class_id);
  }
  else
  {
    ROS_WARN("Could not fetch source class index, using default value...");
    this->src_class_id = 0;
  }

  // Initialize the TF variables required to transform between the probe and base frames.
  // NOTE: This HAS to be performed before using the buffer to look up transforms in the derived classes!
  this->tf_listener = new tf2_ros::TransformListener(this->tf_buffer);
}

/*
 * \brief Extract the source positions in the base frame from the input message.
 */
void pulse_est::EstimationFilter::extractSourcePositions(
  vision_msgs::Detection2DArray msg, Eigen::Isometry3d probe_pose_base_eig, Eigen::MatrixXd& det_pos_base_mat,
  std::vector<int>& src_id_vec, int& num_src_det)
{
  // Positions of source detections and negative elevation counterparts in probe frame.
  Eigen::MatrixXd det_pos_probe_mat;

  int i0;
  int det_pos_probe_row_id = 0;

  // Determine if any of the detections correspond to the source class. This determines whether or not the weights
  // need to be updated.
  for(i0 = 0; i0 < msg.detections.size(); i0++)
  {
    if((msg.detections[i0].results.size() == 1) && (msg.detections[i0].results[0].id == this->src_class_id))
    {
      src_id_vec.push_back(i0);
    }
    else
    {
      // No operation
    }
  }

  // Normalize the score vector for ease of use.
  num_src_det = 2 * src_id_vec.size();

  if(num_src_det > 0)
  {
    // Accommodate each source counted above as well as the negative elevation counterpart.
    det_pos_probe_mat = Eigen::MatrixXd::Zero(4, num_src_det);

    // Fetch the position of each source and construct the position of the corresponding negative elevation counterpart.
    for(i0 = 0; i0 < src_id_vec.size(); i0++)
    {
      det_pos_probe_mat(0, det_pos_probe_row_id) = msg.detections[src_id_vec[i0]].results[0].pose.pose.position.x;
      det_pos_probe_mat(1, det_pos_probe_row_id) = msg.detections[src_id_vec[i0]].results[0].pose.pose.position.y;
      det_pos_probe_mat(2, det_pos_probe_row_id) = msg.detections[src_id_vec[i0]].results[0].pose.pose.position.z;
      det_pos_probe_mat(3, det_pos_probe_row_id) = 1.0;
      det_pos_probe_row_id++;

      if(abs(msg.detections[src_id_vec[i0]].results[0].pose.pose.position.y) > 1e-4)
      {
        det_pos_probe_mat.block<4, 1>(0, det_pos_probe_row_id) =
          det_pos_probe_mat.block<4, 1>(0, det_pos_probe_row_id - 1);
        det_pos_probe_mat(1, det_pos_probe_row_id) = -det_pos_probe_mat(1, det_pos_probe_row_id);
        det_pos_probe_row_id++;
      }
      else
      {
        // No operation
      }
    }

    // Convert the positions of the network detections from the probe frame to the base frame.
    num_src_det = det_pos_probe_row_id;
    det_pos_base_mat = probe_pose_base_eig.matrix() * det_pos_probe_mat.block(0, 0, 4, num_src_det);
  }
  else
  {
    // No operation
  }
}
