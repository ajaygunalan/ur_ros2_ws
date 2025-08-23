/**
 *
 * \file bbox_fcvs_class.cpp
 *
 * \brief Visual servoing system using bounding box inputs and force control
 *
 * \author Mardava Gubbi <mgubbi1@jhu.edu>
 */
#include "gubbi_icra_2021/bbox_fcvs_class.hpp"

/*
 * \brief Construct an object of the type BoundingBoxForceControlVisualServoingClass.
 */
vs::BoundingBoxForceControlVisualServoingClass::
  BoundingBoxForceControlVisualServoingClass(
    ros::NodeHandle& nh, std::string probe_frame_name,
    std::string tool_frame_name)
  : mgi::Ur5eMoveGroupInterfaceClass(nh, probe_frame_name)
  , tool_frame_name(tool_frame_name)
  , temporal_validity_thresh(2.0 * LASER_PULSE_REP_TIME)
{
  std::string img_dim_mm_param_name;
  std::string pixel_dim_mm_param_name;
  std::string pt_src_det_topic_name;
  std::string pt_src_topic_ns;
  std::string src_conf_score_thresh_param_name;

  if(!this->nh.getParam("point_source_topic_namespace", pt_src_topic_ns))
  {
    // Use `/d2_faster_rcnn` as the default namespace for image parameters
    // and bounding box inputs.
    pt_src_topic_ns = "/d2_faster_rcnn";
    ROS_WARN_STREAM(
      "Could not fetch parameter 'point_source_topic_namespace', using default "
      "value of '"
      << pt_src_topic_ns << "'...");
  }

  // Get the image dimensions and resolution in the axial and lateral dimensions
  // to convert to pixels in the bounding box inputs to vectors in the physical
  // space.
  //
  // The image resolution values are required to scale from pixels to
  // meters.
  pixel_dim_mm_param_name = pt_src_topic_ns + "/pixel_dim_mm";

  if(!this->nh.getParam(pixel_dim_mm_param_name, this->pixel_dim_m))
  {
    ROS_WARN_STREAM(
      ""
      << "Could not fetch parameter '" << pixel_dim_mm_param_name
      << "', using default values...");
    this->pixel_dim_m.push_back(120e-3 / 256.0);
    this->pixel_dim_m.push_back(sqrt(2.0) * 120e-3 / 256.0);
  }
  else
  {
    ROS_DEBUG("Scaling image resolution to meters...");
    this->pixel_dim_m[0] *= 1e-3;
    this->pixel_dim_m[1] *= 1e-3;
  }

  ROS_DEBUG(
    "Image resolution: [%.2f, %.2f] um", this->pixel_dim_m[0] * 1e6,
    this->pixel_dim_m[1] * 1e6);

  // The image dimensions in meters are required to offset the lateral source
  // position after scaling.
  img_dim_mm_param_name = pt_src_topic_ns + "/img_dim_mm";

  if(!this->nh.getParam(img_dim_mm_param_name, this->img_dim_m))
  {
    ROS_WARN_STREAM(
      ""
      << "Could not fetch parameter '" << img_dim_mm_param_name
      << "', using default values...");
    this->img_dim_m.push_back(120e-3);
    this->img_dim_m.push_back(sqrt(2.0) * 120e-3);
  }
  else
  {
    ROS_DEBUG("Scaling image resolution to meters...");
    this->img_dim_m[0] *= 1e-3;
    this->img_dim_m[1] *= 1e-3;
  }

  ROS_DEBUG(
    "Image dimensions: [%.2f, %.2f] mm", 1e3 * this->img_dim_m[0],
    1e3 * this->img_dim_m[1]);

  // The confidence score threshold for the 'source' category allows us to
  // discard sources with insufficient confidence scores.
  src_conf_score_thresh_param_name = pt_src_topic_ns + "/src_conf_score_thresh";

  if(!this->nh.getParam(
       src_conf_score_thresh_param_name, this->src_conf_score_thresh))
  {
    ROS_WARN_STREAM(
      ""
      << "Could not fetch parameter '" << src_conf_score_thresh_param_name
      << "', using default value of 0.0...");
    this->src_conf_score_thresh = 0.0;
  }

  // Start the robot in the stationary mode, waiting for the first detection to
  // transition to tracking.
  this->fsm_state = vs::STATIONARY;

  if(!this->nh.getParam("fsm_period", this->fsm_period))
  {
    this->fsm_period = 0.05;
    ROS_WARN("Could not fetch parameter 'fsm_period', using default value...");
  }

  ROS_DEBUG("FSM period: %.2f ms", this->fsm_period * 1e3);

  // Ensure that you are always dealing with the most recent output from the
  // point source localization system by setting the buffer length to unity.
  pt_src_det_topic_name = pt_src_topic_ns + "/detections";
  this->pt_src_det_sub = this->nh.subscribe(
    pt_src_det_topic_name, 1,
    &vs::BoundingBoxForceControlVisualServoingClass::pointSourceBboxCallback,
    this);

  // Initialize and trigger the FSM timer.
  this->fsm_timer = this->nh.createTimer(
    ros::Duration(fsm_period),
    &vs::BoundingBoxForceControlVisualServoingClass::fsmTimerCallback, this);
}

/*
 * \brief Callback function for point source bounding box messages.
 */
void vs::BoundingBoxForceControlVisualServoingClass::pointSourceBboxCallback(
  const vision_msgs::Detection2DArray& msg)
{
  int i0;
  int max_src_prob_id = -1;

  for(i0 = 0; i0 < msg.detections.size(); i0++)
  {
    // vision_msgs::Detection2D messages allow for multiple class hypotheses per
    // object, a feature we do not use. Our point source localization systems
    // filter their predictions down to a single class for each detection.
    if(msg.detections[i0].results.size() == 1)
    {
      // Check to see if the object hypothesis for the current detection is
      // 'source', with ID 0.
      // NOTE: In general we do not expect more than
      if(
          (msg.detections[i0].results[0].id == 0)
          && ((max_src_prob_id < 0)
            || (msg.detections[i0].results[0].score
              > msg.detections[max_src_prob_id].results[0].score)))
      {
        max_src_prob_id = i0;
      }
    }
    else
    {
      ROS_ERROR(
        "Expected one object hypothesis per detection, received %ld for "
        "detection %d.",
        msg.detections[i0].results.size(), i0);
    }
  }

  if(max_src_prob_id >= 0)
  {
    if(msg.detections[max_src_prob_id].results[0].score >= this->src_conf_score_thresh)
    {
      // Convert bounding box to source position vector in probe frame `P`.
      Eigen::Vector3d src_pos_p;
      src_pos_p.setZero();
      src_pos_p(0) =
        (msg.detections[max_src_prob_id].bbox.center.x * this->pixel_dim_m[1])
        - (this->img_dim_m[1] / 2.0);
      src_pos_p(2) =
        msg.detections[max_src_prob_id].bbox.center.y * this->pixel_dim_m[0];

      // Transform vector from probe frame `P` to robot base frame `B`.
      geometry_msgs::PoseStamped geo_msg_b_p = this->mgi_obj->getCurrentPose();
      Eigen::Isometry3d eig_b_p;
      tf::poseMsgToEigen(geo_msg_b_p.pose, eig_b_p);

      Eigen::Vector3d src_pos_b = eig_b_p * src_pos_p;
      bool spatio_temporally_valid;
      ros::Duration delta_t_valid =
        msg.header.stamp - this->valid_src_time_curr;

      if(delta_t_valid.toSec() < this->temporal_validity_thresh)
      {
        Eigen::Vector3d delta_src_b = src_pos_b - this->eig_valid_src_b_curr;

        // NOTE: This spatio-temporal consistency check is different from the
        // previous one as we no longer need a buffer of 5 frames for the
        // computation.
        if(delta_src_b.norm() / delta_t_valid.toSec() <= MAX_SRC_SPEED_THRESH)
        {
          ROS_DEBUG("Spatio-temporal consistency check passed.");
          spatio_temporally_valid = true;
        }
        else
        {
          ROS_WARN(
            "Speed of detected source between previous and current frames "
            "exceeds threshold, marking latest detection as invalid...");
          spatio_temporally_valid = false;
        }
      }
      else
      {
        // Skip the spatio-temporal consistency check if the last valid source
        // position was received more than two frames prior to the current time.
        ROS_WARN(
          "Last valid source received %.2f ms ago (> %.2f ms). Skipping "
          "consistency check...",
          delta_t_valid.toSec() * 1e3, this->temporal_validity_thresh * 1e3);
        spatio_temporally_valid = true;
      }

      if(spatio_temporally_valid)
      {
        ROS_DEBUG("Resetting counter of messages without valid sources...");
        this->no_valid_src_counter = 0;

        // Update the previous position and time-stamp of valid source
        // detections, making room to compute the current values.
        this->eig_valid_src_b_curr = src_pos_b;
        this->valid_src_time_curr = msg.header.stamp;
      }
      else
      {
        ROS_DEBUG("Incrementing counter of messages without valid sources...");
        this->no_valid_src_counter++;
      }
    }
    else
    {
      ROS_WARN("Detected source with insufficient confidence score. "
               "Incrementing counter of messages without valid sources...");
      this->no_valid_src_counter++;
    }
  }
  else
  {
    // No detections were classified as sources, but detections were received.
    ROS_WARN("No sources detected. Incrementing counter of messages without "
             "valid sources...");
    this->no_valid_src_counter++;
  }
}

/*
 * \brief Callback function for FSM timer event.
 */
void vs::BoundingBoxForceControlVisualServoingClass::fsmTimerCallback(
  const ros::TimerEvent& e)
{
  // Check the counter of the number of messages without valid source detections
  // to determine what to do.
  if(this->no_valid_src_counter >= 10)
  {
    // TODO: Force control only.
    ROS_ERROR("No source found, holding stationary...");
  }
  else if(this->no_valid_src_counter > 0)
  {
    // TODO: Spiral search with force control.
    ROS_WARN("Searching for source...");
  }
  else
  {
    ros::Duration last_valid_src_dur =
      ros::Time::now() - this->valid_src_time_curr;

    // A valid source position may only be used for at most twice the laser pulse repetition time.
    if(last_valid_src_dur.toSec() < this->temporal_validity_thresh)
    {
      // TODO: Tracking with force control.
      ROS_DEBUG("Moving toward source...");
    }
    else
    {
      // TODO: Force control only.
      ROS_WARN("Last source position too old, holding stationary...");
    }
  }
}
