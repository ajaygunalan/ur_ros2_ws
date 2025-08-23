#include "pulse_estimation_filters/consistency_check_class.hpp"

/*
 * \brief Construct an object of the type `ConsistencyCheck`
 */
pulse_est::ConsistencyCheck::ConsistencyCheck(ros::NodeHandle& nh) : EstimationFilter(nh), buff_length(5), buff_id(0)
{
  std::string detection_topic_name;

  this->pt_src_pos_base_buff = Eigen::MatrixXd::Zero(3, this->buff_length);
  this->pt_src_found_buff.resize(this->buff_length, false);

  if(this->nh.hasParam("pt_src_mean_dist_thresh"))
  {
    this->nh.getParam("pt_src_mean_dist_thresh", this->pt_src_mean_dist_thresh);
  }
  else
  {
    ROS_WARN(
      "Could not fetch distance threshold between current and mean point source locations, using default value...");
    this->pt_src_mean_dist_thresh = 1e-2;
  }

  if(this->nh.hasParam("detection_topic_name"))
  {
    this->nh.getParam("detection_topic_name", detection_topic_name);
  }
  else
  {
    ROS_WARN("Could not fetch name of topic containing point source detections, using default value...");
    detection_topic_name = "/pt_src_loc_sys_2d/detections";
  }

  // Initialize publisher of estimated target position.
  this->tgt_pos_pub = this->nh.advertise<pulse_vs_msgs::KalmanArray>("target_pose", 1);

  // Initialize ROS publishers and subscribers.
  this->detection_sub =
    this->nh.subscribe(detection_topic_name, 1, &ConsistencyCheck::detectionSubscriberCallback, this);
}

/*
 * \brief Callback function for subscriber to network outputs
 */
void pulse_est::ConsistencyCheck::detectionSubscriberCallback(vision_msgs::Detection2DArray msg)
{
  bool pt_src_found = true;
  double max_conf_score = 0.0;
  Eigen::Isometry3d base_pose_probe_eig;
  Eigen::Isometry3d probe_pose_base_eig;
  Eigen::MatrixXd pt_src_pos_base_vec = Eigen::MatrixXd::Zero(4, 1);
  Eigen::MatrixXd pt_src_pos_probe_vec = Eigen::MatrixXd::Zero(4, 1);
  Eigen::Vector3d pt_src_pos_base_mean = Eigen::MatrixXd::Zero(3, 1);
  pulse_vs_msgs::Kalman pt_src_pos_base_msg;
  geometry_msgs::TransformStamped base_pose_probe_msg;
  geometry_msgs::TransformStamped probe_pose_base_msg;
  int i0;
  int max_conf_score_src_id = -1;
  pulse_vs_msgs::KalmanArray tgt_pos_msg;

  try
  {
    // Fetch and process the transforms between the base and probe frames.
    base_pose_probe_msg = this->tf_buffer.lookupTransform(this->probe_frame_name, this->base_frame_name, ros::Time(0));
    tf::transformMsgToEigen(base_pose_probe_msg.transform, base_pose_probe_eig);
    probe_pose_base_msg = this->tf_buffer.lookupTransform(this->base_frame_name, this->probe_frame_name, ros::Time(0));
    tf::transformMsgToEigen(probe_pose_base_msg.transform, probe_pose_base_eig);

    // Fetch the detection with the highest confidence score among those corresponding to the source class.
    for(i0 = 0; i0 < msg.detections.size(); i0++)
    {
      if(msg.detections[i0].results[0].id == this->src_class_id)
      {
        if((max_conf_score_src_id < 0) || (msg.detections[i0].results[0].score > max_conf_score))
        {
          max_conf_score_src_id = i0;
          max_conf_score = msg.detections[i0].results[0].score;
        }
        else
        {
          // No operation
        }
      }
    }

    if(max_conf_score_src_id >= 0)
    {
      // A detection was found, so convert it to the base frame and store in the buffer.
      ROS_DEBUG("Using detection %d with confidence score %.2f...", max_conf_score_src_id, max_conf_score);
      pt_src_pos_probe_vec(0) = msg.detections[max_conf_score_src_id].results[0].pose.pose.position.x;
      pt_src_pos_probe_vec(1) = msg.detections[max_conf_score_src_id].results[0].pose.pose.position.y;
      pt_src_pos_probe_vec(2) = msg.detections[max_conf_score_src_id].results[0].pose.pose.position.z;
      pt_src_pos_probe_vec(3) = 1.0;
      pt_src_pos_base_vec = probe_pose_base_eig.matrix() * pt_src_pos_probe_vec;
      this->pt_src_pos_base_buff.block<3, 1>(0, this->buff_id) = pt_src_pos_base_vec.block<3, 1>(0, 0);
      this->pt_src_found_buff[this->buff_id] = true;
    }
    else
    {
      ROS_WARN("No detection found corresponding to source class.");
      this->pt_src_found_buff[this->buff_id] = false;
      pt_src_found = false;
    }

    if(pt_src_found)
    {
      ROS_DEBUG("Scanning buffer to ensure that all elements correspond to detected sources...");

      for(i0 = 0; i0 < this->buff_length; i0++)
      {
        pt_src_found &= this->pt_src_found_buff[i0];
      }

      ROS_DEBUG("Buffer scan output: %s", pt_src_found ? "true" : "false");
    }
    else
    {
      ROS_INFO("Skipping buffer scan...");
    }

    if(pt_src_found)
    {
      ROS_DEBUG("Computing buffer mean...");

      // Compute the mean point source location across the buffer.
      for(i0 = 0; i0 < this->buff_length; i0++)
      {
        pt_src_pos_base_mean += this->pt_src_pos_base_buff.block<3, 1>(0, i0);
      }

      pt_src_pos_base_mean /= this->buff_length;

      // Subtract the current point source location from the computed mean.
      pt_src_pos_base_mean -= this->pt_src_pos_base_buff.block<3, 1>(0, this->buff_id);

      // Compare the norm of this difference with the threshold to determine validity.
      pt_src_found &= (pt_src_pos_base_mean.norm() <= this->pt_src_mean_dist_thresh);

      ROS_DEBUG("Buffer mean distance comparison: %s", pt_src_found ? "true" : "false");
    }
    else
    {
      ROS_INFO("Skipping buffer mean distance comparison...");
    }

    tgt_pos_msg.header.stamp = ros::Time::now();

    if(pt_src_found)
    {
      tgt_pos_msg.header.frame_id = this->base_frame_name;
    }
    else
    {
      tgt_pos_msg.header.frame_id = "invalid";
    }

    pt_src_pos_base_msg.kalman_state[0] = this->pt_src_pos_base_buff(0, this->buff_id);
    pt_src_pos_base_msg.kalman_state[1] = this->pt_src_pos_base_buff(1, this->buff_id);
    pt_src_pos_base_msg.kalman_state[2] = this->pt_src_pos_base_buff(2, this->buff_id);
    tgt_pos_msg.filters.push_back(pt_src_pos_base_msg);

    ROS_DEBUG("Publishing estimated target pose...");
    this->tgt_pos_pub.publish(tgt_pos_msg);

    this->buff_id++;
    this->buff_id %= this->buff_length;
  }
  catch(tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
  }
}
