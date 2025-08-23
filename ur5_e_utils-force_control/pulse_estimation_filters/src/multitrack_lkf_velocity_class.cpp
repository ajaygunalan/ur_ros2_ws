#include "pulse_estimation_filters/multitrack_lkf_velocity_class.hpp"

/*
 * \brief Construct an object of the type `MultiTrackLkfVelocity`
 */
pulse_est::MultiTrackLkfVelocity::MultiTrackLkfVelocity(ros::NodeHandle& nh) : EstimationFilter(nh), number_of_tracks(0)
{
  std::string detection_topic_name;

  if(this->nh.hasParam("detection_topic_name"))
  {
    this->nh.getParam("detection_topic_name", detection_topic_name);
  }
  else
  {
    ROS_WARN("Could not fetch name of topic containing point source detections, using default value...");
    detection_topic_name = "/pt_src_loc_sys_3a/detections";
  }

  // Initialize publisher of estimated target velocity.
  this->tgt_pos_pub = this->nh.advertise<pulse_vs_msgs::KalmanArray>("target_pose", 1);

  // Initialize ROS publishers and subscribers.
  this->detection_sub =
    this->nh.subscribe(detection_topic_name, 1, &MultiTrackLkfVelocity::detectionSubscriberCallback, this);
}

/*
 * \brief Update function for multi-track Kalman filter
 *
 * \details
 */
void pulse_est::MultiTrackLkfVelocity::detectionSubscriberCallback(vision_msgs::Detection2DArray msg)
{
  double src_track_dist_tmp;
  double inf = std::numeric_limits<double>::infinity();

  Eigen::Isometry3d base_pose_probe_eig;
  Eigen::Isometry3d probe_pose_base_eig;

  // Positions of source detections and negative elevation counterparts in base frame.
  Eigen::MatrixXd det_pos_base_mat;

  // Distance of each source detection from each existing track
  Eigen::MatrixXd src_track_dist_mat;

  // Current transforms between the base and probe frame.
  geometry_msgs::TransformStamped base_pose_probe_msg;
  geometry_msgs::TransformStamped probe_pose_base_msg;

  bool track_states_changed = false;

  bool multi_track_valid = false;
  Eigen::Vector3d tracking_track_diff_vec;
  int i0;
  int i1;
  int i2;
  int min_dist_src_id;
  int min_dist_track_id;
  int num_src_det;
  pulse_vs_msgs::Kalman track_msg;
  pulse_vs_msgs::KalmanArray output_msg;
  std::vector<int> src_id_vec;
  std::vector<int> track_to_src_map;
  std::vector<int> tracks_to_delete;
  std::vector<int> track_originators;
  std::vector<int> tracking_track_id_vec;
  double tracking_track_dist_deriv = 0.0;

  try
  {
    // Fetch transforms back and forth between probe and base frames.
    base_pose_probe_msg = this->tf_buffer.lookupTransform(this->probe_frame_name, this->base_frame_name, ros::Time(0));
    base_pose_probe_eig = tf2::transformToEigen(base_pose_probe_msg);

    probe_pose_base_msg = this->tf_buffer.lookupTransform(this->base_frame_name, this->probe_frame_name, ros::Time(0));
    probe_pose_base_eig = tf2::transformToEigen(probe_pose_base_msg);

    // Extract source detections and negative elevation counterparts in the base frame into a matrix, along with the
    // indices of network outputs corresponding to the source class.
    ROS_DEBUG("Extracting source positions from input detections...");
    this->extractSourcePositions(msg, probe_pose_base_eig, det_pos_base_mat, src_id_vec, num_src_det);

    // Run the prediction step of each existing LKF track.
    if(this->lkf_tracks.size() > 0)
    {
      ROS_DEBUG("Performing prediction step for %ld LKF-based tracks...", this->lkf_tracks.size());

      for(i0 = 0; i0 < this->lkf_tracks.size(); i0++)
      {
        if(this->lkf_tracks[i0]->lkf_fsm_state == LinearKalmanFsmState::LKF_DELETE)
        {
          // Delete tracks marked for deletion.
          this->lkf_tracks.erase(this->lkf_tracks.begin() + i0);
          i0--;
        }
        else
        {
          this->lkf_tracks[i0]->predict();
        }
      }
    }

    if(num_src_det > 0)
    {
      if(this->lkf_tracks.size() > 0)
      {
        // Compute the gated distance between each existing LKF track and each source detection extracted from the input
        // message.
        ROS_DEBUG("Associating %d source detections with %ld LKF-based tracks...", num_src_det, this->lkf_tracks.size());
        src_track_dist_mat = Eigen::MatrixXd::Zero(this->lkf_tracks.size(), num_src_det);

        for(i0 = 0; i0 < this->lkf_tracks.size(); i0++)
        {
          // Identify the source detection closest to the current track while computing these distances.
          min_dist_src_id = -1;

          for(i1 = 0; i1 < num_src_det; i1++)
          {
            // Determine whether the current source detection lies inside the gate of the current track. Also compute
            // the distance between the source detection and track prediction.
            if(this->lkf_tracks[i0]->gate(det_pos_base_mat.block<3, 1>(0, i1), src_track_dist_tmp))
            {
              // The current source detection lies within the gate of the current track (i.e., we might be able to
              // associate this detection and track). Store the computed distance in the corresponding location in the
              // matrix `src_track_dist_mat`.
              src_track_dist_mat(i0, i1) = src_track_dist_tmp;

              // The finiteness check might not be required here, as `src_track_dist_tmp` would only be infinite if the
              // corresponding measurement prediction covariance matrix were singular.
              if(
                (std::isfinite(src_track_dist_mat(i0, i1)))
                && ((min_dist_src_id == -1) || (src_track_dist_mat(i0, min_dist_src_id) > src_track_dist_mat(i0, i1))))
              {
                // Update the index of the source detection closest to the current track.
                min_dist_src_id = i1;
              }
              else
              {
                // No operation
              }
            }
            else
            {
              // The current source detection lies outside the gate of the current track. Mark the corresponding
              // distance as infinite to prevent possible association between the two further down in this function.
              src_track_dist_mat(i0, i1) = inf;
            }
          }

          // ROS_DEBUG_STREAM("src_track_dist_mat = " << std::endl << src_track_dist_mat);

          if(min_dist_src_id >= 0)
          {
            // Set the other distances to infinity to ensure that each track has at most one associated source
            // detection.
            for(i1 = 0; i1 < num_src_det; i1++)
            {
              if(i1 != min_dist_src_id)
              {
                src_track_dist_mat(i0, i1) = inf;
              }
              else
              {
                // No operation
              }
            }
          }
          else
          {
            // No operation
          }

          track_to_src_map.push_back(min_dist_src_id);
          // ROS_DEBUG_STREAM("src_track_dist_mat = " << std::endl << src_track_dist_mat);
        }

        // Merge tracks associated with the same source detection.
        for(i0 = 0; i0 < num_src_det; i0++)
        {
          // Determine the existing track closest to the current source detection.
          min_dist_track_id = -1;

          for(i1 = 0; i1 < this->lkf_tracks.size(); i1++)
          {
            if(
              (std::isfinite(src_track_dist_mat(i1, i0)))
              && ((min_dist_track_id == -1) || (src_track_dist_mat(min_dist_track_id, i0) > src_track_dist_mat(i1, i0))))
            {
              min_dist_track_id = i1;
            }
            else
            {
              // No operation
            }
          }

          if(min_dist_track_id >= 0)
          {
            for(i1 = 0; i1 < this->lkf_tracks.size(); i1++)
            {
              if((std::isfinite(src_track_dist_mat(i1, i0))) && (i1 != min_dist_track_id))
              {
                // The current track is associated with but not the closest to the current source detection, so merge
                // this track with track `min_dist_track_id` (i.e., delete this track).
                ROS_DEBUG("Merging track %d with track %d...", i1, min_dist_track_id);
                tracks_to_delete.push_back(i1);
              }
              else
              {
                // No operation
              }
            }
          }
          else
          {
            // The current source detection is not associated with any existing track, so mark it as a track originator.
            ROS_DEBUG("Marking detection %d as tracck originator...", i0);
            track_originators.push_back(i0);
          }
        }

        // Delete all tracks marked for merging.
        if(tracks_to_delete.size() > 0)
        {
          // Sort and remove duplicates from the indices of tracks to be deleted.
          sort(tracks_to_delete.begin(), tracks_to_delete.end());
          i0 = 1;

          while(i0 < tracks_to_delete.size())
          {
            if(tracks_to_delete[i0] == tracks_to_delete[i0 - 1])
            {
              tracks_to_delete.erase(tracks_to_delete.begin() + i0);
            }
            else
            {
              i0++;
            }
          }

          while(tracks_to_delete.size() > 0)
          {
            // Delete the LKF track indicated by the last index. Work from the highest index down to avoid errors
            // related to changing LKF track indices.
            this->lkf_tracks.erase(this->lkf_tracks.begin() + tracks_to_delete.back());

            // Delete the corresponding element in the map from track indices to source detection indices.
            track_to_src_map.erase(track_to_src_map.begin() + tracks_to_delete.back());

            // Delete the last index.
            tracks_to_delete.pop_back();
          }
        }
        else
        {
          // No operation
        }

        if(this->lkf_tracks.size() > 0)
        {
          for(i0 = 0; i0 < this->lkf_tracks.size(); i0++)
          {
            if(track_to_src_map[i0] >= 0)
            {
              // Update the current LKF track with the corresponding source detection using the map from track indices
              // to source detection indices. Also determine whether or not this involves a change in LKF states.
              track_states_changed |= (this->lkf_tracks[i0]->lkf_fsm_state != LKF_TRACKING);
              this->lkf_tracks[i0]->updateWithMeasurement(det_pos_base_mat.block<3, 1>(0, track_to_src_map[i0]));
            }
            else
            {
              // The current LKF track was not associated with a source detection, so update without. Also determine
              // whether or not this involves a change in LKF states.
              track_states_changed |= (this->lkf_tracks[i0]->lkf_fsm_state == LKF_TRACKING);
              this->lkf_tracks[i0]->updateWithoutMeasurement();
            }
          }
        }
        else
        {
          // No operation
        }
      }
      else
      {
        // No valid tracks exist, so mark each detection as a track originator.
        ROS_DEBUG(
          "Marking %d source detections as track originators (no tracks currently in existence)...", num_src_det);

        for(i0 = 0; i0 < num_src_det; i0++)
        {
          track_originators.push_back(i0);
        }
      }
    }
    else if(this->lkf_tracks.size() > 0)
    {
      // No source detections were obtained, so update each LKF track without a source detection.
      ROS_DEBUG(
        "Updating %ld LKF-based tracks without measurements (no source detections within constructed gates)...",
        this->lkf_tracks.size());

      for(i0 = 0; i0 < this->lkf_tracks.size(); i0++)
      {
        this->lkf_tracks[i0]->updateWithoutMeasurement();
      }

      track_states_changed = true;
    }
    else
    {
      // No currently running tracks and no new detections, so no operation
      ROS_WARN("No source detections in input message, and no tracks currently in existence.");
    }

    // Start a new track for each identified track originator.
    if(track_originators.size() > 0)
    {
      ROS_DEBUG("Initializing %ld new tracks...", track_originators.size());

      for(i0 = 0; i0 < track_originators.size(); i0++)
      {
        this->lkf_tracks.push_back(
          new LkfVelocity(det_pos_base_mat.block<3, 1>(0, track_originators[i0]), this->lkf_smooth_coeff));
      }
    }
    else
    {
      // No operation
    }

    // Publish the tracked target positions and velocities to the ROS topic.
    for(i0 = 0; i0 < this->lkf_tracks.size(); i0++)
    {
      switch(this->lkf_tracks[i0]->lkf_fsm_state)
      {
        case LKF_INITIALIZE:
          track_msg.fsm_state = "LKF_INITIALIZE";
          break;
        case LKF_TRACKING:
          track_msg.fsm_state = "LKF_TRACKING";
          multi_track_valid = true;
          break;
        case LKF_MISSED_ONE:
          track_msg.fsm_state = "LKF_MISSED_ONE";
          multi_track_valid = true;
          break;
        case LKF_MISSED_TWO:
          track_msg.fsm_state = "LKF_MISSED_TWO";
          multi_track_valid = true;
          break;
        case LKF_DELETE:
          track_msg.fsm_state = "LKF_DELETE";
          break;
        default:
          track_msg.fsm_state = "";
      }

      if(track_msg.fsm_state.length() > 0)
      {
        ROS_DEBUG("[%d/%ld] LKF state = %s", i0, this->lkf_tracks.size(), track_msg.fsm_state.c_str());
      }
      else
      {
        ROS_ERROR("[%d/%ld] Unknown LKF state %d.", i0, this->lkf_tracks.size(), this->lkf_tracks[i0]->lkf_fsm_state);
      }

      for(i1 = 0; i1 < 6; i1++)
      {
        track_msg.kalman_state[i1] = this->lkf_tracks[i0]->x_curr(i1);

        for(i2 = 0; i2 < 6; i2++)
        {
          track_msg.kalman_covariance[(9 * i1) + i2] = this->lkf_tracks[i0]->p_curr(i1, i2);
        }
      }

      track_msg.track_length = this->lkf_tracks[i0]->track_length;
      output_msg.filters.push_back(track_msg);
    }

    if(this->lkf_tracks.size() > 2)
    {
      ROS_WARN("Found %ld > 2 Kalman filter tracks.", this->lkf_tracks.size());
    }
    else
    {
      // No operation
    }

    output_msg.header.stamp = ros::Time::now();

    if(multi_track_valid)
    {
      output_msg.header.frame_id = this->base_frame_name;

      for(i0 = 0; i0 < this->lkf_tracks.size(); i0++)
      {
        if(this->lkf_tracks[i0]->lkf_fsm_state == LKF_TRACKING)
        {
          tracking_track_id_vec.push_back(i0);
        }
        else
        {
          // No operation
        }
      }

      if(tracking_track_id_vec.size() > 1)
      {
        this->tracking_track_dist_curr = 0.0;

        for(i0 = 0; i0 < tracking_track_id_vec.size(); i0++)
        {
          for(i1 = i0 + 1; i1 < tracking_track_id_vec.size(); i1++)
          {
            tracking_track_diff_vec = this->lkf_tracks[tracking_track_id_vec[i0]]->x_curr.block<3, 1>(0, 0)
              - this->lkf_tracks[tracking_track_id_vec[i1]]->x_curr.block<3, 1>(0, 0);
            this->tracking_track_dist_curr += tracking_track_diff_vec.norm();
          }
        }

        if(!track_states_changed)
        {
          // States have not changed, so compute the differential.
          tracking_track_dist_deriv = this->tracking_track_dist_curr - this->tracking_track_dist_prev;
        }
        else
        {
          // States have changed, so we cannot compute the differential.
          tracking_track_dist_deriv = 0.0;
        }

        this->tracking_track_dist_prev = this->tracking_track_dist_curr;
      }
      else
      {
        // We do not have a sufficient number of tracks in the tracking state, so reset the corresponding distance
        // metrics.
        this->tracking_track_dist_curr = 0.0;
        this->tracking_track_dist_prev = 0.0;
      }
    }
    else
    {
      output_msg.header.frame_id = "invalid";
    }

    output_msg.tracking_track_dist = this->tracking_track_dist_curr;
    output_msg.tracking_track_dist_deriv = tracking_track_dist_deriv;

    this->tgt_pos_pub.publish(output_msg);
  }
  catch(tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
  }
}
