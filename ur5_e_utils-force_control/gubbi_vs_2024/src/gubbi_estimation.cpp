#include "gubbi_vs_2024/gubbi_class.hpp"

/*
 * \brief Compute the sum of pair-wise distances among LKF tracks in the tracking state
 */
void GubbiVisualServoingClass::computeLkfTrackingDistance(void)
{
  Eigen::Vector3d lkf_track_diff_vec;
  int i0;
  int i1;
  std::vector<int> tracking_id_vec;

  // Fetch the indices of LKF tracks in the tracking state.
  for(i0 = 0; i0 < this->lkf_tracks.size(); i0++)
  {
    if(this->lkf_tracks[i0]->lkf_fsm_state == pulse_est::LinearKalmanFsmState::LKF_TRACKING)
    {
      tracking_id_vec.push_back(i0);
    }
    else
    {
      // No operation
    }
  }

  if(tracking_id_vec.size() > 1)
  {
    this->lkf_track_dist_curr = 0.0;

    for(i0 = 0; i0 < tracking_id_vec.size(); i0++)
    {
      for(i1 = i0 + 1; i1 < tracking_id_vec.size(); i1++)
      {
        // Compute the distance between the current pair of LKF tracks in the tracking state.
        lkf_track_diff_vec = this->lkf_tracks[tracking_id_vec[i0]]->x_curr.block<3, 1>(0, 0)
          - this->lkf_tracks[tracking_id_vec[i1]]->x_curr.block<3, 1>(0, 0);
        this->lkf_track_dist_curr += lkf_track_diff_vec.norm();
      }
    }

    if(this->lkf_track_dist_prev >= 0.0)
    {
      // Apply a first order filter to the computed sum of pairwise distances.
      this->lkf_track_dist_curr = ((1.0 - this->lkf_track_dist_filter_coeff) * this->lkf_track_dist_prev)
        + (this->lkf_track_dist_filter_coeff * this->lkf_track_dist_curr);

      // The filter is running normally, so we can compute the derivative without any issues.
      this->lkf_track_dist_deriv = this->lkf_track_dist_curr - this->lkf_track_dist_prev;
    }
    else
    {
      // The filter was just reset, so do not compute the derivative this iteration.
      this->lkf_track_dist_deriv = 0.0;
    }

    ROS_DEBUG(
      "Pairwise track distances = %.2f mm <- %.2f mm (change of %.2f mm)", 1.0e3 * this->lkf_track_dist_curr,
      1.0e3 * this->lkf_track_dist_prev, 1.0e3 * this->lkf_track_dist_deriv);

    // Update the previous distance for the next iteration (i.e., input from the point source localization system).
    this->lkf_track_dist_prev = this->lkf_track_dist_curr;
  }
  else
  {
    ROS_DEBUG("Found %ld LKF track(s), cannot compute sum of pairwise distances.", tracking_id_vec.size());
  }
}

/*
 * \brief Extract the source positions in the base frame from the input message.
 */
void GubbiVisualServoingClass::extractSourcePositions(
  pulse_vs_msgs::Segmentation2DArray det_msg, Eigen::MatrixXd& det_pos_base_mat, int& num_src_det)
{
  // Positions of source detections and negative elevation counterparts in probe frame.
  Eigen::MatrixXd det_pos_probe_mat;

  int i0;
  int det_pos_probe_row_id = 0;
  std::vector<int> src_id_vec;

  // Determine if any of the detections correspond to the source class. This determines whether or not the weights
  // need to be updated.
  for(i0 = 0; i0 < det_msg.segmentations.size(); i0++)
  {
    if((det_msg.segmentations[i0].results.size() == 1) && (det_msg.segmentations[i0].results[0].id == this->src_class_id))
    {
      if(this->single_det)
      {
        // Only a single detection can be selected from the options available. Choose the source with the maximum
        // confidence score.
        if(src_id_vec.empty())
        {
          src_id_vec.push_back(i0);
        }
        else if(det_msg.segmentations[i0].results[0].score > det_msg.segmentations[src_id_vec[0]].results[0].score)
        {
          src_id_vec[0] = i0;
        }
        else
        {
          // No operation
        }
      }
      else
      {
        src_id_vec.push_back(i0);
      }
    }
    else if(det_msg.segmentations[i0].results.size() > 1)
    {
      ROS_ERROR("Unexpected number of results (%ld > 1).", det_msg.segmentations[i0].results.size());
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
      det_pos_probe_mat(0, det_pos_probe_row_id) =
        det_msg.segmentations[src_id_vec[i0]].results[0].pose.pose.position.x;

      if(this->ignore_elevation)
      {
        det_pos_probe_mat(1, det_pos_probe_row_id) = 0.0;
      }
      else
      {
        det_pos_probe_mat(1, det_pos_probe_row_id) =
          det_msg.segmentations[src_id_vec[i0]].results[0].pose.pose.position.y;
      }

      det_pos_probe_mat(2, det_pos_probe_row_id) =
        det_msg.segmentations[src_id_vec[i0]].results[0].pose.pose.position.z;
      det_pos_probe_mat(3, det_pos_probe_row_id) = 1.0;
      det_pos_probe_row_id++;

      if(abs(det_pos_probe_mat(1, det_pos_probe_row_id)) > 1e-4)
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
    det_pos_base_mat = this->probe_pose_base_eig.matrix() * det_pos_probe_mat.block(0, 0, 4, num_src_det);
  }
  else
  {
    // No operation
  }

  ROS_DEBUG("Extracted %d source detections with positions:", num_src_det);

  for(i0 = 0; i0 < num_src_det; i0++)
  {
    ROS_DEBUG(
      "\n\t[%d/%d] [%.2f, %.2f, %.2f] mm ('%s') [%.2f, %.2f, %.2f] mm ('%s')", i0 + 1, num_src_det,
      1.0e3 * det_pos_probe_mat(0, i0), 1.0e3 * det_pos_probe_mat(1, i0), 1.0e3 * det_pos_probe_mat(2, i0),
      this->probe_frame_name.c_str(), 1.0e3 * det_pos_base_mat(0, i0), 1.0e3 * det_pos_base_mat(1, i0),
      1.0e3 * det_pos_base_mat(2, i0), this->base_frame_name.c_str());
  }
}

void GubbiVisualServoingClass::predictSourcePositions(void)
{
  int i0;

  // Run the prediction step of each existing LKF track.
  if(this->lkf_tracks.size() > 0)
  {
    for(i0 = 0; i0 < this->lkf_tracks.size(); i0++)
    {
      if(this->lkf_tracks[i0]->lkf_fsm_state != pulse_est::LinearKalmanFsmState::LKF_DELETE)
      {
        this->lkf_tracks[i0]->predict();
      }
      else
      {
        // Delete tracks marked for deletion.
        this->lkf_tracks.erase(this->lkf_tracks.begin() + i0);
        i0--;
      }
    }
  }
  else
  {
    ROS_WARN("No LKF tracks currently in existence.");
  }
}

/*
 * \brief Publish the states and outputs of the current LKF tracks
 */
void GubbiVisualServoingClass::publishLkfTracks(void)
{
  bool multi_track_valid = false;
  int i0;
  int i1;
  int i2;
  pulse_vs_msgs::Kalman track_msg;
  pulse_vs_msgs::KalmanArray output_msg;

  // Publish the tracked target positions and velocities to the ROS topic.
  for(i0 = 0; i0 < this->lkf_tracks.size(); i0++)
  {
    switch(this->lkf_tracks[i0]->lkf_fsm_state)
    {
      case pulse_est::LinearKalmanFsmState::LKF_INITIALIZE:
        track_msg.fsm_state = "LKF_INITIALIZE";
        break;
      case pulse_est::LinearKalmanFsmState::LKF_TRACKING:
        track_msg.fsm_state = "LKF_TRACKING";
        multi_track_valid = true;
        break;
      case pulse_est::LinearKalmanFsmState::LKF_MISSED_ONE:
        track_msg.fsm_state = "LKF_MISSED_ONE";
        multi_track_valid = true;
        break;
      case pulse_est::LinearKalmanFsmState::LKF_MISSED_TWO:
        track_msg.fsm_state = "LKF_MISSED_TWO";
        multi_track_valid = true;
        break;
      case pulse_est::LinearKalmanFsmState::LKF_DELETE:
        track_msg.fsm_state = "LKF_DELETE";
        break;
      default:
        track_msg.fsm_state = "";
    }

    for(i1 = 0; i1 < 6; i1++)
    {
      track_msg.kalman_state[i1] = this->lkf_tracks[i0]->x_curr(i1);

      for(i2 = 0; i2 < 6; i2++)
      {
        track_msg.kalman_covariance[(9 * i1) + i2] = this->lkf_tracks[i0]->p_curr(i1, i2);
      }
    }

    if(track_msg.fsm_state.length() > 0)
    {
      ROS_DEBUG(
        "[%d/%ld] LKF state = %s, [%.2f, %.2f, %.2f] mm, [%.2f, %.2f, %.2f] mm/s (%.2f mm/s)", i0,
        this->lkf_tracks.size(), track_msg.fsm_state.c_str(), 1.0e3 * track_msg.kalman_state[0],
        1.0e3 * track_msg.kalman_state[1], 1.0e3 * track_msg.kalman_state[2], 1.0e3 * track_msg.kalman_state[3],
        1.0e3 * track_msg.kalman_state[4], 1.0e3 * track_msg.kalman_state[5],
        1.0e3 * this->lkf_tracks[i0]->x_curr.block<3, 1>(3, 0).norm());
    }
    else
    {
      ROS_ERROR("[%d/%ld] Unknown LKF state %d.", i0, this->lkf_tracks.size(), this->lkf_tracks[i0]->lkf_fsm_state);
    }

    track_msg.track_length = this->lkf_tracks[i0]->track_length;
    output_msg.filters.push_back(track_msg);
  }

  output_msg.header.stamp = ros::Time::now();

  if(multi_track_valid)
  {
    output_msg.header.frame_id = this->base_frame_name;
  }
  else
  {
    output_msg.header.frame_id = "invalid";
  }

  output_msg.tracking_track_dist = this->lkf_track_dist_curr;
  output_msg.tracking_track_dist_deriv = this->lkf_track_dist_deriv;

  this->lkf_track_pub.publish(output_msg);
}

/*
 * \brief Reset the first order filter applied to the pair-wise distances of LKF tracks in the tracking state
 */
void GubbiVisualServoingClass::resetLkfTrackingDistanceFilter(void)
{
  ROS_WARN("Resetting LKF tracking distance filter...");

  this->lkf_track_dist_curr = -1.0;
  this->lkf_track_dist_prev = -1.0;
  this->lkf_track_dist_deriv = 0.0;
}

void GubbiVisualServoingClass::updateSourcePositions(Eigen::MatrixXd det_pos_base_mat, int num_src_det)
{
  // Flag indicating whether or not any LKF tracks have entered or exited the tracking state.
  bool track_states_changed = false;

  double inf = std::numeric_limits<double>::infinity();

  // Distance between a given source detection and track.
  double src_track_dist_tmp;

  // [N x M] Matrix containing the distance of each source detection (columns) from each existing track (rows)
  Eigen::MatrixXd src_track_dist_mat;

  int i0;
  int i1;

  // Index of source closest to a given track.
  int min_dist_src_id;

  // Index of LKF track closest to a given source detection.
  int min_dist_track_id;

  // Vector of source detection indices to be used to start new LKF tracks.
  std::vector<int> track_originators;

  // Map from LKF tracks to corresponding closest source detections.
  std::vector<int> track_to_src_map;

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
              this->lkf_tracks[i1]->lkf_fsm_state = pulse_est::LinearKalmanFsmState::LKF_DELETE;
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
          ROS_DEBUG("Marking detection %d as track originator...", i0);
          track_originators.push_back(i0);
        }
      }

      // Search for and remove tracks marked for deletion.
      for(i0 = 0; i0 < this->lkf_tracks.size(); i0++)
      {
        if(this->lkf_tracks[i0]->lkf_fsm_state == pulse_est::LinearKalmanFsmState::LKF_DELETE)
        {
          delete this->lkf_tracks[i0];
          this->lkf_tracks.erase(this->lkf_tracks.begin() + i0);

          // Also remove the corresponding element from the track-to-source map.
          track_to_src_map.erase(track_to_src_map.begin() + i0);
          i0--;
        }
        else
        {
          // No operation
        }
      }

      if(this->lkf_tracks.size() > 0)
      {
        for(i0 = 0; i0 < this->lkf_tracks.size(); i0++)
        {
          if(track_to_src_map[i0] >= 0)
          {
            // Update the current LKF track with the corresponding source detection using the map from track indices
            // to source detection indices. Also determine whether or not this involves a change in LKF states.
            track_states_changed |=
              (this->lkf_tracks[i0]->lkf_fsm_state != pulse_est::LinearKalmanFsmState::LKF_TRACKING);
            this->lkf_tracks[i0]->updateWithMeasurement(det_pos_base_mat.block<3, 1>(0, track_to_src_map[i0]));
          }
          else
          {
            // The current LKF track was not associated with a source detection, so update without. Also determine
            // whether or not this involves a change in LKF states.
            track_states_changed |=
              (this->lkf_tracks[i0]->lkf_fsm_state == pulse_est::LinearKalmanFsmState::LKF_TRACKING);
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
      for(i0 = 0; i0 < num_src_det; i0++)
      {
        track_originators.push_back(i0);
      }
    }
  }
  else if(this->lkf_tracks.size() > 0)
  {
    // No source detections were obtained, so update each LKF track without a source detection.
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
    for(i0 = 0; i0 < track_originators.size(); i0++)
    {
      this->lkf_tracks.push_back(new pulse_est::LkfVelocity(
        det_pos_base_mat.block<3, 1>(0, track_originators[i0]), this->lkf_track_dist_filter_coeff));
    }
  }
  else
  {
    // No operation
  }

  if(track_states_changed)
  {
    // TODO: Reset the smoothing filter to be applied to the derivative of pair-wise distances among LKF tracks in the
    // tracking state.
    this->resetLkfTrackingDistanceFilter();

    // TODO: Reset the index of the track being followed by the guidance function.
  }
  else
  {
    // No operation
  }

  this->computeLkfTrackingDistance();
}

