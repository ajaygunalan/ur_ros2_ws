#include "gubbi_vs_2024/gubbi_class.hpp"

/*
 * \brief Execute the guidance algorithm
 */
void GubbiVisualServoingClass::executeGuidanceFsm(void)
{
  // Execute the appropriate function based on the current state of the FSM.
  switch(this->fsm_state)
  {
    case VS_TRACK_X:
    case VS_TRACK_Y:
      // Follow the selected target.
      ROS_INFO("Following source position estimate(s) (VS_TRACK_X|VS_TRACK_Y)...");
      this->guidanceFsmTrack();

      break;
    case VS_SEARCH:
      // Move the end effector in a pre-determined search pattern to search for the target.
      ROS_INFO("Searching for point source (VS_SEARCH)...");
      this->guidanceFsmSearch();

      break;
    case VS_INITIALIZE:
      ROS_INFO("Initializing robot (VS_INITIALIZE)...");
      this->guidanceFsmInitialize();

      break;
    default:
      // Keep the robot stationary.
      ROS_WARN_STREAM("Holding robot stationary (FSM state " << this->fsm_state << ")...");
      this->guidanceFsmFreeze();
  }
}

/*
 * \brief Hold the visual servoing system stationary indefinitely (no recent valid poses have been received).
 */
void GubbiVisualServoingClass::guidanceFsmFreeze(void)
{
  // We cannot issue a cancel command to the robot here as force control is performed independent of this guidance
  // algorithm.
  // TODO: Reset control filters to ensure that the robot does not move in the selected directions.
}

/*
 * \brief Publish the initial transform between the probe and base frame
 */
void GubbiVisualServoingClass::guidanceFsmInitialize(void)
{
  // Do nothing.
}

/*
 * \brief Transition checks for the finite state machine controlling the guidance algorithms
 */
void GubbiVisualServoingClass::guidanceFsmTransitionCheck(void)
{
  int i0;
  geometry_msgs::TransformStamped search_center_base_msg;
  int num_dist_increases = 0;

  Eigen::Vector3d inter_track_pos_diff;

  std::vector<int> lkf_tracking_id_vec;
  std::vector<int> lkf_init_id_vec;
  std::vector<int> lkf_missed_one_id_vec;
  std::vector<int> lkf_missed_two_id_vec;
  std::vector<double> track_speeds;

  if(this->fsm_state != VS_NO_CONTACT)
  {
    this->guidance_lkf_track_id = -1;

    // Search through the tracks and sort them by the corresponding states of their FSMs.
    for(i0 = 0; i0 < this->lkf_tracks.size(); i0++)
    {
      if(this->lkf_tracks[i0]->lkf_fsm_state == pulse_est::LinearKalmanFsmState::LKF_TRACKING)
      {
        lkf_tracking_id_vec.push_back(i0);
      }
      else if(this->lkf_tracks[i0]->lkf_fsm_state == pulse_est::LinearKalmanFsmState::LKF_INITIALIZE)
      {
        lkf_init_id_vec.push_back(i0);
      }
      else if(this->lkf_tracks[i0]->lkf_fsm_state == pulse_est::LinearKalmanFsmState::LKF_MISSED_ONE)
      {
        lkf_missed_one_id_vec.push_back(i0);
      }
      else if(this->lkf_tracks[i0]->lkf_fsm_state == pulse_est::LinearKalmanFsmState::LKF_MISSED_TWO)
      {
        lkf_missed_two_id_vec.push_back(i0);
      }
      else
      {
        // No operation
      }

      if(this->lkf_tracks[i0]->guidance_following)
      {
        this->guidance_lkf_track_id = i0;
        this->guidance_lkf_track_num_iter++;
      }
      else
      {
        // No operation
      }
    }

    if(this->guidance_lkf_track_id < 0)
    {
      if(lkf_tracking_id_vec.size() > 0)
      {
        // We have one or more valid tracks, so just pick the first one among these to be followed.
        this->guidance_lkf_track_id = lkf_tracking_id_vec[0];
      }
      else if(lkf_init_id_vec.size() > 0)
      {
        // We do not have any valid tracks currently tracking, but we do have one or more freshly initialized tracks
        // which is the next best thing. Pick and follow the first one among these.
        this->guidance_lkf_track_id = lkf_init_id_vec[0];
      }
      else if(lkf_missed_one_id_vec.size() > 0)
      {
        // We do not have any currently tracking or freshly initialized tracks, but we do have one or more tracks which
        // are outdated by one measurement. Select and follow the first of these, assuming that we got a single missed
        // detection from the network.
        this->guidance_lkf_track_id = lkf_missed_one_id_vec[0];
      }
      else if(lkf_missed_two_id_vec.size() > 0)
      {
        // We do not have any currently tracking or freshly initialized tracks, but we do have one or more tracks which
        // are outdated by two measurements. Select and follow the first of these, assuming that we got two consecutive
        // missed detections from the network.
        this->guidance_lkf_track_id = lkf_missed_two_id_vec[0];
      }
      else
      {
        // We do not have any valid or near-valid (i.e., outdated by at most two measurements) tracks. Search and/or
        // freeze further down.
      }

      this->guidance_lkf_track_num_iter = 0;
    }
    else
    {
      // No operation
    }

    if(this->guidance_lkf_track_id >= 0)
    {
      if(this->fsm_state != VS_TRACK_Y)
      {
        this->fsm_state = VS_TRACK_X;
      }
      else
      {
        // No operation
      }

      this->lkf_tracks[this->guidance_lkf_track_id]->guidance_following = true;
    }
    else
    {
      // No LKF track is currently being followed.
      switch(this->fsm_state)
      {
        case VS_SEARCH:
          // Continue in the search state.
          break;
        case VS_FREEZE:
          // Hold the robot stationary indefinitely.
          this->fsm_state = VS_FREEZE;
          break;
        default:
          this->search_num_iter = 0;
          this->fsm_state = VS_SEARCH;
      }
    }
  }
  else
  {
    ROS_DEBUG("Robot is not in contact with surface.");
  }
}

/*
 * \brief Search for the point source
 */
void GubbiVisualServoingClass::guidanceFsmSearch(void)
{
  Eigen::Isometry3d probe_pose_base_eig;
  Eigen::MatrixXd probe_pose_base_mat;
  geometry_msgs::TransformStamped probe_pose_base_msg;

  if(this->search_num_iter == 0)
  {
    try
    {
      probe_pose_base_msg =
        this->tf_buffer.lookupTransform(this->base_frame_name, this->probe_frame_name, ros::Time(0));
      tf::transformMsgToEigen(probe_pose_base_msg.transform, probe_pose_base_eig);
      probe_pose_base_mat = probe_pose_base_eig.matrix();

      this->search_wrist_3_goal = (probe_pose_base_mat(0, 0) * probe_pose_base_mat(1, 0) > 0) ? -1 : 1;
    }
    catch(tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
    }
  }
  else if(this->search_num_iter % 128 == 64)
  {
    this->search_wrist_3_goal *= -1.0;
  }
  else
  {
    // No operation
  }

  if(this->search_num_iter > 1000)
  {
    this->fsm_state = VS_FREEZE;
  }
  else
  {
    // No operation
  }

  this->search_num_iter++;
}

/*
 * \brief Track and center the probe above the estimated point source location.
 */
void GubbiVisualServoingClass::guidanceFsmTrack(void)
{
  double probe_cmd_rot_probe_z;
  Eigen::Isometry3d base_pose_probe_eig;
  Eigen::Isometry3d probe_cmd_pose_probe_eig;
  Eigen::Isometry3d probe_pose_base_eig;
  Eigen::MatrixXd probe_pose_base_mat;
  Eigen::MatrixXd pt_src_pos_base_vec = Eigen::MatrixXd::Zero(4, 1);
  Eigen::MatrixXd pt_src_pos_probe_vec = Eigen::MatrixXd::Zero(4, 1);
  Eigen::Quaterniond probe_cmd_rot_probe_q;
  geometry_msgs::TransformStamped base_pose_probe_msg;
  geometry_msgs::TransformStamped probe_pose_base_msg;

  try
  {
    base_pose_probe_msg = this->tf_buffer.lookupTransform(this->probe_frame_name, this->base_frame_name, ros::Time(0));
    tf::transformMsgToEigen(base_pose_probe_msg.transform, base_pose_probe_eig);
    probe_pose_base_msg = this->tf_buffer.lookupTransform(this->base_frame_name, this->probe_frame_name, ros::Time(0));
    tf::transformMsgToEigen(probe_pose_base_msg.transform, probe_pose_base_eig);

    // Fetch the estimated point source position in the probe frame.
    pt_src_pos_base_vec.block<3, 1>(0, 0) = this->lkf_tracks[this->guidance_lkf_track_id]->x_curr.block<3, 1>(0, 0);
    pt_src_pos_base_vec(3) = 1.0;

    // We do not want this guidance module to move the probe along the axial dimension of the probe, so identify and
    // zero out the corresponding component in the probe frame.
    pt_src_pos_probe_vec = base_pose_probe_eig.matrix() * pt_src_pos_base_vec;
    pt_src_pos_probe_vec(2) = 0.0;

    // Construct the commanded probe pose in the current probe frame.
    probe_cmd_pose_probe_eig.setIdentity();

    if((abs(pt_src_pos_probe_vec(0)) > 1e-3) || (abs(pt_src_pos_probe_vec(1)) <= 5e-3))
    {
      this->fsm_state = VS_TRACK_X;

      pt_src_pos_probe_vec(1) = 0.0;
      ROS_DEBUG(
        "Commanded probe translation in probe frame: [%.2f, %.2f, %.2f] mm", 1.0e3 * pt_src_pos_probe_vec(0),
        1.0e3 * pt_src_pos_probe_vec(1), 1.0e3 * pt_src_pos_probe_vec(2));

      probe_cmd_pose_probe_eig.pretranslate(pt_src_pos_probe_vec.block<3, 1>(0, 0));
    }
    else
    {
      if(this->fsm_state != VS_TRACK_Y)
      {
        this->fsm_state = VS_TRACK_Y;
        this->track_y_num_iter = 0;
        probe_pose_base_mat = probe_pose_base_eig.matrix();

        this->track_y_wrist_3_goal = (probe_pose_base_mat(0, 0) * probe_pose_base_mat(1, 0) > 0.0) ? -1 : 1;
      }
      else
      {
        if(this->track_y_num_iter % 64 == 32)
        {
          this->track_y_wrist_3_goal *= -1;
        }
        else
        {
          // No operation
        }

        this->track_y_num_iter++;
      }

      ROS_DEBUG("VS_TRACK_Y for %d iterations with rotation %.1f\n", this->track_y_num_iter, this->track_y_wrist_3_goal);
    }

    // probe_cmd_pose_probe_eig.prerotate(probe_cmd_rot_probe_q);
    ROS_DEBUG_STREAM("Commanded probe pose in probe frame: [" << std::endl << probe_cmd_pose_probe_eig.matrix() << "]");

    // Transform the commanded probe pose to the base frame.
    this->probe_cmd_pose_base_eig = probe_pose_base_eig * probe_cmd_pose_probe_eig;
  }
  catch(tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
  }
}

/*
 * \brief Publish the state and output of the guidance algorithm
 */
void GubbiVisualServoingClass::publishTrackingGuidance(void)
{
  geometry_msgs::TransformStamped probe_cmd_pose_msg;

  tf::transformEigenToMsg(this->probe_cmd_pose_base_eig, probe_cmd_pose_msg.transform);
  probe_cmd_pose_msg.header.stamp = ros::Time::now();
  probe_cmd_pose_msg.header.frame_id = this->base_frame_name;
  probe_cmd_pose_msg.child_frame_id = this->probe_frame_name;
  this->guidance_pub.publish(probe_cmd_pose_msg);
}

void GubbiVisualServoingClass::resetGuidanceTrackingDistanceBuffer(void)
{
  int i0;

  for(i0 = 0; i0 < this->guidance_lkf_dist_buff_len; i0++)
  {
    this->guidance_lkf_dist_buff[i0] = 0.0;
    this->guidance_lkf_dist_deriv_buff[i0] = 0.0;
  }

  this->guidance_lkf_dist_buff_id = 0;
}
