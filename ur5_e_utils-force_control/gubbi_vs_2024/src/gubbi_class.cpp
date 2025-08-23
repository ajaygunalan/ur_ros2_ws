#include "gubbi_vs_2024/gubbi_class.hpp"

/*
 * \brief Construct an object of the type `GubbiVisualServoingClass`
 */
GubbiVisualServoingClass::GubbiVisualServoingClass(
  ros::NodeHandle& nh, std::string action_server_name, std::string ee_name, double delta_t, double joint_omega_max,
  double joint_alpha_max)
  : Ur5eBaseClass(nh, action_server_name, ee_name, delta_t, joint_omega_max, joint_alpha_max)
  , fsm_state(VS_INITIALIZE)
  , lkf_track_dist_curr(-1.0)
  , lkf_track_dist_prev(-1.0)
  , lkf_track_dist_deriv(0.0)
  , invalid_pose_counter(0)
  , pause_max_invalid_count(10)
  , search_max_invalid_count(50)
  , src_class_id(0)
  , guidance_lkf_dist_buff_len(5)
  , guidance_lkf_dist_buff_id(0)
  , guidance_lkf_track_id(-1)
  , probe_frame_name(ee_name)
  , track_y_num_iter(0)
  , track_y_wrist_3_goal(1)
  , search_num_iter(0)
  , search_wrist_3_goal(1)
{
  ROS_INFO("Node namespace: %s", this->nh.getNamespace().c_str());

  std::string detection_topic_name;
  std::string force_torque_topic_name;

  // Fetch frame names corresponding to robot base and ultrasound transducer.
  if(this->nh.hasParam("base_frame_name"))
  {
    this->nh.getParam("base_frame_name", this->base_frame_name);
  }
  else
  {
    this->base_frame_name = "base_link";
    ROS_WARN(
      "Could not find ROS parameter 'base_frame_name', using default value '%s'...", this->base_frame_name.c_str());
  }

  if(this->nh.hasParam("force_f_z_goal"))
  {
    this->nh.getParam("force_f_z_goal", this->force_f_z_goal);
  }
  else
  {
    this->force_f_z_goal = -1.5;
    ROS_WARN("Could not find ROS parameter 'force_f_z_goal', using default value '%.2f'...", this->force_f_z_goal);
  }

  if(this->nh.hasParam("force_f_z_max"))
  {
    this->nh.getParam("force_f_z_max", this->force_f_z_max);
  }
  else
  {
    this->force_f_z_max = -0.5;
    ROS_WARN("Could not find ROS parameter 'force_f_z_max', using default value '%.2f'...", this->force_f_z_max);
  }

  if(this->nh.hasParam("force_f_z_thresh"))
  {
    this->nh.getParam("force_f_z_thresh", this->force_f_z_thresh);
  }
  else
  {
    this->force_f_z_thresh = 0.2;
    ROS_WARN("Could not find ROS parameter 'force_f_z_thresh', using default value '%.2f'...", this->force_f_z_thresh);
  }

  if(this->nh.hasParam("lkf_track_dist_filter_coeff"))
  {
    this->nh.getParam("lkf_track_dist_filter_coeff", this->lkf_track_dist_filter_coeff);
  }
  else
  {
    this->lkf_track_dist_filter_coeff = 0.5;
    ROS_WARN(
      "Could not find ROS parameter 'lkf_track_dist_filter_coeff', using default value %.2f...",
      this->lkf_track_dist_filter_coeff);
  }

  if(this->nh.hasParam("ignore_elevation"))
  {
    this->nh.getParam("ignore_elevation", this->ignore_elevation);
  }
  else
  {
    this->ignore_elevation = false;
  }

  if(this->nh.hasParam("single_det"))
  {
    this->nh.getParam("single_det", this->single_det);
  }
  else
  {
    this->single_det = false;
  }

  if(this->nh.hasParam("detection_topic_name"))
  {
    this->nh.getParam("detection_topic_name", detection_topic_name);
  }
  else
  {
    detection_topic_name = "/pt_src_loc_sys_3a/detections";
    ROS_WARN(
      "Could not fetch parameter 'detection_topic_name', using default value '%s'...", detection_topic_name.c_str());
  }

  if(this->nh.hasParam("force_torque_topic_name"))
  {
    this->nh.getParam("force_torque_topic_name", force_torque_topic_name);
  }
  else
  {
    force_torque_topic_name = "/netft/proc_probe";
    ROS_WARN(
      "Could not fetch parameter 'force_torque_topic_name', using default value '%s'...",
      force_torque_topic_name.c_str());
  }

  // Initialize buffers of inter-track distances used by guidance algorithm.
  this->guidance_lkf_dist_buff.resize(this->guidance_lkf_dist_buff_len);
  this->guidance_lkf_dist_deriv_buff.resize(this->guidance_lkf_dist_buff_len);

  // Initialize publishers of states and outputs of the estimation and guidance algorithms.
  this->guidance_pub = this->nh.advertise<geometry_msgs::TransformStamped>("guidance", 1);
  this->lkf_track_pub = this->nh.advertise<pulse_vs_msgs::KalmanArray>("multi_track_lkf", 1);

  // TODO: Initialize remaining publishers.

  this->tf_listener = new tf2_ros::TransformListener(this->tf_buffer);
  this->detection_sub =
    this->nh.subscribe(detection_topic_name, 1, &GubbiVisualServoingClass::detectionSubscriberCallback, this);
  this->force_torque_sub =
    this->nh.subscribe(force_torque_topic_name, 1, &GubbiVisualServoingClass::forceTorqueSubscriberCallback, this);

  this->ctrl_timer =
    this->nh.createTimer(ros::Duration(this->delta_t), &GubbiVisualServoingClass::controlTimerCallback, this);
}

void GubbiVisualServoingClass::detectionSubscriberCallback(pulse_vs_msgs::Segmentation2DArray det_msg)
{
  // Matrix of detected source positions (including positive and negative elevation displacements) extracted from
  // input message
  Eigen::MatrixXd det_pos_base_mat;

  // Current transform from probe frame to robot base frame
  geometry_msgs::TransformStamped base_pose_probe_msg;

  // Current transform from robot base frame to probe frame
  geometry_msgs::TransformStamped probe_pose_base_msg;

  // Number of source detections (including positive and negative elevation displacements) extracted from input message
  int num_src_det;

  if(this->fsm_state != VS_NO_CONTACT)
  {
    try
    {
      // Update the transform from the base frame to the probe frame.
      base_pose_probe_msg =
        this->tf_buffer.lookupTransform(this->probe_frame_name, this->base_frame_name, ros::Time(0));
      this->base_pose_probe_eig = tf2::transformToEigen(base_pose_probe_msg);

      // Update the transform from the probe frame to the base frame.
      probe_pose_base_msg =
        this->tf_buffer.lookupTransform(this->base_frame_name, this->probe_frame_name, ros::Time(0));
      this->probe_pose_base_eig = tf2::transformToEigen(probe_pose_base_msg);

      // Parse through received message for source detections.
      this->extractSourcePositions(det_msg, det_pos_base_mat, num_src_det);

      // Predict the source positions using the multi-track LKF.
      this->predictSourcePositions();

      // Update the source positions tracked with the multi-track LKF using the extracted network outputs.
      this->updateSourcePositions(det_pos_base_mat, num_src_det);

      // Publish the current states and outputs of the LKF tracks.
      this->publishLkfTracks();

      // Determine desired probe position from multi-track LKF.
      this->guidanceFsmTransitionCheck();

      // Execute the guidance algorithm based on the updated guidance FSM state.
      this->executeGuidanceFsm();

      // Publish the output of the guidance algorithm.
      this->publishTrackingGuidance();
    }
    catch(tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
    }
  }
  else
  {
    ROS_DEBUG("Resetting estimation filters due lack of contact between probe and surface...");

    while(!this->lkf_tracks.empty())
    {
      delete this->lkf_tracks.back();
      this->lkf_tracks.pop_back();
    }

    this->resetLkfTrackingDistanceFilter();
  }
}

void GubbiVisualServoingClass::forceTorqueSubscriberCallback(geometry_msgs::WrenchStamped ft_msg)
{
  this->force_f_z_curr = ft_msg.wrench.force.z;

  if(this->force_f_z_curr > this->force_f_z_max)
  {
    ROS_WARN("No contact!");
    this->fsm_state = VS_NO_CONTACT;
  }
  else if(this->fsm_state == VS_NO_CONTACT)
  {
    // We are now in contact with the surface, so allow other functions to work.
    ROS_DEBUG("Contact acquired, maintaining...");
    this->fsm_state = VS_FREEZE;
  }
}
