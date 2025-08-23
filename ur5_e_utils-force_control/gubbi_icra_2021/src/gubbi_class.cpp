/**
 *
 * \file gubbi_class.cpp
 *
 * \brief Class for ultrasound or photoacoustic-based visual servoing system developed by members of the PULSE Lab.
 *
 * \details The visual servoing node subscribes to a topic containing information regarding the target position and
 * validity, and moves the robot according to the received messages.
 *
 * \references
 * 1. Gubbi and Bell, "Deep Learning-Based Photoacoustic Visual Servoing: Using Outputs from Raw Sensor Data as Inputs
 * to a Robot Controller", IEEE ICRA 2021.
 *
 * \author Mardava Gubbi <mgubbi1@jhu.edu>
 *
 */
#include "gubbi_icra_2021/gubbi_class.h"

/*
 * \brief Construct an object of the type Ur5eGubbiVisualServoingClass.
 */
Ur5eGubbiVisualServoingClass::Ur5eGubbiVisualServoingClass(
  ros::NodeHandle& nh, std::string action_server_name, std::string ee_name, double delta_t, double joint_omega_max,
  double search_r_dot, double search_omega, int wait_max_count, int search_max_count, std::string tgt_loc_ns)
  : Ur5eBaseClass(nh, action_server_name, ee_name, delta_t, joint_omega_max)
  , search_r_dot(search_r_dot)
  , search_omega(search_omega)
{
  int i0;
  int i1;

  // Initialize FSM state and other variables required for visual servoing.
  ROS_DEBUG_STREAM("Initializing visual servoing interaction matrix...");
  this->fsm_state = -1;
  this->invalid_pose_counter = 0;
  this->fsm_follow_tgt_pose_seq_prev = 0;
  this->fsm_follow_tgt_pose_stamp_prev = ros::Time::now();
  this->fsm_trans_tgt_pose_seq_prev = 0;
  this->fsm_trans_tgt_pose_stamp_prev = ros::Time::now();

  // Set the maximum displacement for a given end effector velocity.
  if(this->nh.hasParam("path_planning_delta_x_max"))
  {
    ROS_DEBUG_STREAM("Fetching parameter 'path_planning_delta_x_max'...");
    this->nh.getParam("path_planning_delta_x_max", this->path_planning_delta_x_max);
  }
  else
  {
    ROS_DEBUG_STREAM("Setting maximum displacement for path planning to 2 mm...");
    this->path_planning_delta_x_max = 2e-3;
  }

  this->e_p_vec = Eigen::MatrixXd(6, 1);
  this->e_i_vec = Eigen::MatrixXd(6, 1);

  for(i0 = 0; i0 < this->e_p_vec.rows(); ++i0)
  {
    this->e_p_vec(i0) = 0.0;
    this->e_i_vec(i0) = 0.0;
  }

  this->k_p_mat = Eigen::MatrixXd(6, 6);
  this->k_i_mat = Eigen::MatrixXd(6, 6);
  this->k_d_mat = Eigen::MatrixXd(6, 6);

  for(i0 = 0; i0 < this->k_p_mat.rows(); ++i0)
  {
    for(i1 = 0; i1 < this->k_p_mat.cols(); ++i1)
    {
      this->k_p_mat(i0, i1) = 0.0;
      this->k_i_mat(i0, i1) = 0.0;
      this->k_d_mat(i0, i1) = 0.0;
    }
  }

  this->k_p_mat(0, 0) = 0.8;
  this->k_p_mat(1, 1) = 0.8;

  this->v_cmd = Eigen::MatrixXd(6, 1);

  for(i0 = 0; i0 < this->v_cmd.rows(); ++i0)
  {
    this->v_cmd(i0, 0) = 0.0;
  }

  // Initialize publishers before subscribers and timer callbacks to prevent uninitialized publishers from being called
  // in the callback functions on separate threads before the end of the constructor on the main thread.
  ROS_DEBUG_STREAM("Initializing error and FSM state publishers...");
  this->error_pub = this->nh.advertise<std_msgs::Float64MultiArray>("/visual_servoing/error", 10);
  this->fsm_state_pub = this->nh.advertise<std_msgs::Int16>("/visual_servoing/fsm_state", 10);

  ROS_DEBUG_STREAM("Initializing timer...");
  this->fsm_timer = this->nh.createTimer(ros::Duration(0.1), &Ur5eGubbiVisualServoingClass::fsmExecution, this);

  ROS_DEBUG_STREAM("Initializing subscriber to target pose topic...");
  this->tgt_pose_sub =
    this->nh.subscribe(tgt_loc_ns + "/target_pose", 1, &Ur5eGubbiVisualServoingClass::targetPoseCallback, this);
}

/*
 * \brief Execute the finite state machine of the visual servoing system.
 */
void Ur5eGubbiVisualServoingClass::fsmExecution(const ros::TimerEvent& ev)
{
  // Check for required FSM transitions prior to execution.
  this->fsmTransitionCheck();

  // Execute the appropriate function based on the current state of the FSM.
  switch(this->fsm_state)
  {
    case 1:
      // Follow the valid target position.
      ROS_DEBUG_STREAM("Following target (FSM state 1)...");
      this->fsmFollowTarget();

      break;
    case 2:
      // Hold the robot stationary and wait for a valid target position.
      ROS_DEBUG_STREAM("Holding and waiting for valid target (FSM state 2)...");
      this->fsmHoldAndWait();

      break;
    case 3:
      // Move the end effector in a pre-determined search pattern to search for the target.
      ROS_DEBUG_STREAM("Searching for target (FSM state 3)...");
      this->fsmSearchForTarget();

      break;
    default:
      // Keep the robot stationary.
      ROS_DEBUG_STREAM("Holding robot stationary (FSM state " << this->fsm_state << ")...");
      this->fsmStationary();
  }
}

/*
 * \brief Move the robot towards the computed valid target position.
 */
void Ur5eGubbiVisualServoingClass::fsmFollowTarget(void)
{
  const Eigen::Isometry3d& fwd_kin = this->getGlobalLinkTransform();
  Eigen::MatrixXd e_curr(6, 1);
  Eigen::MatrixXd e_d_vec(6, 1);
  Eigen::MatrixXd rotation(6, 6);
  int i0;
  ros::Time t_curr = this->tgt_pose_msg.header.stamp;
  std_msgs::Float64MultiArray msg;

  if(t_curr.toSec() > this->fsm_follow_tgt_pose_stamp_prev.toSec())
  {
    ROS_DEBUG_STREAM("Computing error vector...");
    e_curr << this->tgt_pose_msg.pose.position.x, this->tgt_pose_msg.pose.position.y, 0.0, 0.0, 0.0, 0.0;
    ROS_DEBUG_STREAM("e_curr =" << std::endl << eigenMatrixXdToString(e_curr) << "...");

    // Execute the PID controller to determine the end effector velocity.
    ROS_DEBUG_STREAM("Computing proportional, integral, and derivative components of error vector...");
    e_d_vec = (e_curr - this->e_p_vec) / (t_curr.toSec() - this->fsm_follow_tgt_pose_stamp_prev.toSec());
    this->e_p_vec = e_curr;
    this->e_i_vec += e_curr * (t_curr.toSec() - this->fsm_follow_tgt_pose_stamp_prev.toSec());

    for(i0 = 0; i0 < 6; ++i0)
    {
      msg.data.push_back(this->e_p_vec(i0));
    }

    for(i0 = 0; i0 < 6; ++i0)
    {
      msg.data.push_back(this->e_i_vec(i0));
    }

    for(i0 = 0; i0 < 6; ++i0)
    {
      msg.data.push_back(e_d_vec(i0));
    }

    this->error_pub.publish(msg);

    this->fsm_follow_tgt_pose_seq_prev = this->tgt_pose_msg.header.seq;
    this->fsm_follow_tgt_pose_stamp_prev = this->tgt_pose_msg.header.stamp;

    ROS_DEBUG_STREAM("Executing PID controller to compute commanded velocity...");
    this->v_cmd = (this->k_p_mat * this->e_p_vec) + (this->k_i_mat * this->e_i_vec) + (this->k_d_mat * e_d_vec);

    // Convert the commanded end effector velocity to a usable form and provide it to the action server.
    ROS_DEBUG_STREAM("Transforming commanded velocity to appropriate frame...");
    rotation << fwd_kin.rotation(), Eigen::MatrixXd::Zero(3, 3), Eigen::MatrixXd::Zero(3, 3), fwd_kin.rotation();
    this->v_cmd = rotation * this->v_cmd;
  }
  else
  {
    ROS_WARN_STREAM(
      "Current time behind previous message [" << t_curr << " <= " << this->fsm_follow_tgt_pose_stamp_prev << "].");
  }

  ROS_DEBUG_STREAM(
    "Setting spatial cartesian velocity to v_cmd =" << std::endl
                                                    << eigenMatrixXdToString(this->v_cmd) << "...");

  this->setSpatialCartesianVelocity(this->v_cmd);
  this->sendGoalAndWait();
}

void Ur5eGubbiVisualServoingClass::fsmHoldAndWait(void)
{
  // Ensure that the robot is stationary.
  this->fsm_follow_tgt_pose_seq_prev = this->tgt_pose_msg.header.seq;
  this->fsm_follow_tgt_pose_stamp_prev = this->tgt_pose_msg.header.stamp;
  SimpleActionClient::cancelAllGoals();
}

/*
 * \brief Search for the target in a spiral pattern around the previously
 * computed center.
 */
void Ur5eGubbiVisualServoingClass::fsmSearchForTarget(void)
{
  const Eigen::Isometry3d& fwd_kin = this->getGlobalLinkTransform();
  ros::Time t_curr = ros::Time::now();
  double t = t_curr.toSec() - this->search_t_start.toSec();
  Eigen::MatrixXd rotation(6, 6);
  Eigen::MatrixXd v_cmd(6, 1);
  int i0;

  ROS_DEBUG_STREAM("t = " << t);
  v_cmd(0, 0) = (this->search_r_dot * cos(this->search_omega * t))
    - (this->search_r_dot * this->search_omega * t * sin(this->search_omega * t));
  v_cmd(1, 0) = (this->search_r_dot * sin(this->search_omega * t))
    + (this->search_r_dot * this->search_omega * t * cos(this->search_omega * t));

  for(i0 = 2; i0 < v_cmd.rows(); ++i0)
  {
    v_cmd(i0, 0) = 0.0;
  }

  ROS_DEBUG_STREAM("v_cmd =" << std::endl << eigenMatrixXdToString(v_cmd) << "...");

  // Convert the commanded end effector velocity to a usable form and provide it to the action server.
  rotation << fwd_kin.rotation(), Eigen::MatrixXd::Zero(3, 3), Eigen::MatrixXd::Zero(3, 3), fwd_kin.rotation();
  v_cmd = rotation * v_cmd;

  ROS_DEBUG_STREAM("rotation =" << std::endl << eigenMatrixXdToString(rotation));
  ROS_DEBUG_STREAM(
    "Setting spatial cartesian velocity to v_cmd =" << std::endl
                                                    << eigenMatrixXdToString(v_cmd) << "...");

  this->fsm_follow_tgt_pose_seq_prev = this->tgt_pose_msg.header.seq;
  this->fsm_follow_tgt_pose_stamp_prev = this->tgt_pose_msg.header.stamp;
  this->setSpatialCartesianVelocity(v_cmd);
  this->sendGoalAndWait();
}

/*
 * \brief Keep the robot stationary.
 */
void Ur5eGubbiVisualServoingClass::fsmStationary(void)
{
  // Ensure that the robot is not moving, and set the current position as a pivot for future target tracking activities.
  this->fsm_follow_tgt_pose_seq_prev = this->tgt_pose_msg.header.seq;
  this->fsm_follow_tgt_pose_stamp_prev = this->tgt_pose_msg.header.stamp;
  SimpleActionClient::cancelAllGoals();
}

/*
 * \brief Transition checks for the finite state machine
 */
void Ur5eGubbiVisualServoingClass::fsmTransitionCheck(void)
{
  int i0;
  std_msgs::Int16 fsm_state_msg;
  ros::Time t_curr = ros::Time::now();

  ROS_DEBUG_STREAM("Checking for FSM transitions...");

  // Check if a new message has been received.
  if(this->tgt_pose_msg.header.seq > this->fsm_trans_tgt_pose_seq_prev)
  {
    this->fsm_trans_tgt_pose_seq_prev = this->tgt_pose_msg.header.seq;
    this->fsm_trans_tgt_pose_stamp_prev = this->tgt_pose_msg.header.stamp;

    if(tgt_pose_msg.header.frame_id == "probe")
    {
      // A valid target pose has been received, so move towards it.
      ROS_DEBUG_STREAM("Valid target pose received. Setting FSM state to 1 (center)...");
      this->fsm_state = 1;
      this->invalid_pose_counter = 0;
    }
    else if((this->fsm_state == -1 || this->fsm_state == 1 || this->fsm_state == 2) && (this->invalid_pose_counter < 10))
    {
      // The target pose provided is invalid, so wait for a second before entering the search state.
      // NOTE: This is the preferred method for the initial state as well [-1].
      if(this->fsm_state != 2)
      {
        ROS_INFO_STREAM(
          "Invalid target pose received [" << this->invalid_pose_counter << " < " << this->wait_max_count
                                           << "]. Setting FSM state to 2 (wait)...");
        this->fsm_state = 2;

        for(i0 = 0; i0 < 6; ++i0)
        {
          this->e_p_vec(i0) = 0.0;
          this->e_i_vec(i0) = 0.0;
        }
      }
      else
      {
        ROS_INFO_STREAM(
          "Invalid target pose received [" << this->invalid_pose_counter << " < " << this->wait_max_count
                                           << "]. Keeping FSM state at 2 (wait)...");
      }

      this->invalid_pose_counter++;
    }
    else if((this->fsm_state == 2 || this->fsm_state == 3) && (this->invalid_pose_counter < this->search_max_count))
    {
      // The target pose provided is invalid, so search for 5 seconds before returning to the stationary state.
      ROS_INFO_STREAM(
        "Invalid target pose received [10 <= " << this->invalid_pose_counter << " < " << this->search_max_count << "].");
      if(this->fsm_state == 3)
      {
        ROS_INFO("Keeping FSM state at 3 (search)...");
      }
      else
      {
        ROS_INFO("Setting FSM state to 3 (search)...");
        this->fsm_state = 3;
        this->search_t_start = ros::Time::now();
      }

      this->invalid_pose_counter++;
    }
    else if(this->fsm_state >= 0)
    {
      // Although new frames continue to arrive, the target has not been found after 1 second of stationarity and 5
      // seconds of searching. Keep the robot stationary to prevent the end effector from moving off of the surface.
      ROS_INFO_STREAM("Invalid target pose received. Setting FSM state to 0 (stop)...");
      this->fsm_state = 0;
    }
    else
    {
      // This conditional block is separate from the previous `else if` conditional block merely to provide a different
      // message to the user.
      ROS_INFO_STREAM("Invalid target pose received. Keeping FSM state at " << this->fsm_state << "...");
    }
  }
  else if(t_curr.toSec() - this->fsm_trans_tgt_pose_stamp_prev.toSec() > 0.5)
  {
    // Switch the FSM to the stationary state, since the system has not received a new frame in the previous second,
    // indicating a network connectivity issue.
    ROS_WARN_STREAM(
      "No new target message [sequence ID "
      << this->tgt_pose_msg.header.seq << " <= " << this->fsm_trans_tgt_pose_seq_prev << "] received in" << std::endl
      << "[time stamps " << t_curr << " - " << this->fsm_trans_tgt_pose_stamp_prev << " = "
      << (t_curr.toSec() - this->fsm_trans_tgt_pose_stamp_prev.toSec()) << " > 0.5] at least 0.5 s.");

    if(this->fsm_state >= 0)
    {
      ROS_INFO_STREAM("Setting FSM state to 0 (hold)...");
      this->fsm_state = 0;
    }
    else
    {
      // No operation
      ROS_INFO_STREAM("Keeping FSM state at " << this->fsm_state << "...");
    }
  }
  else
  {
    // No operation
    ROS_DEBUG_STREAM(
      "No new target message [sequence ID "
      << this->tgt_pose_msg.header.seq << " <= " << this->fsm_trans_tgt_pose_seq_prev << "] received in" << std::endl
      << "[time stamps " << t_curr << " - " << this->fsm_trans_tgt_pose_stamp_prev << " = "
      << (t_curr.toSec() - this->fsm_trans_tgt_pose_stamp_prev.toSec()) << " <= 0.5] up to 0.5 s. Keeping FSM state at "
      << this->fsm_state << "...");
  }

  fsm_state_msg.data = this->fsm_state;
  this->fsm_state_pub.publish(fsm_state_msg);
}

/*
 * \brief Subscriber to the target pose topic
 */
void Ur5eGubbiVisualServoingClass::targetPoseCallback(geometry_msgs::PoseStamped msg)
{
  this->tgt_pose_msg = msg;
}
