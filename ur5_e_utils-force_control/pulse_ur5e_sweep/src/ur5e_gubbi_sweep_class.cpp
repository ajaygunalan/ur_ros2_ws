#include "pulse_ur5e_sweep/ur5e_gubbi_sweep_class.hpp"

/*
 * \brief Constructor for Ur5eGubbiSweepClass
 */
Ur5eGubbiSweepClass::Ur5eGubbiSweepClass(
  ros::NodeHandle& nh, std::string action_server_name, std::string ee_name,
  double delta_t, double ee_vel_max, double k_p, double k_i, double k_d,
  double joint_omega_max, double joint_alpha_max, double t_err_max,
  int n_steps_max, std::string robot_ns)
  : Ur5ePosControlClass(
    nh, action_server_name, ee_name, delta_t, ee_vel_max, k_p, k_i, k_d,
    joint_omega_max, joint_alpha_max, t_err_max, n_steps_max, robot_ns)
  , img_acquired(false)
  , new_user_input(false)
  , elev_step_size(5e-3)
  , lat_step_size(5e-3)
  , num_elev_steps(3)
  , num_lat_steps(3)
  , pose_counter(0)
  , fsm_state(FSM_SELECT_INITIAL_POSE)
{
  this->num_poses = this->num_elev_steps * this->num_lat_steps;
  this->fsm_pub = this->nh.advertise<std_msgs::Int8>("/scan/fsm_state", 1000);
  this->fsm_timer = this->nh.createTimer(
    ros::Duration(1.0), &Ur5eGubbiSweepClass::fsmTimerCallback, this);
  this->img_acq_sub = this->nh.subscribe(
    "/verasonics/img_acq", 100,
    &Ur5eGubbiSweepClass::imgAcquisitionSubscriberCallback, this);
  this->user_input_sub = this->nh.subscribe(
    "/key", 100, &Ur5eGubbiSweepClass::userInputSubscriberCallback, this);
}

/*
 * \brief Generate the grid of imaging poses from the initial user-set pose
 */
void Ur5eGubbiSweepClass::fsmGeneratePoseGrid(void)
{
  ROS_INFO_STREAM("Generating grid of imaging poses...");
  const Eigen::Isometry3d fwd_kin = this->getGlobalLinkTransform();
  Eigen::Isometry3d sweep_initial_pose_tmp = fwd_kin;

  this->sweep_initial_pose = fwd_kin;
  this->next_sweep_wait_pose =
    sweep_initial_pose_tmp.translate(Eigen::Vector3d(0, 0, -5e-2));
  this->pose_counter = 0;
  this->fsm_state = FSM_MOVE_ROBOT;
}

/*
 * \brief Move the robot to the pose specified by the current system state
 */
void Ur5eGubbiSweepClass::fsmMoveRobot(void)
{
  if(this->pose_counter < this->num_poses)
  {
    Eigen::Isometry3d sweep_initial_pose_tmp = this->sweep_initial_pose;
    Eigen::Vector3d x_curr;
    int elev_step_count = this->pose_counter / this->num_lat_steps;
    int lat_step_count = this->pose_counter % this->num_lat_steps;

    if(elev_step_count % 2 == 1)
    {
      lat_step_count = this->num_lat_steps - 1 - lat_step_count;
    }
    else
    {
      // No operation
    }

    x_curr(0) = (lat_step_count * this->lat_step_size);
    x_curr(1) = (elev_step_count * this->elev_step_size);
    x_curr(2) = 0.0;

    this->sweep_img_pose_curr = sweep_initial_pose_tmp.translate(x_curr);
    ROS_INFO_STREAM(
      "[" << this->pose_counter << "/" << this->num_poses
          << "] Moving robot to imaging pose...");
  }
  else
  {
    ROS_INFO_STREAM(
      "["
      << this->pose_counter << "/" << this->num_poses
      << "] Sweep complete. Raising robot to wait for start of next sweep...");
    this->sweep_img_pose_curr = this->next_sweep_wait_pose;
  }

  // TODO: Move the robot here.
  ROS_DEBUG_STREAM(
    "[" << this->pose_counter << "/" << this->num_poses
        << "] Moving robot to pose:" << std::endl
        << eigenIsometry3dToString(this->sweep_img_pose_curr));
  this->setSpatialCartesianPose(this->sweep_img_pose_curr);

  ROS_DEBUG_STREAM(
    "[" << this->pose_counter << "/" << this->num_poses << "] Updating FSM...");

  if(this->pose_counter < this->num_poses)
  {
    this->pose_counter++;
    this->fsm_state = FSM_WAIT_FOR_IMAGES;
  }
  else
  {
    this->fsm_state = FSM_WAIT_FOR_NEXT_SWEEP;
  }
}

/*
 * \brief Wait for the user to move the robot to the desired initial pose
 */
void Ur5eGubbiSweepClass::fsmSelectInitialPose(void)
{
  ROS_INFO_STREAM("Move robot to desired initial pose and press <enter> in "
                  "console containing keyboard node.");

  if(this->new_user_input)
  {
    if(this->user_input_data == 10)
    {
      this->fsm_state = FSM_GENERATE_POSE_GRID;
    }
    else
    {
      ROS_ERROR_STREAM(
        "Unknown user input '" << this->user_input_data
                               << "'. Please press <enter> after moving "
                                  "the robot to the desired initial pose.");
    }

    this->new_user_input = false;
    this->user_input_data = 0;
  }
  else
  {
    // No operation
  }
}

/*
 * \brief Callback function for FSM execution
 */
void Ur5eGubbiSweepClass::fsmTimerCallback(const ros::TimerEvent& e)
{
  std_msgs::Int8 msg;

  msg.data = this->fsm_state;
  this->fsm_pub.publish(msg);

  switch(this->fsm_state)
  {
    case FSM_SELECT_INITIAL_POSE:
      this->fsmSelectInitialPose();

      break;
    case FSM_GENERATE_POSE_GRID:
      this->fsmGeneratePoseGrid();

      break;
    case FSM_MOVE_ROBOT:
      this->fsmMoveRobot();

      break;
    case FSM_WAIT_FOR_IMAGES:
      this->fsmWaitForImages();

      break;
    case FSM_WAIT_FOR_NEXT_SWEEP:
      this->fsmWaitForNextSweep();

      break;
  }
}

void Ur5eGubbiSweepClass::fsmWaitForImages(void)
{
  ROS_INFO_STREAM(
    "[" << this->pose_counter << "/" << this->num_poses
        << "] Waiting for images to be saved on ultrasound scanner...");
  this->fsm_state = FSM_MOVE_ROBOT;
}

void Ur5eGubbiSweepClass::fsmWaitForNextSweep(void)
{
  ROS_INFO_STREAM("Waiting for beginning of next sweep...");

  if((this->new_user_input == true) && (this->user_input_data == 10))
  {
    this->new_user_input = false;
    this->user_input_data = 0;
    this->pose_counter = 0;
    this->fsm_state = FSM_MOVE_ROBOT;
  }
  else
  {
    // No operation
  }
}

void Ur5eGubbiSweepClass::imgAcquisitionSubscriberCallback(
  const std_msgs::Int8 msg)
{
  this->img_acquired =
    ((this->fsm_state == FSM_WAIT_FOR_IMAGES) && (msg.data > 0));
}

/*
 * \brief Callback function for user input subscription
 */
void Ur5eGubbiSweepClass::userInputSubscriberCallback(const std_msgs::Int8 msg)
{
  ROS_INFO_STREAM("Received user input '" << msg.data << "'.");
  this->new_user_input = true;
  this->user_input_data = msg.data;
}
