#include "pulse_force_control/gubbi_standalone_force_control_class.hpp"

static const std::string PLANNING_GROUP = "manipulator";

/*
 * \brief Constructor for GubbiStandaloneForceControlClass
 */
pfc::GubbiStandaloneForceControlClass::GubbiStandaloneForceControlClass(
  ros::NodeHandle& nh, std::string base_frame_name,
  std::string probe_frame_name, std::string tool_frame_name)
  : pfc::GubbiForceControlClass(
    nh, base_frame_name, probe_frame_name, tool_frame_name)
{
  // Terminate the timer from the base class to assign it to the callback function of this class.
  this->fsm_timer.stop();

  // Initialize the move group interface required for motion control.
  this->mgi_obj =
    new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
  this->mgi_obj->startStateMonitor();
  this->mgi_obj->setMaxVelocityScalingFactor(0.1);
  this->mgi_obj->setMaxAccelerationScalingFactor(0.7);
  this->mgi_obj->setPlannerId("LIN");
  this->jmg_obj =
    this->mgi_obj->getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  this->joint_names_vec.push_back("shoulder_pan_joint");
  this->joint_names_vec.push_back("shoulder_lift_joint");
  this->joint_names_vec.push_back("elbow_joint");
  this->joint_names_vec.push_back("wrist_1_joint");
  this->joint_names_vec.push_back("wrist_2_joint");
  this->joint_names_vec.push_back("wrist_3_joint");

  ROS_DEBUG(
    "Initializing timer for GubbiStandaloneForceControlClass with period %.2f "
    "ms...",
    1e3 * this->fsm_period);
  this->fsm_timer = this->nh.createTimer(
    ros::Duration(this->fsm_period),
    &GubbiStandaloneForceControlClass::forceControlLoopTimerCallback, this);
}

/*
 * \brief Callback function for control loop timer
 */
void pfc::GubbiStandaloneForceControlClass::forceControlLoopTimerCallback(
  const ros::TimerEvent& e)
{
  ROS_DEBUG("GubbiStandaloneForceControlClass::forceControlLoopTimerCallback");

  moveit::core::MoveItErrorCode move_err;
  tf::StampedTransform st_p_t;
  Eigen::Isometry3d eig_p_t;

  // Execute the timer callback function of the parent class to generate the
  // updated tool frame.
  pfc::GubbiForceControlClass::forceControlLoopTimerCallback(e);

  try
  {
    this->tf_listener.lookupTransform(
      this->probe_frame_name, this->tool_frame_name, ros::Time(0), st_p_t);
    tf::transformTFToEigen(st_p_t, eig_p_t);
    move_err = this->moveRobotProbeFrame(eig_p_t);

    if(move_err == moveit::core::MoveItErrorCode::SUCCESS)
    {
      ROS_DEBUG("Force controlled motion successful.");
    }
    else
    {
      ROS_WARN_STREAM(
        "Error performing force controlled motion: "
        << move_err << ". Trying again next cycle...");
      ROS_ERROR("Shutting down node for ease of debugging...");
      ros::shutdown();
    }
  }
  catch(tf::TransformException& ex)
  {
    ROS_ERROR("%s\nTrying again next cycle...", ex.what());
  }
}
