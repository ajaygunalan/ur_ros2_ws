#include "pulse_force_control/tool_frame_force_control_class.hpp"

/*
 * \brief Construct an object of the type ToolFrameForceControlClass
 *
 * TODO: Fix hard-coding. Load these from the ROS parameter server or
 * command-line arguments.
 */
pfc::ToolFrameForceControlClass::ToolFrameForceControlClass(
  ros::NodeHandle& nh, std::string base_frame_name,
  std::string probe_frame_name, std::string tool_frame_name)
  : mgi::Ur5eMoveGroupInterfaceClass(nh, probe_frame_name)
  , base_frame_name(base_frame_name)
  , tool_frame_name(tool_frame_name)
  , z_trans_k_p(1e-4)
  , ee_trans_sat_thresh(1e-3)
  , f_z_deadband(1.0)
{
  double contact_force;

  // Fetch the desired contact force if provided as a parameter.
  this->ft_cmd.setZero();

  if(this->nh.getParam("contact_force", contact_force))
  {
    this->ft_cmd(2) = -contact_force;
  }
  else
  {
    ROS_WARN(
      "Could not find parameter 'contact_force', using default value...");
    this->ft_cmd(2) = -2.0;
  }

  ROS_DEBUG("Desired contact force into probe face: %.2f N", -this->ft_cmd(2));

  if(!this->nh.getParam("fsm_period", this->fsm_period))
  {
    ROS_WARN("Could not find parameter 'fsm_period', using default value...");
    this->fsm_period = 0.05;
  }

  ROS_DEBUG("Force control loop time period: %.2f ms", 1e3 * this->fsm_period);

  // Compute the maximum rotational velocity along the x- and y-dimensions of
  // the end effector.
  if(!this->nh.getParam("max_ee_rot_vel", this->max_ee_rot_vel))
  {
    ROS_WARN(
      "Could not find parameter 'max_ee_rot_vel', using default value...");
    this->max_ee_rot_vel = 1.0 * M_PI / 180.0;
  }

  ROS_DEBUG(
    "Maximum end effector rotational velocity: %.2f deg/s",
    this->max_ee_rot_vel * 180.0 / M_PI);

  // Compute the rotational saturation threshold using the FSM time period.
  this->ee_rot_sat_thresh = this->max_ee_rot_vel * this->fsm_period;
  ROS_DEBUG(
    "End effector rotational saturation threshold: %.2f deg",
    this->ee_rot_sat_thresh);

  // Compute the maximum translational velocity along the z-dimension of the
  // end effector.
  if(!this->nh.getParam("max_ee_trans_vel", this->max_ee_trans_vel))
  {
    ROS_WARN(
      "Could not find parameter 'max_ee_trans_vel', using default value...");
    this->max_ee_trans_vel = 25e-3;
  }

  ROS_DEBUG(
    "Maximum end effector translational velocity: %.2f mm/s",
    1e3 * this->max_ee_trans_vel);

  // Compute the translational saturation threshold using the FSM time period.
  this->ee_trans_sat_thresh = this->max_ee_trans_vel * this->fsm_period;
  ROS_DEBUG(
    "End effector translational saturation threshold: %.2f mm",
    this->ee_trans_sat_thresh);

  // Initialize the force control loop execution timer.
  this->force_control_timer = new et::PulseExecutionTimer(nh);
}

/*
 * \brief Callback function for subscriber to processed F/T sensor readings
 */
void pfc::ToolFrameForceControlClass::netftProcSensorSubscriberCallback(
  const geometry_msgs::WrenchStamped& msg)
{
  tf::wrenchMsgToEigen(msg.wrench, this->ft_curr);

  // The computation of the local normal depends on this error computation with
  // positive `ft_curr`, so if a change in sign is required for proper control,
  // it must be applied to the gain coefficients.
  this->ft_err = this->ft_curr - this->ft_cmd;
  // ROS_DEBUG_STREAM(
  // ""
  // "Force-torque error: ["
  // << this->ft_err.transpose() << "]" << std::endl
  // << "= [" << this->ft_curr.transpose() << "]" << std::endl
  // << "- [" << this->ft_cmd.transpose() << "]");
}
