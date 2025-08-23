#include "pulse_force_control/simple_z_control_class.hpp"

SimpleZControlClass::SimpleZControlClass(
  ros::NodeHandle& nh, std::string action_server_name, std::string ee_name, double delta_t, double joint_omega_max)
  : Ur5eBaseClass(nh, action_server_name, ee_name, delta_t, joint_omega_max)
{
  if(this->nh.hasParam("f_z_goal"))
  {
    this->nh.getParam("f_z_goal", this->f_z_goal);
  }
  else
  {
    this->f_z_goal = 0.0;
    ROS_WARN("Could not find parameter 'f_z_goal', using default value of %.2f N...", this->f_z_goal);
  }

  if(this->nh.hasParam("f_z_thresh"))
  {
    this->nh.getParam("f_z_thresh", this->f_z_thresh);
  }
  else
  {
    this->f_z_thresh = 0.5;
    ROS_WARN("Could not find parameter 'f_z_thresh', using default value of %.2f N...", this->f_z_thresh);
  }

  ROS_DEBUG("Initializing force control timer...");
  this->force_control_timer =
    this->nh.createTimer(ros::Duration(delta_t), &SimpleZControlClass::forceControlTimerCallback, this);

  this->force_sensor_sub =
    this->nh.subscribe("/netft/proc_probe", 1, &SimpleZControlClass::forceSensorSubscriberCallback, this);
}

void SimpleZControlClass::forceControlTimerCallback(const ros::TimerEvent& e)
{
  Eigen::MatrixXd v_cmd = Eigen::MatrixXd::Zero(6, 1);

  if(this->f_z_curr > this->f_z_goal + this->f_z_thresh)
  {
    v_cmd(2) = 0.01 * (this->f_z_curr - this->f_z_goal);
    ROS_DEBUG(
      "Force: %.2f > %.2f + %.2f -> going down by %.2f mm/s...", this->f_z_curr, this->f_z_goal, this->f_z_thresh,
      1.0e3 * v_cmd(2));
  }
  else if(this->f_z_curr < this->f_z_goal - this->f_z_thresh)
  {
    v_cmd(2) = 0.01 * (this->f_z_curr - this->f_z_goal);
    ROS_DEBUG(
      "Force: %.2f < %.2f - %.2f -> going up by %.2f mm/s...", this->f_z_curr, this->f_z_goal, this->f_z_thresh,
      1.0e3 * v_cmd(2));
  }

  this->setBodyCartesianVelocity(v_cmd);
  this->sendGoalAndWait();
}

void SimpleZControlClass::forceSensorSubscriberCallback(geometry_msgs::WrenchStamped msg)
{
  this->f_z_curr = msg.wrench.force.z;

  // ROS_DEBUG("Current contact force = %.6f N", this->f_z_curr);
}
