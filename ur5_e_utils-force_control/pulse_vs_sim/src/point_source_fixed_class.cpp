#include "pulse_vs_sim/point_source_fixed_class.hpp"

vs_sim::PointSourceFixed::PointSourceFixed(ros::NodeHandle& nh) : PointSourceBase(nh)
{
  Eigen::Vector3d pt_src_pos_base_vec;

  // Fetch parameters from ROS parameter server. These parameters relate to the control timer period, the velocity
  // amplitude, and velocity frequency (for sinusiodal patterns).
  if(this->nh.hasParam("x_pos"))
  {
    this->nh.getParam("x_pos", pt_src_pos_base_vec(0));
  }
  else
  {
    pt_src_pos_base_vec(0) = 0.0e-3;
  }

  if(this->nh.hasParam("y_pos"))
  {
    this->nh.getParam("y_pos", pt_src_pos_base_vec(1));
  }
  else
  {
    pt_src_pos_base_vec(1) = 5.0e-3;
  }

  if(this->nh.hasParam("z_pos"))
  {
    this->nh.getParam("z_pos", pt_src_pos_base_vec(2));
  }
  else
  {
    pt_src_pos_base_vec(2) = 60.0e-3;
  }

  this->pt_src_pose_base_eig.setIdentity();
  this->pt_src_pose_base_eig.translate(pt_src_pos_base_vec);

  ROS_DEBUG("Point source controller period: %.1f ms", 1e3 * this->ctrl_delta_t);
  ROS_DEBUG(
    "Point source position center   : [%.1f, %.1f, %.1f] mm", 1e3 * pt_src_pos_base_vec(0),
    1e3 * pt_src_pos_base_vec(1), 1e3 * pt_src_pos_base_vec(2));

  // Initialize the timer to propagate the point source movement.
  this->ctrl_timer =
    this->nh.createTimer(ros::Duration(this->ctrl_delta_t), &PointSourceFixed::ctrlTimerCallback, this);
}

void vs_sim::PointSourceFixed::ctrlTimerCallback(const ros::TimerEvent& e)
{
  geometry_msgs::TransformStamped pt_src_pose_base_msg;
  tf::transformEigenToMsg(this->pt_src_pose_base_eig, pt_src_pose_base_msg.transform);
  pt_src_pose_base_msg.header.stamp = ros::Time::now();
  pt_src_pose_base_msg.header.frame_id = this->base_frame_name;
  pt_src_pose_base_msg.child_frame_id = this->pt_src_frame_name;

  ROS_DEBUG(
    "x_t = [%.2f, %.2f, %.2f] mm", 1e3 * pt_src_pose_base_msg.transform.translation.x,
    1e3 * pt_src_pose_base_msg.transform.translation.y, 1e3 * pt_src_pose_base_msg.transform.translation.z);
  this->tf_broadcaster.sendTransform(pt_src_pose_base_msg);

  this->ctrl_iter_id++;
}
