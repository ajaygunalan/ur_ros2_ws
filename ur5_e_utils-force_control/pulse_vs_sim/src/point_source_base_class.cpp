#include "pulse_vs_sim/point_source_base_class.hpp"

vs_sim::PointSourceBase::PointSourceBase(ros::NodeHandle& nh) : nh(nh), ctrl_iter_id(1.0)
{
  Eigen::Isometry3d probe_cmd_pose_base_eig;
  geometry_msgs::TransformStamped probe_cmd_pose_base_msg;

  // Fetch parameters from ROS parameter server. These parameters relate to the control timer period, the velocity
  // amplitude, and velocity frequency (for sinusiodal patterns).
  if(this->nh.hasParam("delta_t"))
  {
    this->nh.getParam("delta_t", this->ctrl_delta_t);
  }
  else
  {
    this->ctrl_delta_t = 0.1;
  }

  if(this->nh.hasParam("base_frame_name"))
  {
    this->nh.getParam("base_frame_name", this->base_frame_name);
  }
  else
  {
    this->base_frame_name = "base_link";
  }

  if(this->nh.hasParam("pt_src_frame_name"))
  {
    this->nh.getParam("pt_src_frame_name", this->pt_src_frame_name);
  }
  else
  {
    this->pt_src_frame_name = "pt_src";
  }

  if(this->nh.hasParam("probe_frame_name"))
  {
    this->nh.getParam("probe_frame_name", this->probe_frame_name);
  }
  else
  {
    this->probe_frame_name = "p42v_link1";
  }

  // Publish the initial transform between the probe and base frames for ease of simulations.
  probe_cmd_pose_base_eig.setIdentity();
  tf::transformEigenToMsg(probe_cmd_pose_base_eig, probe_cmd_pose_base_msg.transform);
  probe_cmd_pose_base_msg.header.stamp = ros::Time::now();
  probe_cmd_pose_base_msg.header.frame_id = this->base_frame_name;
  probe_cmd_pose_base_msg.child_frame_id = this->probe_frame_name;
  // this->tf_broadcaster.sendTransform(probe_cmd_pose_base_msg);
}
