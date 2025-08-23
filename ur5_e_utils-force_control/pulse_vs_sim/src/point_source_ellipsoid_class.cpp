#include "pulse_vs_sim/point_source_ellipsoid_class.hpp"

vs_sim::PointSourceEllipsoid::PointSourceEllipsoid(ros::NodeHandle& nh) : PointSourceBase(nh)
{
  // Fetch parameters from ROS parameter server. These parameters relate to the control timer period, the velocity
  // amplitude, and velocity frequency (for sinusiodal patterns).
  if(this->nh.hasParam("x_pos_center"))
  {
    this->nh.getParam("x_pos_center", this->pt_src_pos_center(0));
  }
  else
  {
    ROS_WARN("Could not find parameter 'x_pos_center', using default value...");
    this->pt_src_pos_center(0) = 8.0e-3;
  }

  if(this->nh.hasParam("y_pos_center"))
  {
    this->nh.getParam("y_pos_center", this->pt_src_pos_center(1));
  }
  else
  {
    ROS_WARN("Could not find parameter 'y_pos_center', using default value...");
    this->pt_src_pos_center(1) = 4.0e-3;
  }

  if(this->nh.hasParam("z_pos_center"))
  {
    this->nh.getParam("z_pos_center", this->pt_src_pos_center(2));
  }
  else
  {
    ROS_WARN("Could not find parameter 'z_pos_center', using default value...");
    this->pt_src_pos_center(2) = 15.0e-3;
  }

  if(this->nh.hasParam("x_pos_axis"))
  {
    this->nh.getParam("x_pos_axis", this->pt_src_pos_semi_axes(0));
  }
  else
  {
    ROS_WARN("Could not find parameter 'x_pos_axis', using default value...");
    this->pt_src_pos_semi_axes(0) = 8.0e-3;
  }

  if(this->nh.hasParam("y_pos_axis"))
  {
    this->nh.getParam("y_pos_axis", this->pt_src_pos_semi_axes(1));
  }
  else
  {
    ROS_WARN("Could not find parameter 'y_pos_axis', using default value...");
    this->pt_src_pos_semi_axes(1) = 4.0e-3;
  }

  if(this->nh.hasParam("z_pos_axis"))
  {
    this->nh.getParam("z_pos_axis", this->pt_src_pos_semi_axes(2));
  }
  else
  {
    ROS_WARN("Could not find parameter 'z_pos_axis', using default value...");
    this->pt_src_pos_semi_axes(2) = 15.0e-3;
  }

  if(this->nh.hasParam("x_pos_freq"))
  {
    this->nh.getParam("x_pos_freq", this->pt_src_pos_freq(0));
  }
  else
  {
    ROS_WARN("Could not find parameter 'x_pos_freq', using default value...");
    this->pt_src_pos_freq(0) = 3.0 / 8.0;
  }

  if(this->nh.hasParam("y_pos_freq"))
  {
    this->nh.getParam("y_pos_freq", this->pt_src_pos_freq(1));
  }
  else
  {
    ROS_WARN("Could not find parameter 'y_pos_freq', using default value...");
    this->pt_src_pos_freq(1) = 2.0 / 8.0;
  }

  if(this->nh.hasParam("z_pos_freq"))
  {
    this->nh.getParam("z_pos_freq", this->pt_src_pos_freq(2));
  }
  else
  {
    ROS_WARN("Could not find parameter 'y_pos_freq', using default value...");
    this->pt_src_pos_freq(2) = 1.0 / 8.0;
  }

  if(this->nh.hasParam("add_process_noise"))
  {
    this->nh.getParam("add_process_noise", this->add_process_noise);
  }
  else
  {
    ROS_WARN("Could not find parameter 'add_process_noise', using default value...");
    this->add_process_noise = true;
  }

  this->process_noise = new std::normal_distribution<double>(0.0, 5e-5);

  ROS_INFO("Point source controller period: %.1f ms", 1e3 * this->ctrl_delta_t);
  ROS_INFO(
    "Point source position center   : [%.1f, %.1f, %.1f] mm", 1e3 * this->pt_src_pos_center(0),
    1e3 * this->pt_src_pos_center(1), 1e3 * this->pt_src_pos_center(2));
  ROS_INFO(
    "Point source position amplitude: [%.1f, %.1f, %.1f] mm", 1e3 * this->pt_src_pos_semi_axes(0),
    1e3 * this->pt_src_pos_semi_axes(1), 1e3 * this->pt_src_pos_semi_axes(2));
  ROS_INFO(
    "Point source position frequency: [%.1f, %.1f, %.1f] Hz", this->pt_src_pos_freq(0) / (2.0 * M_PI),
    this->pt_src_pos_freq(1) / (2.0 * M_PI), this->pt_src_pos_freq(2) / (2.0 * M_PI));

  // Initialize the timer to propagate the point source movement.
  this->ctrl_timer =
    this->nh.createTimer(ros::Duration(this->ctrl_delta_t), &PointSourceEllipsoid::ctrlTimerCallback, this);
}

void vs_sim::PointSourceEllipsoid::ctrlTimerCallback(const ros::TimerEvent& e)
{
  // Current time
  double t_val = ros::Time::now().toSec();

  // Commanded velocity of point source
  Eigen::Vector3d process_noise_vec;
  int i0;
  double phase_shift;

  Eigen::Vector3d pt_src_pos_base_vec;
  geometry_msgs::TransformStamped pt_src_pose_base_msg;

  for(i0 = 0; i0 < 3; i0++)
  {
    // Phase shift the x-dimension so we get circular paths in the xz- and xy-planes.
    phase_shift = (i0 == 0) * M_PI / 2.0;

    pt_src_pos_base_vec(i0) = this->pt_src_pos_center(i0)
      + (this->pt_src_pos_semi_axes(i0)
         * sin((this->pt_src_pos_freq(i0) * this->ctrl_iter_id * this->ctrl_delta_t) + phase_shift));

    // Brackets required to use `()` operator (not function) with pointer `this->process_noise`.
    process_noise_vec(i0) = (*(this->process_noise))(this->rand_num_gen);
  }

  // Add process noise to point source location if required by the corresponding flag.
  if(this->add_process_noise)
  {
    pt_src_pos_base_vec += process_noise_vec;
  }
  else
  {
    // No operation
  }

  this->pt_src_pose_base_eig.setIdentity();
  this->pt_src_pose_base_eig.translate(pt_src_pos_base_vec);
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
