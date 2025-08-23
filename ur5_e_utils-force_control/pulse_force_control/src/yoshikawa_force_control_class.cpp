#include "pulse_force_control/yoshikawa_force_control_class.hpp"

/*
 * \brief Constructor for YoshikawaForceControlClass
 *
 * TODO: Fix hard-coding. Load these from the ROS parameter server or
 * command-line arguments.
 */
pfc::YoshikawaForceControlClass::YoshikawaForceControlClass(
  ros::NodeHandle& nh, std::string base_frame_name,
  std::string probe_frame_name, std::string tool_frame_name)
  : pfc::ToolFrameForceControlClass(
    nh, base_frame_name, probe_frame_name, tool_frame_name)
{
  // Fetch the deadband value for the forces in the translational x- and y-dimensions.
  if(!this->nh.getParam("force_xy_deadband", this->f_xy_deadband))
  {
    ROS_WARN(
      "Could not find parameter 'force_xy_deadband', using default value...");
    this->f_xy_deadband = 0.1;
  }

  ROS_DEBUG(
    "Deadband for forces in translational x- and y-dimensions: %.2f N",
    this->f_xy_deadband);

  ROS_DEBUG(
    "Initializing timer for YoshikawaForceControlClass with period %.2f "
    "ms...",
    1e3 * this->fsm_period);
  this->fsm_timer = this->nh.createTimer(
    ros::Duration(this->fsm_period),
    &pfc::YoshikawaForceControlClass::forceControlLoopTimerCallback, this);

  // https://answers.ros.org/question/190920/how-can-i-subscribe-to-a-topic-using-a-parent-class-function-as-the-callback/
  this->ft_proc_tool_sub = this->nh.subscribe(
    "/netft/proc_probe", 1,
    &pfc::ToolFrameForceControlClass::netftProcSensorSubscriberCallback,
    dynamic_cast<pfc::ToolFrameForceControlClass*>(this));
}

/*
 * \brief Estimate the local normal of the unknown constraint surface
 * as described in Section III of Yoshikawa et al. [1]
 */
Eigen::Quaterniond pfc::YoshikawaForceControlClass::estimateLocalNormal(void)
{
  ROS_DEBUG("YoshikawaForceControlClass::estimateLocalNormal");
  // Compute `e1-tilde` in Equ. (15) in [1] in the probe frame `P`. To do so, we
  // must first compute `delta-r`. This is accomplished by computing the
  // incremental movement of the probe in the robot base frame `B`, and
  // transforming that vector into the current probe frame `P`.
  Eigen::Isometry3d eig_b_p;
  Eigen::Vector3d delta_r_b;
  Eigen::Vector3d delta_r_p;
  Eigen::Vector3d e1_tilde_p;
  Eigen::Vector3d t_prev_b_p;
  geometry_msgs::PoseStamped geo_msg_b_p;
  tf::StampedTransform st_b_p;

  // ROS_DEBUG("Computing `e1-tilde` in Equ. (15)...");
  t_prev_b_p = this->t_curr_b_p;

  geo_msg_b_p = this->mgi_obj->getCurrentPose();
  tf::poseMsgToEigen(geo_msg_b_p.pose, eig_b_p);
  this->t_curr_b_p = eig_b_p.translation();
  delta_r_b = this->t_curr_b_p - t_prev_b_p;

  ROS_DEBUG(
    "`delta-r_b` = [% .6f, % .6f, % .6f] [norm %.6f]", delta_r_b(0),
    delta_r_b(1), delta_r_b(2), delta_r_b.norm());
  // ROS_DEBUG_STREAM(
  // ""
  // << "`delta-r_b` = [" << delta_r_b.transpose() << "] [norm "
  // << delta_r_b.norm() << "]");

  Eigen::Matrix3d rot_p_b_mat = eig_b_p.rotation().matrix().transpose();
  ROS_DEBUG(
    "`rot_p_b` = [\n"
    "% .6f, % .6f, % .6f\n"
    "% .6f, % .6f, % .6f\n"
    "% .6f, % .6f, % .6f]",
    rot_p_b_mat(0, 0), rot_p_b_mat(0, 1), rot_p_b_mat(0, 2), rot_p_b_mat(1, 0),
    rot_p_b_mat(1, 1), rot_p_b_mat(1, 2), rot_p_b_mat(2, 0), rot_p_b_mat(2, 1),
    rot_p_b_mat(2, 2));
  // ROS_DEBUG_STREAM(
  // ""
  // << "`rot_p_b` = [" << std::endl
  // << eig_b_p.rotation().matrix().transpose() << "]");

  delta_r_p = eig_b_p.rotation().matrix().transpose() * delta_r_b;
  ROS_DEBUG(
    "`delta-r_p` = [% .6f, % .6f, % .6f] [norm %.6f]", delta_r_p(0),
    delta_r_p(1), delta_r_p(2), delta_r_p.norm());

  // We are not interested in motion along the z-axis of the probe.
  delta_r_p(2) = 0.0;
  ROS_DEBUG(
    "`delta-r_p` (after z-axis zeroing) = [% .6f, % .6f, % .6f] [norm %.6f]",
    delta_r_p(0), delta_r_p(1), delta_r_p(2), delta_r_p.norm());

  // For extremely small robot motion, hard-code a value along the lateral axis
  // of the probe.
  if(delta_r_p.norm() < 1e-4)
  {
    delta_r_p.setZero();
    delta_r_p(0) = 1e-4;
    ROS_DEBUG(
      "`delta-r_p` (corrected for small norm) = [% .6f, % .6f, % .6f] [norm "
      "%.6f]",
      delta_r_p(0), delta_r_p(1), delta_r_p(2), delta_r_p.norm());
  }

  // ROS_DEBUG_STREAM("`delta-r_p` = [" << delta_r_p.transpose() << "]");

  e1_tilde_p = delta_r_p / delta_r_p.norm();
  ROS_DEBUG(
    "`e1-tilde` = [% .6f, % .6f, % .6f]", e1_tilde_p(0), e1_tilde_p(1),
    e1_tilde_p(2));
  // ROS_DEBUG_STREAM("`e1-tilde` = [" << e1_tilde_p.transpose() << "]");

  // Apply dead-bands and saturation filters to the current force-torque values
  // prior to the remaining computations for the local normal estimation.
  ROS_DEBUG(
    "`ft_curr` = [% .6f, % .6f, % .6f, % .6f, % .6f, % .6f]", this->ft_curr(0),
    this->ft_curr(1), this->ft_curr(2), this->ft_curr(3), this->ft_curr(4),
    this->ft_curr(5));
  // ROS_DEBUG_STREAM("`ft_curr` = [" << this->ft_curr.transpose() << "]");

  Eigen::Vector3d ft_curr_proc = this->ft_curr.block(0, 0, 3, 1);
  int i0;

  // ROS_DEBUG(
  // "Applying dead-band of %.2f N in translation x- and y-dimensions...",
  // this->f_xy_deadband);

  for(i0 = 0; i0 < 2; i0++)
  {
    if(ft_curr_proc(i0) > this->f_xy_deadband)
    {
      ft_curr_proc(i0) -= this->f_xy_deadband;
    }
    else if(ft_curr_proc(i0) < -this->f_xy_deadband)
    {
      ft_curr_proc(i0) += this->f_xy_deadband;
    }
    else
    {
      ft_curr_proc(i0) = 0.0;
    }
  }

  ROS_DEBUG(
    "`ft_curr_proc` = [% .6f, % .6f, % .6f] (post xy-deadband of % .2f N)",
    ft_curr_proc(0), ft_curr_proc(1), ft_curr_proc(2), this->f_xy_deadband);
  // ROS_DEBUG_STREAM("`ft_curr_proc` = [" << ft_curr_proc.transpose() << "]");

  // Compute `f_r-tilde` in Equ. (17) in [1] (i.e., the residual contact force
  // after subtracting the frictional force component) in the probe frame `P`.
  double ft_curr_t_e1_tilde_p = 0.0;
  Eigen::Vector3d f_r_tilde_p;

  // ROS_DEBUG("Computing `f_r-tilde` in Equ. (17)...");

  for(i0 = 0; i0 < 3; i0++)
  {
    ft_curr_t_e1_tilde_p += ft_curr_proc(i0) * e1_tilde_p(i0);
  }

  for(i0 = 0; i0 < 3; i0++)
  {
    f_r_tilde_p(i0) =
      ft_curr_proc(i0) - (ft_curr_t_e1_tilde_p * e1_tilde_p(i0));
  }

  ROS_DEBUG(
    "`f_r-tilde` = [% .6f, % .6f, % .6f]", f_r_tilde_p(0), f_r_tilde_p(1),
    f_r_tilde_p(2));
  // ROS_DEBUG_STREAM("`f_r-tilde` = [" << f_r_tilde_p.transpose() << "]");

  // Compute `e3-hat` in Equ. (16) in [1] in the probe frame `P`. We use `est`
  // instead of `hat` for `e1-hat`, `e2-hat`, and `e3-hat` to avoid confusion
  // with the hat operator used for vector cross products.
  Eigen::Vector3d e3_est_p;

  // ROS_DEBUG("Computing `e3-hat` in Equ. (16)...");
  e3_est_p = -f_r_tilde_p / f_r_tilde_p.norm();
  ROS_DEBUG(
    "`e3-hat` = [% .6f, % .6f, % .6f]", e3_est_p(0), e3_est_p(1), e3_est_p(2));
  // ROS_DEBUG_STREAM("`e3-hat` = [" << e3_est_p.transpose() << "]");

  // Set `w` in Equ. (19) in [1]. As far as possible, we wish to move while
  // keeping the imaging plane constant, minimizing motion along the elevation
  // dimension of the probe. Therefore, `w` is set to the y-dimension in the
  // probe frame `P`.
  Eigen::Vector3d w_p;

  // ROS_DEBUG("Setting `w` in Equ. (19)...");
  w_p.setZero();
  w_p(1) = 1.0;

  // Compute `e1-hat` in Equ. (18) in [1] in the probe frame `P`.
  Eigen::Vector3d e1_est_p;

  // ROS_DEBUG("Computing `e1-hat` in Equ. (18)...");
  e1_est_p = hatOperator(w_p) * e3_est_p;
  e1_est_p = e1_est_p / e1_est_p.norm();
  ROS_DEBUG(
    "`e1-hat` = [% .6f, % .6f, % .6f]", e1_est_p(0), e1_est_p(1), e1_est_p(2));
  // ROS_DEBUG_STREAM("`e1-hat` = [" << e1_est_p.transpose() << "]");

  // Compute `e2-hat` in Equ. (20) in [1] in the probe frame `P`.
  Eigen::Vector3d e2_est_p;

  // ROS_DEBUG("Computing `e2-hat` in Equ. (20)...");
  e2_est_p = hatOperator(e3_est_p) * e1_est_p;
  ROS_DEBUG(
    "`e2-hat` = [% .6f, % .6f, % .6f]", e2_est_p(0), e2_est_p(1), e2_est_p(2));
  // ROS_DEBUG_STREAM("`e2-hat` = [" << e2_est_p.transpose() << "]");

  // Populate and print the rotation matrix from the newly compute task
  // frame `T` to the probe frame `P`.
  Eigen::Matrix3d rot_p_t;
  Eigen::Quaterniond q_p_t;

  // ROS_DEBUG(
  // "Computing rotation matrix from task frame `T` to probe frame `P`...");
  rot_p_t.block(0, 0, 3, 1) = e1_est_p;
  rot_p_t.block(0, 1, 3, 1) = e2_est_p;
  rot_p_t.block(0, 2, 3, 1) = e3_est_p;

  ROS_DEBUG(
    "`rot_p_t` = [\n"
    "% .6f, % .6f, % .6f\n"
    "% .6f, % .6f, % .6f\n"
    "% .6f, % .6f, % .6f]",
    rot_p_t(0, 0), rot_p_t(0, 1), rot_p_t(0, 2), rot_p_t(1, 0), rot_p_t(1, 1),
    rot_p_t(1, 2), rot_p_t(2, 0), rot_p_t(2, 1), rot_p_t(2, 2));
  // ROS_DEBUG_STREAM("`rot_p_t` = [" << std::endl << rot_p_t << "]");
  q_p_t = rot_p_t;
  q_p_t.normalize();
  ROS_DEBUG(
    "`q_p_t` = [% .6f, % .6f, % .6f, % .6f]", q_p_t.x(), q_p_t.y(), q_p_t.z(),
    q_p_t.w());
  // ROS_DEBUG_STREAM(
  // ""
  // << "`q_p_t` = [" << q_p_t.x() << ", " << q_p_t.y() << ", " << q_p_t.z()
  // << ", " << q_p_t.w() << "]");

  return q_p_t;
}

/*
 * \brief Callback function for force control FSM timer
 *
 * TODO: Fetch the transform from the sensor frame to the end effector frame (as
 * defined by URDF file) and create the adjoint matrix.
 */
void pfc::YoshikawaForceControlClass::forceControlLoopTimerCallback(
  const ros::TimerEvent& e)
{
  ROS_DEBUG("YoshikawaForceControlClass::forceControlLoopTimerCallback");
  // Store the start time of callback function execution.
  this->force_control_timer->start();

  Eigen::Vector3d z_trans;
  z_trans.setZero();
  z_trans(2) = this->z_trans_k_p * this->ft_err(2);

  // Run a saturation filter on the intended axial movement of the probe.
  if(z_trans(2) > this->ee_trans_sat_thresh)
  {
    z_trans(2) = this->ee_trans_sat_thresh;
  }
  else if(z_trans(2) < -this->ee_trans_sat_thresh)
  {
    z_trans(2) = -this->ee_trans_sat_thresh;
  }

  // Define the desired tool frame `T` in the form of its desired transform
  // relative to the current probe frame `P`.
  Eigen::Isometry3d eig_p_t;
  eig_p_t.setIdentity();
  eig_p_t.translate(z_trans);

  try
  {
    // Run the estimation regardless of whether or not it is to be used (see
    // conditional check below) to ensure that the current pose is being
    // updated.
    ROS_DEBUG("Estimating local normal...");
    Eigen::Quaterniond q_p_t = this->estimateLocalNormal();

    if(abs(this->ft_err(2)) <= this->f_z_deadband)
    {
      // The contact force between the probe and the imaging surface is within
      // the acceptable range to attempt to reorient the probe to be normal to
      // the surface.
      //
      // Compute the angle of rotation from the generated quaternion and
      // implement a saturation filter on the computed angle.
      double theta_p_t = 2.0 * acos(q_p_t.w());

      ROS_DEBUG(
        "Angle of rotation before saturation: %.6f deg",
        theta_p_t * 180.0 / M_PI);

      if(theta_p_t > this->ee_rot_sat_thresh)
      {
        theta_p_t = this->ee_rot_sat_thresh;
      }
      else if(theta_p_t < -this->ee_rot_sat_thresh)
      {
        theta_p_t = -this->ee_rot_sat_thresh;
      }

      ROS_DEBUG(
        "Angle of rotation after  saturation: %.6f deg",
        theta_p_t * 180.0 / M_PI);

      // Rescale the vector and scalar components of the quaternion `q_p_t` to
      // match the new angle of rotation.
      Eigen::Vector3d rot_vec_p_t = q_p_t.vec();

      // Prevent a division by zero when the vector component of `q_p_t` is
      // zero.
      if(rot_vec_p_t.norm() > 0.0)
      {
        rot_vec_p_t = rot_vec_p_t / rot_vec_p_t.norm();
      }

      rot_vec_p_t = rot_vec_p_t * sin(theta_p_t / 2.0);

      q_p_t.vec() = rot_vec_p_t;
      q_p_t.w() = cos(theta_p_t / 2.0);

      // This normalization step should be superfluous at this point.
      q_p_t.normalize();
      ROS_DEBUG_STREAM(
        ""
        << "`q_p_t` = [" << q_p_t.x() << ", " << q_p_t.y() << ", " << q_p_t.z()
        << ", " << q_p_t.w() << "]");

      // Apply the quaternion to the desired tool frame `T`.
      eig_p_t.rotate(q_p_t);
    }
    else
    {
      // Skip the rotation to focus on achieving the desired contact force.
      ROS_WARN(
        "Contact not achieved [abs(%.2e) N > %.2e N], avoiding rotation...",
        this->ft_err(2), this->f_z_deadband);
    }

    ROS_DEBUG_STREAM("`eig_p_t` = [" << std::endl << eig_p_t.matrix() << "]");
    tf::Transform t_p_t;
    tf::transformEigenToTF(eig_p_t, t_p_t);
    this->tf_broadcaster.sendTransform(tf::StampedTransform(
      t_p_t, ros::Time::now(), this->probe_frame_name, this->tool_frame_name));
  }
  catch(tf::TransformException& ex)
  {
    ROS_ERROR("%s\nTrying again next cycle...", ex.what());
  }

  // Store the end time and compute the execution duration of the callback function.
  this->force_control_timer->end();
}
