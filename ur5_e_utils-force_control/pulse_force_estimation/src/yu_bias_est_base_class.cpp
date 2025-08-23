/**
 * \file yu_bias_est_base_class.cpp
 *
 * \brief Estimate the gravitational forces and sensor bias as described by Yu
 * et al. in the paper "Bias Estimation and Gravity Compensation for
 * Wrist-Mounted Force/Torque Sensor" published in IEEE Sensors in 2022.
 *
 * \author Mardava Gubbi <mgubbi1@jhu.edu>
 */
#include "pulse_force_estimation/yu_bias_est_base_class.hpp"

/*
 * \brief Constructor for YuBiasEstBaseClass
 */
ft::YuBiasEstBaseClass::YuBiasEstBaseClass(
  ros::NodeHandle& nh, std::string base_frame_name, std::string ee_frame_name, std::string sensor_frame_name,
  std::string probe_frame_name)
  : mgi::Ur5eMoveGroupInterfaceClass(nh, probe_frame_name)
  , base_frame_name(base_frame_name)
  , ee_frame_name(ee_frame_name)
  , sensor_frame_name(sensor_frame_name)
  , fsm_state(YuBiasEstFsmState::PRE_INIT)
  , ft_reading_id(0)
  , robot_pose_id(0)
{
  this->rot_b_g.setIdentity();

  if(!this->nh.getParam("fsm_period", this->fsm_period))
  {
    ROS_WARN("Could not find parameter 'fsm_period', setting FSM period to "
             "default value (100 ms)...");
    this->fsm_period = 0.1;
  }

  this->bias_update_client = this->nh.serviceClient<pulse_force_estimation::YuSetBiases>("/netft/yu_set_biases");
  this->netft_raw_sensor_sub =
    this->nh.subscribe("/netft/raw_sensor", 1, &YuBiasEstBaseClass::netftRawSensorSubscriberCallback, this);

  // TODO: Get rid of these debug publishers.
  this->debug_tf_pub =
    this->nh.advertise<geometry_msgs::Transform>("/yu_bias_est/robot_poses", NUM_FT_SAMPLES_PER_POSE);
  this->debug_wrench_pub =
    this->nh.advertise<geometry_msgs::Wrench>("/yu_bias_est/ft_readings", NUM_FT_SAMPLES_PER_POSE);

  // The timer should now execute with the FSM in the `pre_init` state, until
  // the derived class constructor has completed its execution.
  this->fsm_timer = this->nh.createTimer(ros::Duration(this->fsm_period), &YuBiasEstBaseClass::fsmTimerCallback, this);
}

/*
 * \brief Estimate the gravitational forces in the robot base frame as
 * described in Section III-A of Yu et al. [1]
 */
void ft::YuBiasEstBaseClass::estimateGravitationalForcesRobotBaseFrame(void)
{
}

/*
 * \brief Compute the joint angles to be achieved by the robot during the bias
 * estimation procedure.
 */
void ft::YuBiasEstBaseClass::setRobotCmdJointAngles(void)
{
}

/*
 * \brief Estimate the sensor bias by solving the force compensation problem
 * as described in Section III-B of Yu et al. [1].
 */
void ft::YuBiasEstBaseClass::estimateForceBiasSensorFrame(void)
{
  int i0;
  int i1;

  // Average of base-to-end effector rotation matrices `R` in Eq. (37) and
  // related text [1].
  Eigen::Matrix3d eig_e_b_avg_mat;
  Eigen::MatrixXd eig_e_b_tmp_mat(4, 4);

  // Average of force readings `F` in sensor frame in Eq. (37) and related text
  // [1].
  Eigen::Vector3d force_readings_avg_s;

  force_readings_avg_s.setZero();
  eig_e_b_avg_mat.setZero();

  ROS_DEBUG("Computing average of all acquired sensor readings as described in "
            "Eq. (37) in Yu et al. [1]...");

  for(i0 = 0; i0 < this->ft_readings_vec.size(); ++i0)
  {
    for(i1 = 0; i1 < 3; ++i1)
    {
      force_readings_avg_s(i1) += this->ft_readings_vec[i0][i1];
    }
  }

  force_readings_avg_s /= this->ft_readings_vec.size();
  ROS_DEBUG(
    "Averaged sensor readings: [\n% .3f, % .3f, % .3f]", force_readings_avg_s(0), force_readings_avg_s(1),
    force_readings_avg_s(2));

  ROS_DEBUG("Computing average of all acquired transformations from robot base "
            "to end effector frame as described in Eq. (37)...");

  for(i0 = 0; i0 < this->eig_e_b_vec.size(); ++i0)
  {
    eig_e_b_tmp_mat = this->eig_e_b_vec[i0].matrix();
    eig_e_b_avg_mat += eig_e_b_tmp_mat.block(0, 0, 3, 3);
  }

  eig_e_b_avg_mat /= this->eig_e_b_vec.size();
  ROS_DEBUG(
    "Averaged transformations: "
    "[\n% .3f, % .3f, % .3f\n% .3f, % .3f, % .3f\n% .3f, % .3f, % .3f]",
    eig_e_b_avg_mat(0, 0), eig_e_b_avg_mat(0, 1), eig_e_b_avg_mat(0, 2), eig_e_b_avg_mat(1, 0), eig_e_b_avg_mat(1, 1),
    eig_e_b_avg_mat(1, 2), eig_e_b_avg_mat(2, 0), eig_e_b_avg_mat(2, 1), eig_e_b_avg_mat(2, 2));

  this->estimateRotationEndEffectorSensor();

  ROS_DEBUG("Computing force bias in sensor frame as described in Eq. (39)...");
  this->f_bias_s = force_readings_avg_s - (this->rot_s_e * eig_e_b_avg_mat * this->f_grav_b);
  ROS_INFO(
    "Force bias in sensor frame:\n[% .3f, % .3f, % .3f] [N]", this->f_bias_s(0), this->f_bias_s(1), this->f_bias_s(2));
}

void ft::YuBiasEstBaseClass::estimateSensorBiasGravitationalForceUpdate(void)
{
  Eigen::Quaterniond q_s_e, q_g_b;
  pulse_force_estimation::YuSetBiases srv;

  ROS_INFO("Estimating gravitational forces in robot base frame as in "
           "Section III-A in Yu et al....");
  this->estimateGravitationalForcesRobotBaseFrame();

  ROS_INFO("Estimating force bias in sensor frame as in Section III-B in "
           "Yu et al....");
  this->estimateForceBiasSensorFrame();

  ROS_INFO("Estimating torque bias in sensor frame as in Section III-C in "
           "Yu et al....");
  this->estimateTorqueBiasSensorFrame();

  ROS_INFO("Estimating robot installation bias...");
  this->estimateRobotInstallationBias();

  ROS_INFO(
    "Writing gravitational force in robot base frame:\n"
    "[% .3f, % .3f, % .3f] to service request...",
    this->f_grav_b(0), this->f_grav_b(1), this->f_grav_b(2));
  tf::vectorEigenToMsg(this->f_grav_b, srv.request.f_grav_b);

  ROS_INFO(
    "Writing force bias:\n"
    "[% .3f, % .3f, % .3f] to service request...",
    this->f_bias_s(0), this->f_bias_s(1), this->f_bias_s(2));
  tf::vectorEigenToMsg(this->f_bias_s, srv.request.f_bias_s);

  ROS_INFO(
    "Writing torque bias:\n"
    "[% .3f, % .3f, % .3f] to service request...",
    this->t_bias_s(0), this->t_bias_s(1), this->t_bias_s(2));
  tf::vectorEigenToMsg(this->t_bias_s, srv.request.t_bias_s);

  ROS_INFO(
    "Writing gravitational arm vector:\n"
    "[% .3f, % .3f, % .3f] to service request...",
    this->p_grav_s(0), this->p_grav_s(1), this->p_grav_s(2));
  tf::vectorEigenToMsg(this->p_grav_s, srv.request.p_grav_s);

  q_s_e = this->rot_s_e;
  ROS_INFO(
    "Writing rotation from end effector to sensor frame:\n["
    "% .3f, % .3f, % .3f, % .3f] to service request...",
    q_s_e.x(), q_s_e.y(), q_s_e.z(), q_s_e.w());
  tf::quaternionEigenToMsg(q_s_e, srv.request.q_s_e);

  if(this->bias_update_client.call(srv))
  {
    ROS_INFO("Successfully called service '/netft/yu_set_biases'.");
  }
  else
  {
    ROS_ERROR("Failed to call service '/netft/yu_set_biases'.");
  }

  ROS_INFO("Writing bias estimations to file...");
  this->writeEstimationsToFile();
}

/*
 * \brief Estimate the sensor torque bias via the torque identification model
 * described in Section III-C of Yu et al.
 */
void ft::YuBiasEstBaseClass::estimateTorqueBiasSensorFrame(void)
{
  // 3x3 identity matrix used multiple times to compute matrix `C` in Eq. (44)
  Eigen::Matrix3d eye;

  // Matrix `C` in Eqs. (44) and (45)
  Eigen::MatrixXd c_mat(3 * this->num_ft_readings_total, 6);

  // Product of `C^T` and `C` in Eq. (45)
  Eigen::MatrixXd ctc(6, 6);
  Eigen::MatrixXd ctc_inv(6, 6);

  // Bias compensated force readings in Eq. (43)
  Eigen::Vector3d force_bias_comp_s;

  // Vector of torque readings in Eqs. (44) and (45)
  Eigen::VectorXd b_vec(3 * this->num_ft_readings_total);

  // Product of `C^T` and `b` in Eq. (45)
  Eigen::VectorXd ctb(6);

  // Vector containing position of tool center-of-gravity and torque bias
  // estimate, both in sensor frame, used in Eqs. (44) and (45).
  Eigen::VectorXd y(6);

  int i0;
  int i1;

  ROS_DEBUG("Computing matrix `C` and vector `b` as described in text under Eq. "
            "(44)...");
  eye.setIdentity();

  for(i0 = 0; i0 < this->num_ft_readings_total; ++i0)
  {
    for(i1 = 0; i1 < 3; ++i1)
    {
      force_bias_comp_s(i1) = -(this->ft_readings_vec[i0][i1] - this->f_bias_s(i1));
      b_vec((3 * i0) + i1) = this->ft_readings_vec[i0][i1 + 3];
    }

    c_mat.block(3 * i0, 0, 3, 3) = hatOperator(force_bias_comp_s);
    c_mat.block(3 * i0, 3, 3, 3) = eye;

    // ROS_DEBUG(
    // "[C, b] (slice %d): [\n"
    // "% .3f, % .3f, % .3f, % .3f, % .3f, % .3f, % .3f\n"
    // "% .3f, % .3f, % .3f, % .3f, % .3f, % .3f, % .3f\n"
    // "% .3f, % .3f, % .3f, % .3f, % .3f, % .3f, % .3f]",
    // i0, c_mat((3 * i0), 0), c_mat((3 * i0), 1), c_mat((3 * i0), 2),
    // c_mat((3 * i0), 3), c_mat((3 * i0), 4), c_mat((3 * i0), 5),
    // b_vec((3 * i0)), c_mat((3 * i0) + 1, 0), c_mat((3 * i0) + 1, 1),
    // c_mat((3 * i0) + 1, 2), c_mat((3 * i0) + 1, 3), c_mat((3 * i0) + 1, 4),
    // c_mat((3 * i0) + 1, 5), b_vec((3 * i0) + 1), c_mat((3 * i0) + 2, 0),
    // c_mat((3 * i0) + 2, 1), c_mat((3 * i0) + 2, 2), c_mat((3 * i0) + 2, 3),
    // c_mat((3 * i0) + 2, 4), c_mat((3 * i0) + 2, 5), b_vec((3 * i0) + 2));
  }

  ROS_DEBUG("Computing vector `y` as described in Eq. (45)...");
  ctc = c_mat.transpose() * c_mat;
  // ROS_DEBUG(
  // "C^T C: [\n"
  // "% .3f, % .3f, % .3f, % .3f, % .3f, % .3f\n"
  // "% .3f, % .3f, % .3f, % .3f, % .3f, % .3f\n"
  // "% .3f, % .3f, % .3f, % .3f, % .3f, % .3f\n"
  // "% .3f, % .3f, % .3f, % .3f, % .3f, % .3f\n"
  // "% .3f, % .3f, % .3f, % .3f, % .3f, % .3f\n"
  // "% .3f, % .3f, % .3f, % .3f, % .3f, % .3f]",
  // ctc(0, 0), ctc(0, 1), ctc(0, 2), ctc(0, 3), ctc(0, 4), ctc(0, 5), ctc(1,
  // 0), ctc(1, 1), ctc(1, 2), ctc(1, 3), ctc(1, 4), ctc(1, 5), ctc(2, 0),
  // ctc(2, 1), ctc(2, 2), ctc(2, 3), ctc(2, 4), ctc(2, 5), ctc(3, 0), ctc(3,
  // 1), ctc(3, 2), ctc(3, 3), ctc(3, 4), ctc(3, 5), ctc(4, 0), ctc(4, 1),
  // ctc(4, 2), ctc(4, 3), ctc(4, 4), ctc(4, 5), ctc(5, 0), ctc(5, 1), ctc(5,
  // 2), ctc(5, 3), ctc(5, 4), ctc(5, 5));

  ctc_inv = ctc.inverse();
  // ROS_DEBUG(
  // "(C^T C)^{-1}: [\n"
  // "% .3f, % .3f, % .3f, % .3f, % .3f, % .3f\n"
  // "% .3f, % .3f, % .3f, % .3f, % .3f, % .3f\n"
  // "% .3f, % .3f, % .3f, % .3f, % .3f, % .3f\n"
  // "% .3f, % .3f, % .3f, % .3f, % .3f, % .3f\n"
  // "% .3f, % .3f, % .3f, % .3f, % .3f, % .3f\n"
  // "% .3f, % .3f, % .3f, % .3f, % .3f, % .3f]",
  // ctc_inv(0, 0), ctc_inv(0, 1), ctc_inv(0, 2), ctc_inv(0, 3), ctc_inv(0, 4),
  // ctc_inv(0, 5), ctc_inv(1, 0), ctc_inv(1, 1), ctc_inv(1, 2), ctc_inv(1, 3),
  // ctc_inv(1, 4), ctc_inv(1, 5), ctc_inv(2, 0), ctc_inv(2, 1), ctc_inv(2, 2),
  // ctc_inv(2, 3), ctc_inv(2, 4), ctc_inv(2, 5), ctc_inv(3, 0), ctc_inv(3, 1),
  // ctc_inv(3, 2), ctc_inv(3, 3), ctc_inv(3, 4), ctc_inv(3, 5), ctc_inv(4, 0),
  // ctc_inv(4, 1), ctc_inv(4, 2), ctc_inv(4, 3), ctc_inv(4, 4), ctc_inv(4, 5),
  // ctc_inv(5, 0), ctc_inv(5, 1), ctc_inv(5, 2), ctc_inv(5, 3), ctc_inv(5, 4),
  // ctc_inv(5, 5));

  ctb = c_mat.transpose() * b_vec;
  // ROS_DEBUG(
  // "C^T b: [\n% .3f, % .3f, % .3f, % .3f, % .3f, % .3f]", ctb(0), ctb(1),
  // ctb(2), ctb(3), ctb(4), ctb(5));

  y = ctc_inv * ctb;
  // ROS_DEBUG(
  // "y: [\n% .3f, % .3f, % .3f, % .3f, % .3f, % .3f]", y(0), y(1), y(2), y(3),
  // y(4), y(5));

  this->p_grav_s = y.head(3);
  ROS_INFO(
    "Center of gravity of tool in sensor frame:\n[% .3f, % .3f, % .3f] [mm]", this->p_grav_s(0) * 1e3,
    this->p_grav_s(1) * 1e3, this->p_grav_s(2) * 1e3);

  this->t_bias_s = y.segment(3, 3);
  ROS_INFO(
    "Torque bias in sensor frame:\n[% .3f, % .3f, % .3f] [Nm]", this->t_bias_s(0), this->t_bias_s(1), this->t_bias_s(2));
}

/*
 * \brief Estimate the rotation from the tool gravity frame `G` to the robot
 * base frame `B` as described in Section III-D of Yu et al.
 */
void ft::YuBiasEstBaseClass::estimateRobotInstallationBias(void)
{
  // Compute the magnitude of the gravitational force (see Eq. (48)).
  double f_grav_norm = sqrt(
    (this->f_grav_b(0) * this->f_grav_b(0)) + (this->f_grav_b(1) * this->f_grav_b(1))
    + (this->f_grav_b(2) * this->f_grav_b(2)));

  // Compute the Tait-Bryan angles (see Eq. (48)) from the gravitational
  // frame `G` to the robot base frame `B`.
  double tait_bryan_beta = atan2(this->f_grav_b(0), this->f_grav_b(2));

  // Potential singularity here if robot base is effectively horizontal (i.e.,
  // no gravitational force along z-axis of robot base frame `B`). This will
  // likely never occur in the PULSE Lab.
  double tait_bryan_alpha = atan2(-this->f_grav_b(1) * cos(tait_bryan_beta), this->f_grav_b(2));

  // Compute the rotation matrix one term at a time (see Eq. (46)).
  this->rot_b_g.setIdentity();

  this->rot_b_g(0, 0) = cos(tait_bryan_beta);
  this->rot_b_g(0, 1) = sin(tait_bryan_alpha) * sin(tait_bryan_beta);
  this->rot_b_g(0, 2) = cos(tait_bryan_alpha) * sin(tait_bryan_beta);
  this->rot_b_g(1, 0) = 0.0;
  this->rot_b_g(1, 1) = cos(tait_bryan_alpha);
  this->rot_b_g(1, 2) = -sin(tait_bryan_alpha);
  this->rot_b_g(2, 0) = -sin(tait_bryan_beta);
  this->rot_b_g(2, 1) = sin(tait_bryan_alpha) * cos(tait_bryan_beta);
  this->rot_b_g(2, 2) = cos(tait_bryan_alpha) * cos(tait_bryan_beta);
}

/*
 * \brief Estimate the rotation from the robot end effector frame `E` to the
 * sensor frame `S` as described in Section III-B of Yu et al.
 */
void ft::YuBiasEstBaseClass::estimateRotationEndEffectorSensor(void)
{
  int i0;
  int i1;
  int j0;

  // Matrix `D^T` in Eq. (40) and related text.
  Eigen::Matrix3d d_mat_transpose;
  Eigen::JacobiSVD<Eigen::MatrixXd> d_svd;
  Eigen::MatrixXd d_u;
  Eigen::MatrixXd d_v;
  Eigen::MatrixXd d_sigma;
  Eigen::MatrixXd d_recon;

  // Average of base-to-end effector rotation matrices `R` in Eq. (37) and
  // related text.
  Eigen::Matrix3d eig_e_b_avg_mat;
  Eigen::MatrixXd eig_e_b_tmp_mat(4, 4);

  // Average of force readings `F` in sensor frame in Eq. (37) and related text.
  Eigen::Vector3d force_readings_avg_s;
  Eigen::Vector3d force_reading_shifted_tmp;

  force_readings_avg_s.setZero();
  d_mat_transpose.setZero();
  eig_e_b_avg_mat.setZero();

  ROS_DEBUG("Computing average of all acquired sensor readings as described in "
            "Eq. (37)...");

  for(i0 = 0; i0 < this->ft_readings_vec.size(); ++i0)
  {
    for(i1 = 0; i1 < 3; ++i1)
    {
      force_readings_avg_s(i1) += this->ft_readings_vec[i0][i1];
    }
  }

  ROS_DEBUG(
    "Summed sensor readings: [\n% .3f, % .3f, % .3f]", force_readings_avg_s(0), force_readings_avg_s(1),
    force_readings_avg_s(2));
  force_readings_avg_s /= this->ft_readings_vec.size();
  ROS_DEBUG(
    "Averaged sensor readings: [\n% .3f, % .3f, % .3f]", force_readings_avg_s(0), force_readings_avg_s(1),
    force_readings_avg_s(2));

  ROS_DEBUG("Computing average of all acquired transformations from robot base "
            "to end effector frame as described in Eq. (37)...");

  for(i0 = 0; i0 < this->eig_e_b_vec.size(); ++i0)
  {
    eig_e_b_tmp_mat = this->eig_e_b_vec[i0].matrix();
    eig_e_b_avg_mat += eig_e_b_tmp_mat.block(0, 0, 3, 3);
  }

  ROS_DEBUG(
    "Summed transformations: [\n% .3f, % .3f, % .3f\n% .3f, % .3f, % .3f\n% "
    ".3f, "
    "% .3f, % .3f]",
    eig_e_b_avg_mat(0, 0), eig_e_b_avg_mat(0, 1), eig_e_b_avg_mat(0, 2), eig_e_b_avg_mat(1, 0), eig_e_b_avg_mat(1, 1),
    eig_e_b_avg_mat(1, 2), eig_e_b_avg_mat(2, 0), eig_e_b_avg_mat(2, 1), eig_e_b_avg_mat(2, 2));
  eig_e_b_avg_mat /= this->num_robot_poses;
  ROS_DEBUG(
    "Averaged transformations: [\n% .3f, % .3f, % .3f\n% .3f, % .3f, % .3f\n% "
    ".3f, "
    "% .3f, % .3f]",
    eig_e_b_avg_mat(0, 0), eig_e_b_avg_mat(0, 1), eig_e_b_avg_mat(0, 2), eig_e_b_avg_mat(1, 0), eig_e_b_avg_mat(1, 1),
    eig_e_b_avg_mat(1, 2), eig_e_b_avg_mat(2, 0), eig_e_b_avg_mat(2, 1), eig_e_b_avg_mat(2, 2));

  ROS_DEBUG("Computing transpose of matrix `D` as described in text under Eq. (40)...");

  for(i0 = 0; i0 < this->num_ft_readings_total; ++i0)
  {
    j0 = i0 / NUM_FT_SAMPLES_PER_POSE;

    ROS_DEBUG_STREAM(
      ""
      << "Fetching robot pose [" << j0 << "/" << this->eig_e_b_vec.size() << "] corresponding to current F/T reading ["
      << i0 << "/" << this->num_ft_readings_total << "]...");
    eig_e_b_tmp_mat = this->eig_e_b_vec[j0].matrix();

    ROS_DEBUG_STREAM(
      ""
      << "Accessing element [" << i0 << "/" << this->ft_readings_vec.size() << "] of F/T readings matrix...");

    for(i1 = 0; i1 < 3; ++i1)
    {
      force_reading_shifted_tmp(i1) = this->ft_readings_vec[i0][i1] - force_readings_avg_s(i1);
    }

    // Define matrix 'D' according to (40) in Yu et al.
    ROS_DEBUG("Incrementing `D^T` matrix...");
    d_mat_transpose +=
      (eig_e_b_tmp_mat.block(0, 0, 3, 3) - eig_e_b_avg_mat) * this->f_grav_b * force_reading_shifted_tmp.transpose();
  }

  ROS_DEBUG(
    "D^T: ["
    "\n% .3f, % .3f, % .3f"
    "\n% .3f, % .3f, % .3f"
    "\n% .3f, % .3f, % .3f]",
    d_mat_transpose(0, 0), d_mat_transpose(0, 1), d_mat_transpose(0, 2), d_mat_transpose(1, 0), d_mat_transpose(1, 1),
    d_mat_transpose(1, 2), d_mat_transpose(2, 0), d_mat_transpose(2, 1), d_mat_transpose(2, 2));
  ROS_DEBUG("Computing SVD of matrix `D` as described in Eq. (41)...");
  d_svd.compute(d_mat_transpose.transpose(), Eigen::ComputeThinU | Eigen::ComputeThinV);
  d_u = d_svd.matrixU();
  ROS_DEBUG(
    "U: ["
    "\n% .3f, % .3f, % .3f"
    "\n% .3f, % .3f, % .3f"
    "\n% .3f, % .3f, % .3f]",
    d_u(0, 0), d_u(0, 1), d_u(0, 2), d_u(1, 0), d_u(1, 1), d_u(1, 2), d_u(2, 0), d_u(2, 1), d_u(2, 2));

  d_v = d_svd.matrixV();
  ROS_DEBUG(
    "V: ["
    "\n% .3f, % .3f, % .3f"
    "\n% .3f, % .3f, % .3f"
    "\n% .3f, % .3f, % .3f]",
    d_v(0, 0), d_v(0, 1), d_v(0, 2), d_v(1, 0), d_v(1, 1), d_v(1, 2), d_v(2, 0), d_v(2, 1), d_v(2, 2));

  d_sigma = d_svd.singularValues().asDiagonal();
  ROS_DEBUG(
    "Sigma: ["
    "\n% .3f, % .3f, % .3f"
    "\n% .3f, % .3f, % .3f"
    "\n% .3f, % .3f, % .3f]",
    d_sigma(0, 0), d_sigma(0, 1), d_sigma(0, 2), d_sigma(1, 0), d_sigma(1, 1), d_sigma(1, 2), d_sigma(2, 0),
    d_sigma(2, 1), d_sigma(2, 2));

  d_recon = d_u * d_sigma * d_v.transpose();
  ROS_DEBUG(
    "D (reconstructed from SVD): ["
    "\n% .3f, % .3f, % .3f"
    "\n% .3f, % .3f, % .3f"
    "\n% .3f, % .3f, % .3f]",
    d_recon(0, 0), d_recon(0, 1), d_recon(0, 2), d_recon(1, 0), d_recon(1, 1), d_recon(1, 2), d_recon(2, 0),
    d_recon(2, 1), d_recon(2, 2));

  // TODO: Add conditional here checking for `D` singularity to match Eq. (42).
  ROS_DEBUG("Computing rotational transformation from end effector to sensor "
            "frame as described in Eq. (42)...");

  // Compute central matrix in RHS of top conditional block in Eq. (42)
  // (`diag{...}`).
  this->rot_s_e.setIdentity();
  this->rot_s_e(2, 2) = d_svd.matrixU().determinant() * d_svd.matrixV().determinant();
  ROS_DEBUG(
    "Temporary rotation matrix before pre- and post-multiplication: [\n"
    "% .3f, % .3f, % .3f\n% .3f, % .3f, % .3f\n% .3f, % .3f, % .3f]",
    this->rot_s_e(0, 0), this->rot_s_e(0, 1), this->rot_s_e(0, 2), this->rot_s_e(1, 0), this->rot_s_e(1, 1),
    this->rot_s_e(1, 2), this->rot_s_e(2, 0), this->rot_s_e(2, 1), this->rot_s_e(2, 2));

  this->rot_s_e = d_svd.matrixU() * this->rot_s_e * d_svd.matrixV().transpose();
  ROS_INFO(
    "Rotation matrix from end effector frame to sensor frame: [\n% .3f, % .3f, "
    "% .3f\n% .3f, % .3f, % .3f\n% .3f, % .3f, % .3f]",
    this->rot_s_e(0, 0), this->rot_s_e(0, 1), this->rot_s_e(0, 2), this->rot_s_e(1, 0), this->rot_s_e(1, 1),
    this->rot_s_e(1, 2), this->rot_s_e(2, 0), this->rot_s_e(2, 1), this->rot_s_e(2, 2));
}

void ft::YuBiasEstBaseClass::writeEstimationsToFile(void)
{
  boost::filesystem::path package_dir(ros::package::getPath("pulse_force_estimation"));
  boost::filesystem::path output_h5_sub_dir_name("config");
  boost::filesystem::path output_h5_file_base_name("force_bias_est.h5");

  boost::filesystem::path output_h5_dir_name = package_dir / output_h5_sub_dir_name;
  boost::filesystem::path output_h5_file_name = output_h5_dir_name / output_h5_file_base_name;

  Eigen::Quaterniond q_s_e;
  int i0;

  std::vector<double> f_grav_b_vec(3);
  std::vector<double> f_bias_s_vec(3);
  std::vector<double> t_bias_s_vec(3);
  std::vector<double> p_grav_s_vec(3);
  std::vector<double> q_s_e_vec(4);

  if(boost::filesystem::is_directory(output_h5_dir_name))
  {
    ROS_DEBUG("Found existing directory '%s'.", output_h5_dir_name.string().c_str());
  }
  else
  {
    ROS_INFO("Creating directory '%s'...", output_h5_dir_name.string().c_str());
    boost::filesystem::create_directory(output_h5_dir_name);
  }

  for(i0 = 0; i0 < f_grav_b_vec.size(); i0++)
  {
    f_grav_b_vec[i0] = this->f_grav_b(i0);
    f_bias_s_vec[i0] = this->f_bias_s(i0);
    t_bias_s_vec[i0] = this->t_bias_s(i0);
    p_grav_s_vec[i0] = this->p_grav_s(i0);
  }

  q_s_e = this->rot_s_e;
  q_s_e_vec[0] = q_s_e.x();
  q_s_e_vec[1] = q_s_e.y();
  q_s_e_vec[2] = q_s_e.z();
  q_s_e_vec[3] = q_s_e.w();

  ROS_INFO("Writing bias estimations to file '%s'...", output_h5_file_name.string().c_str());
  HighFive::File output_h5_fh(output_h5_file_name.string(), HighFive::File::ReadWrite | HighFive::File::Truncate);
  output_h5_fh.createDataSet<double>("/f_grav_b", HighFive::DataSpace::From(f_grav_b_vec)).write(f_grav_b_vec);
  output_h5_fh.createDataSet<double>("/f_bias_s", HighFive::DataSpace::From(f_bias_s_vec)).write(f_bias_s_vec);
  output_h5_fh.createDataSet<double>("/t_bias_s", HighFive::DataSpace::From(t_bias_s_vec)).write(t_bias_s_vec);
  output_h5_fh.createDataSet<double>("/p_grav_s", HighFive::DataSpace::From(p_grav_s_vec)).write(p_grav_s_vec);
  output_h5_fh.createDataSet<double>("/q_s_e", HighFive::DataSpace::From(q_s_e_vec)).write(q_s_e_vec);
}

/*
 * \brief Callback function for raw sensor-frame FT sensor readings subscriber
 */
void ft::YuBiasEstBaseClass::netftRawSensorSubscriberCallback(geometry_msgs::WrenchStamped msg)
{
  tf::wrenchMsgToEigen(msg.wrench, this->ft_reading_curr);
}

/*
 * \brief Callback function for FSM timer
 */
void ft::YuBiasEstBaseClass::fsmTimerCallback(const ros::TimerEvent& e)
{
  Eigen::Isometry3d eig_e_b;
  tf::StampedTransform st_e_b;
  geometry_msgs::Transform debug_tf_msg;
  geometry_msgs::Wrench debug_wrench_msg;
  Eigen::MatrixXd mat_e_b;

  // Copy to ensure that the reading is not changed during this operation.
  Eigen::MatrixXd ft_reading_tmp = this->ft_reading_curr;
  int i0;
  int i1;
  moveit::core::MoveItErrorCode move_exe_success;
  std::vector<double> q(6);

  switch(this->fsm_state)
  {
    case YuBiasEstFsmState::PRE_INIT:
      ROS_DEBUG("FSM: Waiting for derived class constructor to finish...");
      break;
    case YuBiasEstFsmState::INITIALIZE:
      ROS_DEBUG("FSM: Setting desired robot joint angle vectors...");
      this->setRobotCmdJointAngles();
      this->fsm_state = YuBiasEstFsmState::MOVE_ROBOT;
      break;
    case YuBiasEstFsmState::MOVE_ROBOT:
      ROS_DEBUG("FSM: Deciding whether or not to move robot...");

      if(this->robot_pose_id < this->num_robot_poses)
      {
        ROS_DEBUG("FSM: [%2d/%2d] Planning robot motion...", this->robot_pose_id, this->num_robot_poses);

        move_exe_success = this->moveRobotJointTarget(this->joint_q_cmd_mat[this->robot_pose_id]);

        if(move_exe_success == moveit::core::MoveItErrorCode::SUCCESS)
        {
          this->fsm_state = YuBiasEstFsmState::ACQUIRE_DATA;
          this->robot_pose_id++;
        }
        else
        {
          // No operation
        }
      }
      else
      {
        ROS_DEBUG("FSM: Estimating bias...");
        this->fsm_state = YuBiasEstFsmState::ESTIMATE_BIAS;
      }

      break;
    case YuBiasEstFsmState::ACQUIRE_DATA:
      ROS_DEBUG("FSM: [%3d/%3d] Acquiring raw F/T sensor readings...", this->ft_reading_id, this->num_ft_readings_total);

      // Fill corresponding buffer row with latest raw F/T reading.
      for(i0 = 0; i0 < 6; i0++)
      {
        this->ft_readings_vec[this->ft_reading_id][i0] = ft_reading_tmp(i0);
      }

      tf::wrenchEigenToMsg(ft_reading_tmp, debug_wrench_msg);
      this->debug_wrench_pub.publish(debug_wrench_msg);

      // Increment buffer row index.
      this->ft_reading_id++;

      try
      {
        ROS_DEBUG("FSM: [%3d/%3d] Acquiring current robot pose...", this->robot_pose_id - 1, this->num_robot_poses);
        ROS_DEBUG_STREAM(
          ""
          << "FSM: Listening for transformation from '" << this->base_frame_name << "' to '" << this->ee_frame_name
          << "'...");
        this->tf_listener.lookupTransform(this->ee_frame_name, this->base_frame_name, ros::Time(0), st_e_b);
        ROS_DEBUG("FSM: Transforming robot pose to Eigen Isometry3d object...");
        tf::transformTFToEigen(st_e_b, eig_e_b);
        ROS_DEBUG(
          "FSM: Storing Eigen Isometry3d object in vector of robot poses of size %ld...", this->eig_e_b_vec.size());
        this->eig_e_b_vec[this->robot_pose_id - 1] = eig_e_b;
      }
      catch(tf::TransformException& ex)
      {
        ROS_ERROR("%s", ex.what());
      }

      tf::transformEigenToMsg(eig_e_b, debug_tf_msg);
      this->debug_tf_pub.publish(debug_tf_msg);

      // Check to see if we must move to the next position, or if we must
      // continue acquiring sensor data for the current position.
      if(this->ft_reading_id % NUM_FT_SAMPLES_PER_POSE == 0)
      {
        this->fsm_state = YuBiasEstFsmState::UNTANGLE;
      }
      else
      {
        // No operation
      }

      break;
    case YuBiasEstFsmState::UNTANGLE:
      ROS_DEBUG("FSM: Planning untangling robot motion...");
      move_exe_success = this->moveRobotJointTarget(this->joint_q_cmd_mat[0]);

      if(move_exe_success == moveit::core::MoveItErrorCode::SUCCESS)
      {
        this->fsm_state = YuBiasEstFsmState::MOVE_ROBOT;
      }
      else
      {
        // No operation
      }

      break;
    case YuBiasEstFsmState::ESTIMATE_BIAS:
      ROS_DEBUG("FSM: Estimating and updating sensor bias, gravitational force, and "
                "robot installation bias...");
      this->estimateSensorBiasGravitationalForceUpdate();
      this->fsm_state = YuBiasEstFsmState::COMPLETED;
      break;
    case YuBiasEstFsmState::COMPLETED:
      ROS_DEBUG("FSM: Planning reset robot motion...");
      move_exe_success = this->moveRobotJointTarget(this->joint_q_start);

      if(move_exe_success == moveit::core::MoveItErrorCode::SUCCESS)
      {
        ROS_DEBUG("FSM: Shutting down ROS node...");
        ros::shutdown();
      }
      else
      {
        // No operation
      }

      break;
    default:
      ROS_ERROR_STREAM("FSM: Unknown FSM state " << this->fsm_state);
  }
}
