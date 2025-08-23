/**
 *
 * \file yu_bias_est_lrom_class.cpp
 *
 * \brief LROM-based implementation of bias estimation algorithm presented by Yu et al. [1]
 *
 * \author Mardava Gubbi <mgubbi1@jhu.edu>
 *
 * \references 1. Yu et al. "Bias Estimation and Gravity Compensation for Wrist-Mounted Force/Torque Sensor",
 * IEEE Sensors, 2022
 */
#include "pulse_force_estimation/yu_bias_est_lrom_class.hpp"

/*
 * \brief Constructor for YuBiasEstLromClass
 */
YuBiasEstLromClass::YuBiasEstLromClass(
  ros::NodeHandle& nh, std::string base_frame_name, std::string ee_frame_name, std::string sensor_frame_name)
  : ft::YuBiasEstBaseClass(nh, base_frame_name, ee_frame_name, sensor_frame_name), probe_lift_displacement(0.05)
{
  this->num_robot_poses = LROM_NUM_FT_ROBOT_POSES;
  this->num_ft_readings_total = this->num_robot_poses * NUM_FT_SAMPLES_PER_POSE;
  this->ft_readings_vec = std::vector<std::vector<double>>(this->num_ft_readings_total, std::vector<double>(6));
  this->eig_e_b_vec.resize(this->num_robot_poses);
  // this->probe_frame_name = this->mgi_obj->getEndEffectorLink();
  ROS_DEBUG_STREAM("Fetched probe frame name '" << this->probe_frame_name << "' from MoveGroupInterface object.");

  // Set the FSM state at the end of the constructor to ensure that the FSM is not executed in a partially initialized
  // state.
  this->fsm_state = ft::YuBiasEstFsmState::INITIALIZE;
}

/*
 * \brief Estimate the gravitational forces in the robot base frame as described in Section III-A2 of [1].
 */
void YuBiasEstLromClass::estimateGravitationalForcesRobotBaseFrame(void)
{
  // Initialize matrices `A_6`, and `A_9` (see Eqs. (26) and (28) as well as related text in [1]).
  int num_rows_a_mat = 3 * this->num_ft_readings_total;
  Eigen::MatrixXd a6_mat(num_rows_a_mat, 6);
  Eigen::MatrixXd a9_mat(num_rows_a_mat, 9);

  // Matrix `H` and intermediate matrices required to compute `H` (see Eq. (32) in [1]).
  Eigen::MatrixXd a6_t_a6_inv;
  Eigen::MatrixXd sigma_r_mat_inv = sqrt(3.0) * Eigen::MatrixXd::Identity(9, 9);
  Eigen::MatrixXd h_mat;
  Eigen::MatrixXd h_t_h;
  Eigen::MatrixXd x6_coeff_mat;
  Eigen::MatrixXd rot_e_s_u;
  Eigen::MatrixXd rot_e_s_v;
  Eigen::JacobiSVD<Eigen::MatrixXd> h_t_h_svd;
  Eigen::JacobiSVD<Eigen::MatrixXd> rot_e_s_svd;
  Eigen::VectorXd h_t_h_sigma;
  Eigen::VectorXd y_opt_vec;
  Eigen::VectorXd rot_e_s_sigma;
  Eigen::VectorXd x6_vec;
  Eigen::VectorXd x9_vec;

  int i0, i1, i2, n0, r0;
  int min_eigen_val_id = 0;

  // ROS_DEBUG("Initializing matrices `A_6` [%ldx%ld] and `A_9` [%ldx%ld]...",
  // a6_mat.rows(), a6_mat.cols(), a9_mat.rows(), a9_mat.cols());
  a6_mat.setZero();
  a9_mat.setZero();

  for(i0 = 0; i0 < this->num_ft_readings_total; i0++)
  {
    n0 = 3 * i0;
    r0 = i0 / NUM_FT_SAMPLES_PER_POSE;

    for(i1 = 0; i1 < 3; i1++)
    {
      for(i2 = 0; i2 < 3; i2++)
      {
        // Columns of matrix `A_9` are the transpose of the force reading repeated and shifted based on the row index.
        a9_mat(n0 + i1, (3 * i1) + i2) = this->ft_readings_vec[i0][i2];
      }

      // Columns 4-6 of matrix `A_6` are the negative 3x3 identity matrix.
      a6_mat(n0 + i1, 3 + i1) = -1;
    }

    // Columns 1-3 of matrix `A_6` are the negative of the rotation matrices from the robot base frame `B` to the robot
    // end effector frame `E`.
    a6_mat.block(n0, 0, 3, 3) = -this->eig_e_b_vec[r0].matrix().block(0, 0, 3, 3);

    // ROS_DEBUG_STREAM(
    // ""
    // << "[" << i0 << "/" << this->num_ft_readings_total << "]: A9[" << n0
    // << ":" << n0 + 2 << ", :]:" << std::endl
    // << a9_mat.block(n0, 0, 3, a9_mat.cols()));
    // ROS_DEBUG_STREAM(
    // ""
    // << "[" << i0 << "/" << this->num_ft_readings_total << "]: A6[" << n0
    // << ":" << n0 + 2 << ", :]:" << std::endl
    // << a6_mat.block(n0, 0, 3, a6_mat.cols()));
  }

  // ROS_DEBUG("Computing `(A_6^T A_6)^{-1}`...");
  a6_t_a6_inv = a6_mat.transpose() * a6_mat;
  a6_t_a6_inv = a6_t_a6_inv.inverse();
  // ROS_DEBUG("Dimensions of `(A_6^T A_6)^{-1}` are [%ldx%ld].",
  // a6_t_a6_inv.rows(), a6_t_a6_inv.cols());

  // ROS_DEBUG("Computing coefficient of `y` in Eq. (31) for `x_6`...");
  x6_coeff_mat = -a6_t_a6_inv * a6_mat.transpose() * a9_mat * sigma_r_mat_inv;
  // ROS_DEBUG("Dimensions of coefficient matrix are [%ldx%ld].",
  // x6_coeff_mat.rows(), x6_coeff_mat.cols());

  // ROS_DEBUG("Computing matrix `H^T H`...");
  h_mat = (a9_mat * sigma_r_mat_inv) + (a6_mat * x6_coeff_mat);
  h_t_h = h_mat.transpose() * h_mat;
  // ROS_DEBUG("Dimensions of `H^T H` are [%ldx%ld].", h_t_h.rows(), h_t_h.cols());
  // ROS_DEBUG_STREAM("H^T H = " << std::endl << h_t_h);

  // ROS_DEBUG("Performing SVD on `H^T H` to find eigen vectors and eigen values...");
  h_t_h_svd.compute(h_t_h, Eigen::ComputeThinU | Eigen::ComputeThinV);

  // ROS_DEBUG("Finding minimum eigen value of `H^T H`...");
  h_t_h_sigma = h_t_h_svd.singularValues();

  for(i0 = 0; i0 < h_t_h_sigma.rows(); i0++)
  {
    if(h_t_h_sigma[i0] < h_t_h_sigma[min_eigen_val_id])
    {
      min_eigen_val_id = i0;
    }
  }

  // ROS_DEBUG("Selecting corresponding eigen vector...");
  y_opt_vec = h_t_h_svd.matrixU().block(0, min_eigen_val_id, 9, 1);
  // ROS_DEBUG("Dimensions of optimal `y` vector are [%ldx%ld]", y_opt_vec.rows(), y_opt_vec.cols());

  ROS_DEBUG("Computing `x_6` and `x_9` vectors...");
  x6_vec = x6_coeff_mat * y_opt_vec;
  x9_vec = sigma_r_mat_inv * y_opt_vec;
  ROS_DEBUG(
    "x_6 = \n[% .3f, % .3f, % .3f, % .3f, % .3f, % .3f]", x6_vec(0), x6_vec(1), x6_vec(2), x6_vec(3), x6_vec(4),
    x6_vec(5));

  this->rot_s_e.block(0, 0, 1, 3) = x9_vec.block(0, 0, 3, 1).transpose();
  this->rot_s_e.block(1, 0, 1, 3) = x9_vec.block(3, 0, 3, 1).transpose();
  this->rot_s_e.block(2, 0, 1, 3) = x9_vec.block(6, 0, 3, 1).transpose();
  ROS_DEBUG(
    "Rotation matrix from end effector frame to sensor frame before normalizing: [\n% .3f, % .3f, % .3f\n% .3f, % .3f, "
    "% .3f\n% .3f, % .3f, % .3f]",
    this->rot_s_e(0, 0), this->rot_s_e(0, 1), this->rot_s_e(0, 2), this->rot_s_e(1, 0), this->rot_s_e(1, 1),
    this->rot_s_e(1, 2), this->rot_s_e(2, 0), this->rot_s_e(2, 1), this->rot_s_e(2, 2));
  rot_e_s_svd.compute(this->rot_s_e, Eigen::ComputeThinU | Eigen::ComputeThinV);
  this->rot_s_e = rot_e_s_svd.matrixU() * rot_e_s_svd.matrixV().transpose();
  ROS_INFO(
    "Rotation matrix from end effector frame to sensor frame after normalizing: [\n% .3f, % .3f, % .3f\n% .3f, % .3f, "
    "% .3f\n% .3f, % .3f, % .3f]",
    this->rot_s_e(0, 0), this->rot_s_e(0, 1), this->rot_s_e(0, 2), this->rot_s_e(1, 0), this->rot_s_e(1, 1),
    this->rot_s_e(1, 2), this->rot_s_e(2, 0), this->rot_s_e(2, 1), this->rot_s_e(2, 2));

  // According to Fig. 1 our robot installation has `alpha` close to zero, so we should use the second case in Eq. (34)
  // for the gravitational force in the robot base frame `B`.
  ROS_DEBUG("Computing gravitational force in robot base frame `B`...");

  if(x6_vec(2) < 0.0)
  {
    this->f_grav_b = x6_vec.block(0, 0, 3, 1);
  }
  else
  {
    this->f_grav_b = -x6_vec.block(0, 0, 3, 1);
  }
}

/*
 * \brief Compute the joint angles to be achieved by the robot during the bias estimation procedure.
 *
 * NOTE: This function implicitly assumes a UR robot is being used.
 */
void YuBiasEstLromClass::setRobotCmdJointAngles(void)
{
  double base_q_val;
  double elbow_q_val;
  double shoulder_q_val;
  double wrist_1_q_val;
  double wrist_2_q_val;
  double wrist_3_q_val;
  double wrist_val;
  int i0;
  int m0;
  moveit::core::MoveItErrorCode move_err;
  Eigen::Isometry3d probe_motion;
  std::vector<double> joint_q_first_pose;

  // Store the initial state of the robot to which the robot is to be returned after the bias and gravity estimation
  // computations are complete.
  this->joint_q_start = this->mgi_obj->getCurrentJointValues();

  // Lift the probe above its current position to prevent collisions with the surface.
  probe_motion.setIdentity();
  probe_motion(2, 3) = -this->probe_lift_displacement;
  move_err = this->moveRobotProbeFrame(probe_motion);

  if(move_err == moveit::core::MoveItErrorCode::SUCCESS)
  {
    // Store the current state of the robot to be used to generate the commanded robot joint angle vectors.
    joint_q_first_pose = this->mgi_obj->getCurrentJointValues();

    ROS_DEBUG(
      "LROM: Current joint values: [% .6f, % .6f, % .6f, % .6f, % .6f, % .6f]", joint_q_first_pose[0],
      joint_q_first_pose[1], joint_q_first_pose[2], joint_q_first_pose[3], joint_q_first_pose[4],
      joint_q_first_pose[5]);

    // Set the size of the matrix to be populated with the generated commanded robot joint angle vectors.
    this->joint_q_cmd_mat = std::vector<std::vector<double>>(this->num_robot_poses, std::vector<double>(6));

    // Use the current robot pose for the three larger links.
    base_q_val = joint_q_first_pose[0];
    shoulder_q_val = joint_q_first_pose[1];
    elbow_q_val = joint_q_first_pose[2];
    wrist_1_q_val = joint_q_first_pose[3];
    wrist_2_q_val = joint_q_first_pose[4];

    // Use the current robot state for the first set of readings.
    for(i0 = 0; i0 < 6; i0++)
    {
      this->joint_q_cmd_mat[0][i0] = joint_q_first_pose[i0];
    }

    for(i0 = 1; i0 < this->num_robot_poses; i0++)
    {
      this->joint_q_cmd_mat[i0][0] = base_q_val;
      this->joint_q_cmd_mat[i0][1] = shoulder_q_val;
      this->joint_q_cmd_mat[i0][2] = elbow_q_val;
      this->joint_q_cmd_mat[i0][3] =
        ((((double)(i0 - 1)) / ((double)this->num_robot_poses)) * M_PI / 3.0) - (M_PI / 6.0) + wrist_1_q_val;
      this->joint_q_cmd_mat[i0][4] = ((((double)((i0 - 1) % 8)) / 8.0) * M_PI / 3.0) - (M_PI / 6.0) + wrist_2_q_val;

      // Add some variation to the wrist 3 joint to ensure a decent sampling across the space.
      this->joint_q_cmd_mat[i0][5] =
        (((double)(i0 - 1)) * M_PI / (this->num_robot_poses)) - (M_PI / 2.0) + wrist_3_q_val;
    }
  }
  else
  {
    ROS_FATAL("Could not move robot. Exiting now...");
    ros::shutdown();
  }
}
