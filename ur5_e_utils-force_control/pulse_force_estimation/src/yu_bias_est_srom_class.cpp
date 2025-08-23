#include "pulse_force_estimation/yu_bias_est_srom_class.hpp"

/*
 * \brief Constructor for YuBiasEstSromClass
 */
YuBiasEstSromClass::YuBiasEstSromClass(
  ros::NodeHandle& nh, std::string base_frame_name, std::string ee_frame_name, std::string sensor_frame_name)
  : ft::YuBiasEstBaseClass(nh, base_frame_name, ee_frame_name, sensor_frame_name)
{
  this->num_robot_poses = SROM_NUM_FT_ROBOT_POSES;
  this->num_ft_readings_total = this->num_robot_poses * NUM_FT_SAMPLES_PER_POSE;
  this->ft_readings_vec = std::vector<std::vector<double>>(this->num_ft_readings_total, std::vector<double>(6));
  this->eig_e_b_vec.resize(this->num_robot_poses);
  this->fsm_state = ft::YuBiasEstFsmState::INITIALIZE;
}

/*
 * \brief Estimate the gravitational forces in the robot base frame as
 * described in Section III-A1 of Yu et al.
 */
void YuBiasEstSromClass::estimateGravitationalForcesRobotBaseFrame(void)
{
  // Array of matrices where p_mat[i] corresponds to the set of column vectors
  // [p_{i1}, p_{i2}, p_{i3}] in Yu et al.
  Eigen::Matrix3d p_mat[3];

  // Counters and derived variables
  int i0;
  int i1;
  int i2;
  int i3;
  int j0;
  int j2;
  int p2;

  // Indices of force readings to be used to compute each element in `p_mat`.
  // See Eqs. (17)-(23) and related text.
  int p_id_vec[3][3][8] = {
    { { 1, 2, 3, 4, 5, 6, 7, 8 }, { 9, 10, 11, 12, 13, 14, 15, 16 }, { 17, 18, 19, 20, 21, 22, 23, 24 } },
    { { 9, 13, 17, 21, 10, 14, 18, 22 }, { 1, 5, 19, 23, 2, 6, 20, 24 }, { 3, 7, 11, 15, 4, 8, 12, 16 } },
    { { 11, 16, 20, 23, 12, 15, 19, 24 }, { 4, 7, 17, 22, 3, 8, 18, 21 }, { 1, 6, 10, 13, 2, 5, 9, 14 } }
  };
  double ft_readings_avg_vec[this->num_robot_poses][6];

  ROS_DEBUG("Mapping robot poses and force-torque readings to Fig. 3 in Yu "
            "et al....");
  this->sortPosesSpecialRobotOrientationMethod();

  for(i0 = 0; i0 < 3; ++i0)
  {
    p_mat[i0].setZero();
  }

  // Compute the average force-torque sensor readings for each pose by averaging
  // all readings taken at each pose.
  ROS_DEBUG("Averaging over-sampled force-torque sensor readings...");

  for(i0 = 0; i0 < this->num_ft_readings_total; ++i0)
  {
    j0 = i0 / NUM_FT_SAMPLES_PER_POSE;
    ROS_DEBUG(
      "[%2d/%2d=%2ld] (readings) -> [%2d/%2d] (average)", i0, this->num_ft_readings_total, this->ft_readings_vec.size(),
      j0, this->num_robot_poses);

    for(i1 = 0; i1 < 6; ++i1)
    {
      if(i0 % NUM_FT_SAMPLES_PER_POSE == 0)
      {
        ft_readings_avg_vec[j0][i1] = this->ft_readings_vec[i0][i1];
      }
      else
      {
        ft_readings_avg_vec[j0][i1] += this->ft_readings_vec[i0][i1];
      }
    }
  }

  for(i0 = 0; i0 < this->num_robot_poses; ++i0)
  {
    for(i1 = 0; i1 < 6; ++i1)
    {
      ft_readings_avg_vec[i0][i1] /= NUM_FT_SAMPLES_PER_POSE;
    }
  }

  ROS_DEBUG("Computing matrix `p` described in Eqs. (17)-(23) in Yu et al....");

  for(i0 = 0; i0 < 3; ++i0)
  {
    for(i1 = 0; i1 < 3; ++i1)
    {
      for(i2 = 0; i2 < 8; ++i2)
      {
        if(i2 < 4)
        {
          j2 = 1;
        }
        else
        {
          j2 = -1;
        }

        p2 = this->fwd_kin_pose_id_map[p_id_vec[i0][i1][i2] - 1];

        for(i3 = 0; i3 < 3; ++i3)
        {
          p_mat[i0](i1, i3) += j2 * ft_readings_avg_vec[p2][i3];
        }
      }
    }
  }

  for(i0 = 0; i0 < 3; ++i0)
  {
    p_mat[i0] /= 8.0;
    this->f_grav_b(i0) = std::cbrt(p_mat[i0].determinant());
  }

  ROS_INFO(
    "Gravitational force in robot base frame:\n[% .3f, % .3f, % .3f] [N]", this->f_grav_b(0), this->f_grav_b(1),
    this->f_grav_b(2));
}

/*
 * \brief Compute the joint angles to be achieved by the robot during the bias
 * estimation procedure.
 *
 * NOTE: This function implicitly assumes a UR robot is being used.
 */
void YuBiasEstSromClass::setRobotCmdJointAngles(void)
{
  int i0;
  double base_q_val;
  double shoulder_q_val;
  double elbow_q_val;

  // Store the initial state of the robot to be used to generate the commanded
  // robot joint angle vectors.
  ROS_DEBUG("SROM: Fetching current robot pose...");
  this->joint_q_start = this->mgi_obj->getCurrentJointValues();

  // Set the size of the matrix to be populated with the generated commanded
  // robot joint angle vectors.
  ROS_DEBUG("Resizing commanded joint vector matrix to %d rows...", this->num_robot_poses);
  this->joint_q_cmd_mat = std::vector<std::vector<double>>(this->num_robot_poses, std::vector<double>(6));

  // Hard-code the base and elbow links for now.
  base_q_val = M_PI / 2.0;
  elbow_q_val = M_PI / 2.0;

  // The shoulder link is always set to (-pi / 2).
  shoulder_q_val = -M_PI / 2.0;

  for(i0 = 0; i0 < this->num_robot_poses; i0++)
  {
    this->joint_q_cmd_mat[i0][0] = base_q_val;
    this->joint_q_cmd_mat[i0][1] = shoulder_q_val;
    this->joint_q_cmd_mat[i0][2] = elbow_q_val;

    // The wrist-1 link will jump back and forth between -pi/2 and 0.0,
    // corresponding to downwards and horizontal, respectively.
    if(i0 < 5 || i0 >= 20)
    {
      this->joint_q_cmd_mat[i0][3] = -M_PI / 2.0;
    }
    else
    {
      this->joint_q_cmd_mat[i0][3] = 0.0;
    }

    // The wrist-2 link will be the opposite of the elbow link while the probe
    // is facing downwards, and rotate through angles for horizontal poses.
    if(i0 < 10)
    {
      this->joint_q_cmd_mat[i0][4] = -elbow_q_val;
    }
    else if(i0 < 15)
    {
      this->joint_q_cmd_mat[i0][4] = 0.0;
    }
    else
    {
      this->joint_q_cmd_mat[i0][4] = elbow_q_val;
    }

    // The wrist-3 link ideally should depend on the initial robot state, but
    // that would require some careful planning to determine the best path to
    // avoid tangling cables. At this stage, that is currently not taken into
    // account. If this is to be executed with the probe mounted, then the probe
    // is expected to be disconnected from the scanner, with the connector held
    // in the user's hand.
    switch(i0 % 4)
    {
      case 0:
        this->joint_q_cmd_mat[i0][5] = -M_PI;
        break;
      case 1:
        this->joint_q_cmd_mat[i0][5] = -M_PI / 2.0;
        break;
      case 2:
        this->joint_q_cmd_mat[i0][5] = 0.0;
        break;
      case 3:
        this->joint_q_cmd_mat[i0][5] = M_PI / 2.0;
        break;
    }

    // Finally, determine whether or not the robot pose is to be used for
    // FT sensor data acquisition, or purely for positioning to prevent the
    // cables from the FT sensor and ultrasound probe from being tangled with
    // the robot.
    ROS_DEBUG(
      "[%2d/%2d] [data]\n[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]", i0, this->num_robot_poses, this->joint_q_cmd_mat[i0][0],
      this->joint_q_cmd_mat[i0][1], this->joint_q_cmd_mat[i0][2], this->joint_q_cmd_mat[i0][3],
      this->joint_q_cmd_mat[i0][4], this->joint_q_cmd_mat[i0][5]);
  }
}

/*
 * \brief Generate an index vector sorting the acquired poses and force-torque
 * sensor readings according to the Special Robot Orientation Method described
 * in Fig. 3 in Yu et al.
 */
void YuBiasEstSromClass::sortPosesSpecialRobotOrientationMethod(void)
{
  // See Fig. 3 in Yu et al., IEEE Sensors 2022
  int i0;
  int i1;
  int j1;
  int pose_id_fwd_kin_map[this->num_robot_poses] = { 0 };
  Eigen::Isometry3d e0;

  for(i0 = 0; i0 < this->num_robot_poses; i0++)
  {
    e0 = this->eig_e_b_vec[i0];

    if(e0(0, 0) >= 0.99)
    {
      if(e0(1, 1) >= 0.99 && e0(2, 2) >= 0.99)
      {
        pose_id_fwd_kin_map[i0] = 0;
      }
      else if(e0(1, 1) <= -0.99 && e0(2, 2) <= -0.99)
      {
        pose_id_fwd_kin_map[i0] = 1;
      }
      else if(e0(1, 2) <= -0.99 && e0(2, 1) >= 0.99)
      {
        pose_id_fwd_kin_map[i0] = 2;
      }
      else if(e0(1, 2) >= 0.99 && e0(2, 1) <= -0.99)
      {
        pose_id_fwd_kin_map[i0] = 3;
      }
      else
      {
        ROS_ERROR("Limited Robot Orientation Method not implemented (1).");
      }
    }
    else if(e0(0, 0) <= -0.99)
    {
      if(e0(1, 1) >= 0.99 && e0(2, 2) <= -0.99)
      {
        pose_id_fwd_kin_map[i0] = 4;
      }
      else if(e0(1, 1) <= -0.99 && e0(2, 2) >= 0.99)
      {
        pose_id_fwd_kin_map[i0] = 5;
      }
      else if(e0(1, 2) >= 0.99 && e0(2, 1) >= 0.99)
      {
        pose_id_fwd_kin_map[i0] = 6;
      }
      else if(e0(1, 2) <= -0.99 && e0(2, 1) <= -0.99)
      {
        pose_id_fwd_kin_map[i0] = 7;
      }
      else
      {
        ROS_ERROR("Limited Robot Orientation Method not implemented (2).");
      }
    }
    else if(e0(1, 0) >= 0.99)
    {
      if(e0(0, 1) >= 0.99 && e0(2, 2) <= -0.99)
      {
        pose_id_fwd_kin_map[i0] = 8;
      }
      else if(e0(0, 1) <= -0.99 && e0(2, 2) >= 0.99)
      {
        pose_id_fwd_kin_map[i0] = 9;
      }
      else if(e0(0, 2) >= 0.99 && e0(2, 1) >= 0.99)
      {
        pose_id_fwd_kin_map[i0] = 10;
      }
      else if(e0(0, 2) <= -0.99 && e0(2, 1) <= -0.99)
      {
        pose_id_fwd_kin_map[i0] = 11;
      }
      else
      {
        ROS_ERROR("Limited Robot Orientation Method not implemented (3).");
      }
    }
    else if(e0(1, 0) <= -0.99)
    {
      if(e0(0, 1) >= 0.99 && e0(2, 2) >= 0.99)
      {
        pose_id_fwd_kin_map[i0] = 12;
      }
      else if(e0(0, 1) <= -0.99 && e0(2, 2) <= -0.99)
      {
        pose_id_fwd_kin_map[i0] = 13;
      }
      else if(e0(0, 2) <= -0.99 && e0(2, 1) >= 0.99)
      {
        pose_id_fwd_kin_map[i0] = 14;
      }
      else if(e0(0, 2) >= 0.99 && e0(2, 1) <= -0.99)
      {
        pose_id_fwd_kin_map[i0] = 15;
      }
      else
      {
        ROS_ERROR("Limited Robot Orientation Method not implemented (4).");
      }
    }
    else if(e0(2, 0) >= 0.99)
    {
      if(e0(0, 1) >= 0.99 && e0(1, 2) >= 0.99)
      {
        pose_id_fwd_kin_map[i0] = 16;
      }
      else if(e0(0, 1) <= -0.99 && e0(1, 2) <= -0.99)
      {
        pose_id_fwd_kin_map[i0] = 17;
      }
      else if(e0(0, 2) <= -0.99 && e0(1, 1) >= 0.99)
      {
        pose_id_fwd_kin_map[i0] = 18;
      }
      else if(e0(0, 2) >= 0.99 && e0(1, 1) <= -0.99)
      {
        pose_id_fwd_kin_map[i0] = 19;
      }
      else
      {
        ROS_ERROR("Limited Robot Orientation Method not implemented (5).");
      }
    }
    else if(e0(2, 0) <= -0.99)
    {
      if(e0(0, 1) >= 0.99 && e0(1, 2) <= -0.99)
      {
        pose_id_fwd_kin_map[i0] = 20;
      }
      else if(e0(0, 1) <= -0.99 && e0(1, 2) >= 0.99)
      {
        pose_id_fwd_kin_map[i0] = 21;
      }
      else if(e0(0, 2) >= 0.99 && e0(1, 1) >= 0.99)
      {
        pose_id_fwd_kin_map[i0] = 22;
      }
      else if(e0(0, 2) <= -0.99 && e0(1, 1) <= -0.99)
      {
        pose_id_fwd_kin_map[i0] = 23;
      }
      else
      {
        ROS_ERROR("Limited Robot Orientation Method not implemented (6).");
      }
    }
    else
    {
      ROS_ERROR("Limited Robot Orientation Method not implemented (7).");
    }

    ROS_DEBUG(
      "[%2d/%2d] Sorting end effector pose: [\n% .3f, % .3f, % .3f, "
      "% .3f\n"
      "% .3f, % .3f, % .3f, % .3f\n"
      "% .3f, % .3f, % .3f, % .3f\n"
      "% .3f, % .3f, % .3f, % .3f], to index [%2d/%2d]...",
      i0, this->num_robot_poses, e0(0, 0), e0(0, 1), e0(0, 2), e0(0, 3), e0(1, 0), e0(1, 1), e0(1, 2), e0(1, 3),
      e0(2, 0), e0(2, 1), e0(2, 2), e0(2, 3), e0(3, 0), e0(3, 1), e0(3, 2), e0(3, 3), pose_id_fwd_kin_map[i0],
      this->num_robot_poses);
    this->fwd_kin_pose_id_map[pose_id_fwd_kin_map[i0]] = i0;
  }

  ROS_DEBUG("Printing robot poses in sorted order...");

  for(i0 = 0; i0 < this->num_robot_poses; i0++)
  {
    ROS_DEBUG("[%2d/%2d] -> [%2d/%2d]", i0, this->num_robot_poses, this->fwd_kin_pose_id_map[i0], this->num_robot_poses);
    e0 = this->eig_e_b_vec[this->fwd_kin_pose_id_map[i0]];
    ROS_DEBUG(
      "Pose: [\n% .3f, % .3f, % .3f, % .3f\n"
      "% .3f, % .3f, % .3f, % .3f\n"
      "% .3f, % .3f, % .3f, % .3f\n"
      "% .3f, % .3f, % .3f, % .3f]",
      e0(0, 0), e0(0, 1), e0(0, 2), e0(0, 3), e0(1, 0), e0(1, 1), e0(1, 2), e0(1, 3), e0(2, 0), e0(2, 1), e0(2, 2),
      e0(2, 3), e0(3, 0), e0(3, 1), e0(3, 2), e0(3, 3));
    ROS_DEBUG("\n");
  }
}
