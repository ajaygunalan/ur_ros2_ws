/**
 * \file yu_bias_est_srom_class.hpp
 *
 * \brief Estimate the gravitational forces and sensor bias as described by Yu
 * et al. in the paper "Bias Estimation and Gravity Compensation for
 * Wrist-Mounted Force/Torque Sensor" published in IEEE Sensors in 2022.
 *
 * \details This class implements the Special Robot Orientation Method (SROM)
 * for the robot pose selection and gravitational force estimation.
 *
 * \author Mardava Gubbi <mgubbi1@jhu.edu>
 */
#ifndef YU_BIAS_EST_SROM_CLASS_HPP
#define YU_BIAS_EST_SROM_CLASS_HPP

#include "eigen_conversions/eigen_msg.h"
#include "geometry_msgs/WrenchStamped.h"
#include "tf_conversions/tf_eigen.h"

#include "pulse_force_estimation/yu_bias_est_base_class.hpp"
#include "pulse_force_estimation/YuSetBiases.h"

#define SROM_NUM_FT_ROBOT_POSES 24

// TODO: Replace this with a pose set derived from the original pose of the
// robot to accommodate different visual servoing setups. The primary
// constraint would be to avoid tangling the cables from the FT sensor and the
// ultrasound probe.
/* const double srom_pose_joint_angles_mat[SROM_NUM_FT_ROBOT_POSES][6] = {
  { 0.0, -M_PI / 2.0, -M_PI / 2.0, -M_PI / 2.0, M_PI / 2.0, -M_PI },
  { 0.0, -M_PI / 2.0, -M_PI / 2.0, -M_PI / 2.0, M_PI / 2.0, -M_PI / 2.0 },
  { 0.0, -M_PI / 2.0, -M_PI / 2.0, -M_PI / 2.0, M_PI / 2.0, 0.0 },
  { 0.0, -M_PI / 2.0, -M_PI / 2.0, -M_PI / 2.0, M_PI / 2.0,
    M_PI / 2.0 },  // Downward
  { 0.0, -M_PI / 2.0, -M_PI / 2.0, 0.0, M_PI / 2.0, -M_PI / 2.0 },
  { 0.0, -M_PI / 2.0, -M_PI / 2.0, 0.0, M_PI / 2.0, 0.0 },
  { 0.0, -M_PI / 2.0, -M_PI / 2.0, 0.0, M_PI / 2.0, M_PI / 2.0 },
  { 0.0, -M_PI / 2.0, -M_PI / 2.0, 0.0, M_PI / 2.0, M_PI },  // Forward
  { 0.0, -M_PI / 2.0, -M_PI / 2.0, 0.0, 0.0, M_PI / 2.0 },
  { 0.0, -M_PI / 2.0, -M_PI / 2.0, 0.0, 0.0, 0.0 },
  { 0.0, -M_PI / 2.0, -M_PI / 2.0, 0.0, 0.0, -M_PI / 2.0 },
  { 0.0, -M_PI / 2.0, -M_PI / 2.0, 0.0, 0.0, -M_PI },  // Rightward
  { 0.0, -M_PI / 2.0, -M_PI / 2.0, 0.0, -M_PI / 2.0, -M_PI / 2.0 },
  { 0.0, -M_PI / 2.0, -M_PI / 2.0, 0.0, -M_PI / 2.0, 0.0 },
  { 0.0, -M_PI / 2.0, -M_PI / 2.0, 0.0, -M_PI / 2.0, M_PI / 2.0 },
  { 0.0, -M_PI / 2.0, -M_PI / 2.0, 0.0, -M_PI / 2.0, M_PI },  // Backward
  { 0.0, -M_PI / 2.0, -M_PI / 2.0, -M_PI / 2.0, -M_PI / 2.0, M_PI },
  { 0.0, -M_PI / 2.0, -M_PI / 2.0, -M_PI / 2.0, -M_PI / 2.0, M_PI / 2.0 },
  { 0.0, -M_PI / 2.0, -M_PI / 2.0, -M_PI / 2.0, -M_PI / 2.0, 0.0 },
  { 0.0, -M_PI / 2.0, -M_PI / 2.0, -M_PI / 2.0, -M_PI / 2.0,
    -M_PI / 2.0 },  // Upward
  { -M_PI / 2.0, -M_PI / 2.0, -M_PI / 2.0, 0.0, -M_PI / 2.0, M_PI / 2.0 },
  { -M_PI / 2.0, -M_PI / 2.0, -M_PI / 2.0, 0.0, -M_PI / 2.0, 0.0 },
  { -M_PI / 2.0, -M_PI / 2.0, -M_PI / 2.0, 0.0, -M_PI / 2.0, -M_PI / 2.0 },
  { -M_PI / 2.0, -M_PI / 2.0, -M_PI / 2.0, 0.0, -M_PI / 2.0, -M_PI }  // Leftward
}; */

/**
 * \brief Extend the basic gravitational force and sensor bias estimation class
 * to estimate the parameters in question using the Special Robot Orientation
 * Method.
 *
 * \details 24 orientations of the robot are generated to ensure that the axes
 * of the robot end effector are parallel to the axes of the robot base frame.
 * These orientations are then used to estimate the gravitational force of the
 * tool, which should remain the same across orientations.
 */
class YuBiasEstSromClass : public ft::YuBiasEstBaseClass
{
protected:
  /**
   * \brief Map from indices of robot poses in Fig. 3 of Yu et al. to robot
   * poses attained during bias estimation process.
   *
   * \details The bias estimation process using the SROM method involves moving
   * the robot from one pose to another. The order of poses is chosen to
   * minimize the tangling of cables, and as such, rarely matches the order
   * listed in Fig. 3 of Yu et al. This map and the function
   * `sortPosesSpecialRobotOrientationMethod` correct for that mismatch.
   */
  int fwd_kin_pose_id_map[SROM_NUM_FT_ROBOT_POSES];

  /**
   * \brief Estimate the gravitational forces in the robot base frame as
   * described in Section III-A1 of Yu et al.
   */
  void estimateGravitationalForcesRobotBaseFrame(void) override;

  /**
   * \brief Compute the joint angles to be achieved by the robot during the bias
   * estimation procedure.
   */
  void setRobotCmdJointAngles(void) override;

  /**
   * \brief Generate an index vector sorting the acquired poses and force-torque
   * sensor readings according to the Special Robot Orientation Method described
   * in Fig. 3 in Yu et al.
   *
   * \details The bias estimation process using the SROM method involves moving
   * the robot from one pose to another. The order of poses is chosen to
   * minimize the tangling of cables, and as such, rarely matches the order
   * listed in Fig. 3 of Yu et al. This function and the map
   * `fwd_kin_pose_id_map` correct for that mismatch.
   */
  void sortPosesSpecialRobotOrientationMethod(void);

public:
  /**
   * \brief Constructor for YuBiasEstSromClass
   *
   * \arg[in] nh NodeHandle object
   * \arg[in] base_frame_name The name of the robot base frame `B`
   * \arg[in] ee_frame_name The name of the robot end effector frame `E`
   * \arg[in] sensor_frame_name The name of the sensor frame `S`
   */
  YuBiasEstSromClass(
    ros::NodeHandle& nh, std::string base_frame_name = "base_link",
    std::string ee_frame_name = "tool0",
    std::string sensor_frame_name = "netft_link1");
};

#endif /* YU_BIAS_EST_SROM_CLASS_HPP */
