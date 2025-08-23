/**
 * \file yu_bias_est_base_class.hpp
 *
 * \brief Estimate the gravitational forces and sensor bias as described by Yu et al. [1].
 *
 * \details This class implements everything except for the robot pose selection and gravity estimation. These two tasks
 * depend on whether the Special Robot Orientation Method (SROM) or the Limited Robot Orientation Method (LROM) are to
 * be used. Separate derived classes are provided for each.
 *
 * References:
 * 1. Yu, Yongqiang, Ran Shi, and Yunjiang Lou. "Bias estimation and gravity compensation for wrist-mounted force/torque
 * sensor." IEEE Sensors Journal 22.18 (2021): 17625-17634.
 *
 * \author Mardava Gubbi <mgubbi1@jhu.edu>
 */
#ifndef YU_BIAS_EST_BASE_CLASS_HPP
#define YU_BIAS_EST_BASE_CLASS_HPP

// Header files other than ROS
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <Eigen/Geometry>
#include <highfive/highfive.hpp>

// ROS code header files
#include <ros/ros.h>
#include <ros/package.h>

// ROS message files
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/WrenchStamped.h>

// Moveit header files
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// TF header files
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include "pulse_common_utils/eigen_math_utils.hpp"
#include "pulse_control_utils/ur5e_move_group_if_class.hpp"
#include "pulse_force_estimation/YuSetBiases.h"

#define NUM_FT_SAMPLES_PER_POSE 10

namespace ft
{
enum YuBiasEstFsmState
{
  ERROR = -1,
  PRE_INIT = 0,
  INITIALIZE = 1,
  MOVE_ROBOT = 2,
  ACQUIRE_DATA = 3,
  UNTANGLE = 4,
  ESTIMATE_BIAS = 5,
  COMPLETED = 6,
};

class YuBiasEstBaseClass : public mgi::Ur5eMoveGroupInterfaceClass
{
protected:
  /*
   * \brief Time period of timer for finite state machine (FSM) execution
   */
  double fsm_period;

  /**
   *
   * \brief Estimate of rotation from tool gravity frame `G` to robot base frame `B` (see Eq. (6), Fig. 2 in [1]).
   *
   * \details The tool gravity frame `G` is defined as having its axes parallel to the world frame and origin coincident
   * with the center of mass of the tool. This frame translates as the robot moves, but does not rotate with respect to
   * the world frame. This matrix estimated the constant rotation between the robot base frame and the tool gravity
   * frame, with non-zero components along the x- and y-directions.
   */
  Eigen::Matrix3d rot_b_g;

  /**
   * \brief Estimate of rotation from robot end effector frame `E` to sensor frame `S` (see Eq. (6) in [1]).
   */
  Eigen::Matrix3d rot_s_e;

  /**
   * \brief Latest raw F/T sensor reading in sensor frame `S`
   */
  Eigen::Matrix<double, 6, 1> ft_reading_curr;

  /**
   * \brief Estimate of gravitational force in robot base frame `B` (see Eq. (9) in [1]).
   *
   * \details This force is constant in the base frame, which does not rotate with respect to the world frame.
   */
  Eigen::Vector3d f_grav_b;

  /**
   * \brief Estimate of force component of sensor bias in sensor frame `S` (see Eq. (2) in [1]).
   *
   * \details This is referred to as $^{s}F_{0}$ in [1].
   */
  Eigen::Vector3d f_bias_s;

  /**
   * \brief Estimate of torque component of sensor bias in sensor frame `S` (see Eq. (3) in [1]).
   *
   * \details This is referred to as $^{s}T_{0}$ in [1].
   */
  Eigen::Vector3d t_bias_s;

  /**
   * \brief Estimate of the position of the origin of the tool gravity frame `G` in the sensor frame `S` (see Eq. (5) in [1]).
   *
   * \details The origin of the tool gravity frame `G` corresponds to the center of gravity of the tool.
   */
  Eigen::Vector3d p_grav_s;

  /**
   * \brief Index of current force-torque reading
   */
  int ft_reading_id;

  /**
   * \brief Number of robot poses to be used for data acquisition during the bias estimation process.
   */
  int num_robot_poses;

  /**
   * \brief Total number of force-torque sensor readings to be used for bias estimation
   */
  int num_ft_readings_total;

  /**
   * \brief Index of current robot pose
   */
  int robot_pose_id;

  /*
   * \brief ROS debug publisher
   */
  ros::Publisher debug_tf_pub;
  ros::Publisher debug_wrench_pub;

  /**
   * \brief ROS service client required to update the parameters of the bias compensation node
   */
  ros::ServiceClient bias_update_client;

  /**
   * \brief ROS subscriber to raw F/T sensor readings in sensor frame `S`
   */
  ros::Subscriber netft_raw_sensor_sub;

  /**
   * \brief ROS timer to execute FSM for bias estimation
   */
  ros::Timer fsm_timer;

  /**
   * \brief Name of robot base frame `B`
   *
   * \details This is typically `base_link` for a UR robot.
   */
  std::string base_frame_name;

  /**
   * \brief Name of robot end effector frame `E`
   *
   * \details This frame is physically located on the robot (e.g., `tool0`), not an attached tool (e.g., `p42v_link1`).
   */
  std::string ee_frame_name;

  /**
   * \brief Name of sensor base frame `S`
   *
   * \details This frame should be the frame in which raw sensor readings are received (e.g., `netft_link1` for the ATI
   * Gamma sensor). Refer to the engineering drawing of the sensor (PDF) which contains this information.
   */
  std::string sensor_frame_name;

  /**
   * \brief Joint position of robot at beginning of bias estimation process.
   *
   * \details This joint position is intended to be close to the default pose for the subsequent visual servoing operation.
   * The robot is expected to be free of obstacles in the quadrant formed by the `base_link` and `shoulder_link` joints.
   */
  std::vector<double> joint_q_start;

  /**
   * \brief Matrix of joint angles to be reached during bias estimation.
   *
   * \details This matrix depends on the initial pose of the robot, as well as the selected algorithm (SROM/LROM)
   */
  std::vector<std::vector<double>> joint_q_cmd_mat;

  /**
   * \brief Matrix of force-torque sensor readings collected at various robot poses to be used for bias estimation
   */
  std::vector<std::vector<double>> ft_readings_vec;

  /**
   * \brief Vector of forward kinematic matrices (transforms from robot base frame to end effector frame rather than the
   * other way round, hence the `_e_b_`) collected at each robot pose to be used for bias estimation
   */
  std::vector<Eigen::Isometry3d> eig_e_b_vec;

  /**
   * \brief Current state of the finite state machine (FSM) implemented to perform bias estimation.
   */
  YuBiasEstFsmState fsm_state;

  /**
   *
   * \brief Estimate the sensor force bias by solving the force compensation problem as described in Section III-B of Yu
   * et al. [1]
   *
   */
  void estimateForceBiasSensorFrame(void);

  /**
   *
   * \brief Estimate the sensor bias, gravitational force, and robot installation bias. Upload these estimates to the
   * bias compensation node via a ROS service call.
   *
   */
  void estimateSensorBiasGravitationalForceUpdate(void);

  /**
   *
   * \brief Estimate the sensor torque bias via the torque identification model described in Section III-C of
   * Yu et al. [1]
   *
   */
  void estimateTorqueBiasSensorFrame(void);

  /**
   *
   * \brief Estimate the rotation from the tool gravity frame `G` to the robot base frame `B` as described in Section
   * III-D of Yu et al. [1]
   *
   */
  void estimateRobotInstallationBias(void);

  /**
   *
   * \brief Estimate the rotation from the robot end effector frame `E` to the sensor frame `S` as described in Section
   * III-B of Yu et al. [1]
   *
   */
  void estimateRotationEndEffectorSensor(void);

  /**
   *
   * \brief Estimate the gravitational forces in the robot base frame as described in Section III-A of Yu et al. [1]
   *
   * \details This is a virtual function to be implemented separately for LROM and SROM techniques (see Yu et al. [1]).
   *
   */
  virtual void estimateGravitationalForcesRobotBaseFrame(void);

  /**
   *
   * \brief Compute the joint angles to be achieved by the robot during the bias estimation procedure.
   *
   * \details This is a virtual function to be implemented separately for LROM and SROM techniques (see Yu et al. [1]).
   *
   */
  virtual void setRobotCmdJointAngles(void);

  /**
   *
   * \brief Write the estimated sensor bias and gravitational force variables to an HDF5 file.
   *
   */
  void writeEstimationsToFile(void);

public:
  /**
   *
   * \brief Constructor for YuBiasEstBaseClass
   *
   * \arg[in] nh NodeHandle object
   * \arg[in] base_frame_name The name of the robot base frame `B`
   * \arg[in] ee_frame_name The name of the robot end effector frame `E`
   * \arg[in] sensor_frame_name The name of the sensor frame `S`
   *
   */
  YuBiasEstBaseClass(
    ros::NodeHandle& nh, std::string base_frame_name = "base_link", std::string ee_frame_name = "tool0",
    std::string sensor_frame_name = "netft_link1", std::string probe_frame_name = "p42v_link1");

  /**
   *
   * \brief Callback function for FSM timer
   *
   * \arg[in] e ROS timer event variable (typically not used in function)
   *
   */
  void fsmTimerCallback(const ros::TimerEvent& e);

  /**
   *
   * \brief Callback function for subscriber to raw F/T sensor readings in sensor frame `S`
   *
   * \arg[in] msg The most recent raw FT sensor reading in the sensor frame `S`
   *
   */
  void netftRawSensorSubscriberCallback(geometry_msgs::WrenchStamped msg);
};
}  // namespace ft

#endif /* YU_BIAS_EST_BASE_CLASS_HPP */
