/**
 * \file ur5e_base_class.hpp
 *
 * \brief Base class to interface with UR5e with action client-based movement
 * commands and MoveIt to obtain forward kinematics and other details.
 *
 * \author Mardava Gubbi <mgubbi1@jhu.edu>
 * \author Jessica Su <jsu30@jhu.edu>
 */
#ifndef PULSE_UR5E_BASE_CLASS_HPP
#define PULSE_UR5E_BASE_CLASS_HPP

#include <boost/format.hpp>
#include <stdexcept>

#include <ros/console.h>
#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <netft_utils/Cancel.h>

#include "pulse_common_utils/eigen_math_utils.hpp"

using namespace boost;

/**
 *
 * \brief Base class to interface to a UR5e
 *
 * \details This class serves the following purposes:
 * 1. Interface to a UR5e robot
 * 2. Subscribe to the topic /joint_states
 * 3. Store the robot model and robot state objects from the MoveIt library
 * 4. Facilitate the computation of the inverse kinematics and manipulator
 * jacobian. TODO: See if fmauch has the identical solution (still might not be
 * optimal considering the required modifications to the URDF to accommodate the
 * end effector).
 *
 * \author Mardava Gubbi
 */
class Ur5eBaseClass : public actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
{
protected:
  /*
   * \brief Follow joint trajectory message
   */
  control_msgs::FollowJointTrajectoryGoal follow_joint_traj_goal_msg;
  /*
   * \brief Time-step for path planning
   */
  double delta_t;
  /*
   * \brief Maximum joint angular acceleration
   */
  double joint_alpha_max;
  /*
   * \brief Maximum joint angular velocity
   */
  double joint_omega_max;

  /*
   * \brief Denavit-Hartenberg parameters for UR5e
   * TODO: Add webpage link here.
   */
  double ur5e_a2;
  double ur5e_a3;
  double ur5e_d4;
  double ur5e_d5;
  double ur5e_d6;
  /*
   * \brief Current value of body Jacobian
   */
  Eigen::MatrixXd body_jacobian;
  /*
   * \brief Current joint angular accelerations
   */
  Eigen::VectorXd joint_alpha;
  /*
   * \brief Current joint angular velocities
   */
  Eigen::VectorXd joint_omega_curr;
  /*
   * \brief Previous joint angular velocities
   */
  Eigen::VectorXd joint_omega_prev;
  /*
   * \brief Current joint states
   */
  Eigen::VectorXd joint_q_curr;
  /*
   * \brief Previous joint states
   */
  Eigen::VectorXd joint_q_prev;
  /*
   * \brief Sequence index for joint trajectory messages.
   */
  int joint_traj_msg_id;
  /*
   * \brief Object containing joint names, used to compute forward kinematics
   * and Jacobian
   */
  moveit::core::JointModelGroup* joint_model_group;
  /*
   * \brief Object containing details of robot model
   */
  moveit::core::RobotModelPtr robot_model;
  /*
   * \brief Object containing current state of the robot
   * TODO: Determine how to update this in real-time.
   */
  moveit::core::RobotStatePtr robot_state;
  /*
   * \brief Object containing planning scene monitor
   */
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
  /*
   * \brief Object used to load robot model
   */
  robot_model_loader::RobotModelLoaderPtr robot_model_loader;
  /*
   * \brief NodeHandle object
   */
  ros::NodeHandle nh;
  /*
   * \brief Subscriber to topic providing current joint positions
   */
  ros::Subscriber joint_states_sub;
  /*
   * \brief Subscriber to the force-torque sensor cancel topic (see
   * pulse_netft_sensor_driver)
   */
  ros::Subscriber netft_cancel_sub;
  /*
   * \brief Name of link corresponding to robot end effector
   */
  std::string ee_name;
  /*
   * \brief Publisher for the commanded velocity of the robot end effector
   */
  ros::Publisher cart_vel_pub;
  /*
   * \brief Time stamp corresponding to current joint states, angular
   * velocities, and accelerations
   */
  ros::Time joint_t_curr;
  /*
   * \brief Time stamp corresponding to previous joint states, angular
   * velocities, and accelerations
   */
  ros::Time joint_t_prev;
  /*
   * \brief Vector containing names of UR5e robot joints
   */
  std::vector<std::string> joint_names;

public:
  /**
   * \brief Constructor for Ur5eBaseClass
   *
   * \arg[in] nh NodeHandle object
   * \arg[in] action_server_name Name of action server for robot motion commands
   * \arg[in] ee_name Name of frame corresponding to robot end effector
   * \arg[in] delta_t Time step of individual robot commands
   * \arg[in] joint_omega_max Maximum joint angular velocity to be commanded
   * \arg[in] joint_alpha_max Maximum joint angular acceleration to be commanded
   * \arg[in] robot_ns Namespace corresponding to robot related information
   */
  Ur5eBaseClass(
    ros::NodeHandle& nh, std::string action_server_name, std::string ee_name, double delta_t,
    double joint_omega_max = 0.1, double joint_alpha_max = M_PI / 2.0, std::string robot_ns = "/robot");

  /**
   * \brief Fetch the spatial Jacobian
   *
   * \return Flag indicating whether or not the Jacobian was fetched
   */
  bool getSpatialJacobian(Eigen::MatrixXd& m);

  /**
   * \brief Fetch the spatial Jacobian
   *
   * \return Flag indicating whether or not the Jacobian was fetched
   */
  bool getSpatialJacobian(Eigen::MatrixXd& m, std::string link_name);

  /**
   * \brief Fetch the inverse of the adjoint matrix
   *
   * \return the inverse adjoint matrix
   */
  Eigen::MatrixXd getAdjointInverse(void);

  /**
   * \brief Create 3x3 skew symmetric matrix of a vector
   *
   * \return the skew symmetric matrix
   */
  Eigen::MatrixXd skewSymmetric(Eigen::VectorXd v);

  /**
   * \brief Fetch the global link transformation
   *
   * \return The global link transformation
   */
  const Eigen::Isometry3d& getGlobalLinkTransform(void);

  /**
   * \brief Fetch the name of the robot model
   *
   * \return The robot model name
   */
  const std::string getRobotModelFrame(void);

  /**
   * \brief Get the variable names of the joint model group object
   *
   * \return A vector containing the variable names
   */
  const std::vector<std::string>& getJointModelGroupVariableNames(void);

  /**
   * \brief Get the state update frequency of the planning scene monitor
   *
   * \return The state update frequency of the planning scene monitor [Hz]
   */
  double getPlanningSceneMonitorStateUpdateFrequency(void);

  /**
   * \brief Update the joint group positions
   */
  void copyJointGroupPositions(std::vector<double>& v);

  /**
   *
   * \brief Callback function for joint states topic from robot.
   *
   * \arg[in] msg The message containing the current joint state information
   */
  void jointStatesCallback(const sensor_msgs::JointState& msg);

  /**
   * \brief Callback function for cancel command from force-torque sensor
   *
   * \arg[in] msg The message containing the command to cancel.
   */
  void netftCancelCallback(const netft_utils::Cancel& msg);

  /**
   *
   * \brief Send the generated joint trajectory goal to the action server.
   *
   */
  void sendGoal(void);

  /**
   *
   * \brief Send the generated joint trajectory goal to the action server and
   * wait for the response.
   *
   */
  void sendGoalAndWait(void);

  /**
   * \brief Set the robot to move with the desired Cartesian velocity for
   * `delta_t` seconds.
   *
   * \details Use the manipulator Jacobian to compute the joint velocity vector
   * and populate a message to move the robot by that joint velocity.
   * TODO: Check for collisions.
   *
   */
  void setBodyCartesianVelocity(Eigen::VectorXd v);

  /**
   * \brief Set the robot to move with the desired Cartesian velocity for
   * `delta_t` seconds.
   *
   * \details Use the manipulator Jacobian to compute the joint velocity vector
   * and populate a message to move the robot by that joint velocity.
   * TODO: Check for collisions.
   *
   */
  void setSpatialCartesianVelocity(Eigen::VectorXd v);

  /**
   * \brief Move the robot to the provided joint vector
   *
   * \arg[in] q The joint vector to which the robot must be moved.
   */
  void setJointPosition(std::vector<double> q);

  /**
   * \brief Move the robot with the desired joint velocity for one time-step
   *
   * \details The time-step is defined using a member variable of the
   * Ur5eBaseClass class.
   *
   * \arg[in] q The joint vector to which the robot must be moved.
   */
  void setJointVelocity(std::vector<double> q_dot);
};

#endif /* PULSE_UR5E_BASE_CLASS_HPP */
