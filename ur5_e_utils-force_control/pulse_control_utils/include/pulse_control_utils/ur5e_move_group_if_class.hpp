/**
 *
 * \file ur5e_move_group_if_class.hpp
 *
 * \brief Class for motion planning using `MoveGroupInterface`
 *
 * \author Mardava Gubbi <mgubbi1@jhu.edu>
 *
 */
#ifndef PULSE_UR5E_MOVE_GROUP_IF_CLASS_HPP
#define PULSE_UR5E_MOVE_GROUP_IF_CLASS_HPP

// Header files other than ROS
#include <boost/format.hpp>
#include <Eigen/Geometry>

// ROS code header files
#include <ros/ros.h>

// ROS message files
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <vision_msgs/Detection2DArray.h>

// Moveit header files
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// TF header files
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include "pulse_common_utils/eigen_math_utils.hpp"

namespace mgi
{
/*
 * \brief Planning group to be supplied to `MoveGroupInterface` object
 */
static const std::string PLANNING_GROUP = "manipulator";

class Ur5eMoveGroupInterfaceClass
{
protected:
  /*
   * \brief Object containing joint names and other joint-related information
   */
  const moveit::core::JointModelGroup* jmg_obj;

  /*
   * \brief MoveGroupInterface object to interface with MoveGroup node
   */
  moveit::planning_interface::MoveGroupInterface* mgi_obj;

  /*
   * \brief Object to add scene details for collision avoidance during motion
   * planning
   */
  moveit::planning_interface::PlanningSceneInterface* psi_obj;

  /*
   * \brief ROS node handle
   */
  ros::NodeHandle nh;

  /*
   * \brief Name of reference frame for Moveit-based motion planning
   * (typically `base_link`)
   */
  std::string moveit_ref_frame_name;

  /*
   * \brief Name of probe frame `P` in which photoacoustic images and related
   * data are received
   */
  std::string probe_frame_name;

  /**
   * \brief Vector of joint names to be used for motion planning.
   */
  std::vector<std::string> joint_names_vec;

  /*
   * \brief TF listener to look up transformations between frames
   */
  tf::TransformListener tf_listener;

public:
  /**
   *
   * \brief Construct an object of the type Ur5eMoveGroupInterfaceClass.
   *
   * \arg[in] nh The node handler
   * \arg[in] probe_frame_name The name of the probe frame `P`
   *
   */
  Ur5eMoveGroupInterfaceClass(
    ros::NodeHandle& nh, std::string probe_frame_name = "p42v_link1");

  /**
   *
   * \brief Plan and execute motion to bring the robot to the given
   * `joint_q_vec`.
   *
   * \arg[in] joint_q_vec Target joint angle vector
   *
   * \returns A Moveit error code indicating the outcome of setting the target,
   * planning the robot motion, and executing the motion plan
   *
   */
  moveit::core::MoveItErrorCode
  moveRobotJointTarget(std::vector<double>& joint_q_vec);

  /**
   *
   * \brief Plan and execute motion to bring the robot to the given `eig_goal_p`
   * relative to the current robot probe frame `P`.
   *
   * \details This could be referred to as body frame-based motion planning. The
   * only potential argument against this phrasing is that everything is being
   * converted to the base frame `B` under the hood, and it is position-based
   * motion planning rather than velocity-based planning, which is where the
   * terms `body` and `spatial` more frequently occur.
   *
   * \arg[in] eig_goal_p Target pose relative to current robot pose.
   *
   * \returns A Moveit error code indicating the outcome of setting the target,
   * planning the robot motion, and executing the motion plan
   *
   */
  moveit::core::MoveItErrorCode
  moveRobotProbeFrame(Eigen::Isometry3d eig_goal_p);
};
}  // namespace mgi

#endif /* PULSE_UR5E_MOVE_GROUP_IF_CLASS_HPP */
