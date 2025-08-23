/**
 * \file ur5e_move_group_visual_servoing_class.hpp
 *
 * \brief Visual servoing system using `MoveGroupInterface`.
 *
 * \author Mardava Gubbi <mgubbi1@jhu.edu>
 */
#ifndef UR5E_MOVE_GROUP_VISUAL_SERVOING_CLASS_HPP
#define UR5E_MOVE_GROUP_VISUAL_SERVOING_CLASS_HPP

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
#include "pulse_control_utils/ur5e_move_group_if_class.hpp"

namespace ur5e_moveit_visual_servoing
{
enum MoveitVisualServoingFsmState
{
  MOTION_TEST = -1,
  STATIONARY = 0,
  PRE_CONTACT = 1,
  TRACKING = 2,
  SEARCHING = 3
};

class Ur5eMoveGroupVisualServoingClass : public mgi::Ur5eMoveGroupInterfaceClass
{
protected:
  /*
   * \brief Flag indicating whether or not to use force control during visual servoing
   */
  bool force_control;

  /*
   * \brief Time period of timer for finite state machine (FSM) execution
   */
  double fsm_period;

  /*
   * \brief Latest force-torque readings after bias compensation and conversion
   * to probe frame
   */
  Eigen::Matrix<double, 6, 1> ft_reading_probe;

  /*
   * \brief Enumerated FSM state
   */
  MoveitVisualServoingFsmState fsm_state;

  /*
   * \brief Subscriber to force-torque sensor readings after bias correction and
   * gravity compensation and transformation to the probe frame.
   */
  ros::Subscriber ft_proc_probe_sub;

  /*
   * \brief Subscriber to warning from force-torque sensor regarding excess force,
   * which should trigger shutdown of visual servoing and emergency brakes on robot.
   */
  ros::Subscriber ft_sensor_cancel_sub;

  /*
   * \brief Subscriber to bounding boxes output by amplitude- or learning-based
   * point source localization techniques.
   */
  ros::Subscriber point_source_bbox_sub;

  /*
   * \brief Timer in which robot motion planning is to be performed and
   * execution is to be triggered asynchronously.
   */
  ros::Timer fsm_timer;

  /*
   * \brief Name of frame `S` in which force-torque sensor readings are received
   */
  const std::string sensor_frame_name;

  /*
   * \brief Transform from sensor frame to tool frame.
   */
  tf::StampedTransform tf_tool_sensor;

public:
  /**
   * \brief Construct an object of the type Ur5eMoveGroupVisualServoingClass
   *
   * \arg[in] nh The ROS node handler
   * \arg[in] force_control A flag indicating whether or not to use force control
   * \arg[in] sensor_frame_name The name of the sensor frame (see CAD
   * drawing of F/T sensor if required, typically `/netft_link1`)
   * \arg[in] probe_frame_name The name of the probe frame `P`
   */
  Ur5eMoveGroupVisualServoingClass(
    ros::NodeHandle& nh, bool force_control = true,
    std::string sensor_frame_name = "netft_link1",
    std::string probe_frame_name = "p42v_link1");

  /**
   * \brief Execute the FSM at a fixed periodicity
   *
   * \arg[in] e (unused) timer event
   */
  void fsmTimerCallback(const ros::TimerEvent& e);

  /**
   * \brief Compute the force error for each processed force-torque reading
   *
   * TODO: Determine the frame in which the processed FT readings are provided
   *
   * \arg[in] msg A message containing the processed FT readings after bias and
   * gravity compensation
   */
  void ftProcProbeCallback(const geometry_msgs::WrenchStamped& msg);

  /**
   * \brief Compute the position of the point source filtered by confidence scores
   *
   * \details Tracking across multiple frames must happen here and not on the GPU
   * server, as this node is aware of robot kinematics, which allows us to resolve
   * everything in the fixed robot base frame to reduce tracking errors.
   *
   * \arg[in] msg A message containing bounding boxes of detected sources and artifacts
   */
  void bboxCallback(const vision_msgs::Detection2DArray& msg);
};
}  // namespace ur5e_moveit_visual_servoing

#endif /* UR5E_MOVE_GROUP_VISUAL_SERVOING_CLASS_HPP */
