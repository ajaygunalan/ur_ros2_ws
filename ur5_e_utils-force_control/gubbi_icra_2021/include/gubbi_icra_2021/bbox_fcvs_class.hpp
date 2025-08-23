/**
 *
 * \file bbox_fcvs_class.hpp
 *
 * \brief Visual servoing system using bounding box inputs and force control
 *
 * \details This system receives `vision_msgs/Detection2DArray` messages
 * containing bounding boxes for point source locations and desired tool frame
 * `T` transformations using a transform listener for force control. These
 * inputs are then processed and combined to determine the servoing commands to
 * be sent to the robot via a `MoveGroupInterface` object. These commands are
 * generated in a finite state machine (FSM) running at a faster rate than the
 * pulse repetition rate of the laser (10 Hz).
 *
 * References:
 * 1. Gubbi, Mardava R., and Bell, Muyinatu A. Lediju. "Deep learning-based
 * photoacoustic visual servoing: Using outputs from raw sensor data as inputs
 * to a robot controller." Proceedings of the IEEE International Conference on
 * Robotics and Automation 2021.
 * 2. Untitled robotics force control submission
 *
 * \author Mardava Gubbi <mgubbi1@jhu.edu>
 */
#ifndef PULSE_BBOX_FCVS_CLASS_HPP
#define PULSE_BBOX_FCVS_CLASS_HPP

// ROS code header files
#include <ros/ros.h>

// ROS message files
#include <vision_msgs/Detection2DArray.h>

#include "pulse_control_utils/ur5e_move_group_if_class.hpp"

namespace vs
{
enum MoveitVisualServoingFsmState
{
  MOTION_TEST = -1,
  STATIONARY = 0,
  PRE_CONTACT = 1,
  TRACKING = 2,
  SEARCHING = 3
};

/*
 * \brief Inverse of laser pulse repetition rate (10 Hz for the Opotek laser).
 */
static const double LASER_PULSE_REP_TIME = 0.1;

/*
 * \brief Maximum allowable speed of source between frames. [m/s]
 *
 * \details This value is computed from the previous consistency check [1] which
 * computed the mean source position across 5 frames and allowed for a deviation
 * of 1 cm on either side of the mean, resulting in a maximum deviation of 2 cm
 * across 500 ms (laser pulse repetition rate of 100 ms) of the source position
 * between frames.
 *
 */
static const double MAX_SRC_SPEED_THRESH = 0.4;

class BoundingBoxForceControlVisualServoingClass
  : public mgi::Ur5eMoveGroupInterfaceClass
{
protected:
  /*
   * \brief Flag indicating whether or not to use force control during visual servoing
   */
  bool force_control;

  /*
   * \brief Time period of timer for finite state machine (FSM) execution
   *
   * TODO: See if this needs to be a member variable.
   */
  double fsm_period;

  /*
   * \brief Confidence score threshold for source detections from point source
   * localization system.
   */
  double src_conf_score_thresh;

  /*
   * \brief Maximum duration between consecutive bounding box inputs to be
   * considered for a spatio-temporal consistency check.
   */
  double temporal_validity_thresh;

  /*
   * \brief Vector of most recent valid source position in the probe frame `B`.
   *
   * \details We store this in the base frame to simplify the global
   * spatio-temporal consistency check.
   */
  Eigen::Vector3d eig_valid_src_b_curr;

  /*
   * \brief Counter of consecutive messages from the point source localization
   * system which do not contain a source detection.
   */
  int no_valid_src_counter;

  /*
   * \brief Enumerated FSM state
   */
  MoveitVisualServoingFsmState fsm_state;

  /*
   * \brief ROS subscriber to point source location information
   */
  ros::Subscriber pt_src_det_sub;

  /*
   * \brief Time-stamp of most recent valid source detection
   *
   * \details Validity of the source detection is determined by the
   * spatio-temporal consistency check and confidence score thresholds, if
   * provided.
   */
  ros::Time valid_src_time_curr;

  /*
   * \brief ROS timer to execute FSM for visual servoing
   */
  ros::Timer fsm_timer;

  /*
   * \brief Physical dimensions of images corresponding to bounding box inputs
   * in order [z, x]. [m]
   */
  std::vector<double> img_dim_m;

  /*
   * \brief Physical dimensions of pixels in images corresponding to bounding
   * box inputs in order [z, x]. [m]
   */
  std::vector<double> pixel_dim_m;

  /*
   * \brief Name of desired tool frame `T` to which probe is to be moved for
   * force control purposes
   *
   * \details See `pulse_force_control/yoshikawa_force_control_class.hpp`
   */
  std::string tool_frame_name;

public:
  /**
   *
   * \brief Construct an object of the type BoundingBoxForceControlVisualServoingClass.
   *
   * \arg[in] nh The node handler
   * \arg[in] probe_frame_name The name of the probe frame `P`
   * \arg[in] tool_frame_name The name of the desired tool frame `T` generated
   * by the force control algorithm
   *
   */
  BoundingBoxForceControlVisualServoingClass(
    ros::NodeHandle& nh, std::string probe_frame_name = "p42v_link1",
    std::string tool_frame_name = "p42v_link1_fc");

  /**
   *
   * \brief Callback function for point source bounding box messages.
   *
   * \details Tracking across multiple frames must happen here and not on the GPU
   * server, as this node is aware of robot kinematics, which allows us to resolve
   * everything in the fixed robot base frame to reduce tracking errors.
   *
   * \arg[in] msg Message containing bounding box and object hypothesis information
   *
   */
  void pointSourceBboxCallback(const vision_msgs::Detection2DArray& msg);

  /**
   *
   * \brief Callback function for FSM timer event.
   *
   * \arg[in] e ROS timer event variable (typically not used in function)
   *
   */
  void fsmTimerCallback(const ros::TimerEvent& e);
};
}  // namespace vs

#endif /* PULSE_BBOX_FCVS_CLASS_HPP */
