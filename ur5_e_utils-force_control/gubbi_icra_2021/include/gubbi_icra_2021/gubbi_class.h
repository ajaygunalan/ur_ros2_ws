/**
 *
 * \file gubbi_class.h
 *
 * \brief Class for ultrasound or photoacoustic-based visual servoing system developed by members of the PULSE Lab.
 *
 * \details The visual servoing node subscribes to a topic containing information regarding the target position and
 * validity, and moves the robot according to the received messages.
 *
 * \references
 * 1. Gubbi and Bell, "Deep Learning-Based Photoacoustic Visual Servoing: Using Outputs from Raw Sensor Data as Inputs
 * to a Robot Controller", IEEE ICRA 2021.
 *
 * \author Mardava Gubbi <mgubbi1@jhu.edu>
 *
 */
#ifndef PULSE_VISUAL_SERVOING_GUBBI_CLASS_H
#define PULSE_VISUAL_SERVOING_GUBBI_CLASS_H

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int16.h>

#include "pulse_control_utils/ur5e_base_class.hpp"
#include "pulse_common_utils/eigen_string_utils.hpp"

class Ur5eGubbiVisualServoingClass : public Ur5eBaseClass
{
protected:
  /*
   * \brief Error threshold for end effector pose above which to move the end effector.
   */
  double ee_error_threshold;
  /*
   * \brief Maximum spatial displacement of robot end effector per time step.
   */
  double path_planning_delta_x_max;
  /*
   * \brief Spiral expansion rate of the search pattern.
   */
  double search_r_dot;
  /*
   * \brief Spiral angular speed of the search pattern.
   */
  double search_omega;
  /*
   * \brief Error vector to be minimized.
   */
  Eigen::MatrixXd e_p_vec;
  /*
   * \brief Integral of error vector used by PID controller.
   */
  Eigen::MatrixXd e_i_vec;
  /*
   * \brief Derivative gains for PID controller.
   */
  Eigen::MatrixXd k_d_mat;
  /*
   * \brief Integral gains for PID controller.
   */
  Eigen::MatrixXd k_i_mat;
  /*
   * \brief Proportional gains for PID controller.
   */
  Eigen::MatrixXd k_p_mat;
  Eigen::MatrixXd v_cmd;
  /*
   * \brief Current target position in the probe frame computed from the most recent photoacoustic image.
   */
  geometry_msgs::PoseStamped tgt_pose_msg;
  /*
   * \brief Current FSM state.
   */
  int fsm_state;
  /*
   * \brief Counter of invalid poses provided to the node.
   */
  int invalid_pose_counter;
  /*
   * \brief Max value of invalid pose counter in search state.
   *
   * After the invalid pose counter exceeds this value, the FSM transitions to the stationary state.
   */
  int search_max_count;
  /*
   * \brief Header sequence number of previous target pose probe message.
   *
   * This is used by the FSM to verify whether or not a message has been parsed before.
   */
  int fsm_trans_tgt_pose_seq_prev;
  int fsm_follow_tgt_pose_seq_prev;
  /*
   * \brief Max value of invalid pose counter in wait state.
   *
   * After the invalid pose counter exceeds this value, the FSM transitions to the search state.
   */
  int wait_max_count;
  /*
   * \brief Publisher for the tracking error (measured from the target pose topic)
   */
  ros::Publisher error_pub;
  /*
   * \brief Publisher for the FSM state
   */
  ros::Publisher fsm_state_pub;
  /*
   * \brief Subscriber object for the target pose topic
   */
  ros::Subscriber tgt_pose_sub;
  /*
   * \brief Time at which previous message was received.
   *
   * This is used because Gazebo functions on a separate time from UTC, and using bag files from real experiments on
   * Gazebo for post testing does not work without this.
   */
  ros::Time fsm_trans_tgt_pose_stamp_prev;
  ros::Time fsm_follow_tgt_pose_stamp_prev;
  /*
   * \brief Time at which the latest search pattern was begun.
   */
  ros::Time search_t_start;
  /*
   * \brief Timer used to run the finite state machine for visual servoing
   */
  ros::Timer fsm_timer;

  /**
   *
   * \brief Execute the finite state machine of the visual servoing system.
   *
   * \arg[in] ev The timer event
   *
   */
  void fsmExecution(const ros::TimerEvent& ev);

  /**
   *
   * \brief Move the robot towards the computed valid target position.
   *
   */
  void fsmFollowTarget(void);

  /**
   *
   * \brief Temporarily keep the robot stationary while waiting for a valid target to be obtained.
   *
   * Also store the current robot pose as the center of the search pattern and count the number of invalid target
   * positions for transition purposes.
   *
   */
  void fsmHoldAndWait(void);

  /**
   *
   * \brief Search for the target in a spiral pattern around the previously computed center.
   *
   */
  void fsmSearchForTarget(void);

  /**
   *
   * \brief Keep the robot stationary.
   *
   */
  void fsmStationary(void);

  /**
   *
   * \brief Transition checks for the finite state machine
   *
   */
  void fsmTransitionCheck(void);

  /**
   *
   * \brief Callback function for the target pose topic
   *
   * \arg[in] msg Input message containing timestamped pose of target
   *
   */
  void targetPoseCallback(geometry_msgs::PoseStamped msg);

public:
  /**
   *
   * \brief Construct an object of the type Ur5eGubbiVisualServoingClass.
   *
   * \arg[in] nh The node handler
   * \arg[in] action_server_name The name of the action server to connect to
   * \arg[in] search_ns The namespace of the search parameters
   * \arg[in] path_planning_ns The namespace of the path planning parameters
   * \arg[in] end_effector_ns The namespace of the end effector parameters
   * \arg[in] joint_ns The namespace of the joint parameters
   * \arg[in] tgt_loc_ns The namespace of the target localization topic to subscribe to
   *
   */
  Ur5eGubbiVisualServoingClass(
    ros::NodeHandle& nh, std::string action_server_name, std::string ee_name = "p42v_link1", double delta_t = 0.1,
    double joint_omega_max = (M_PI / 10.0), double search_r_dot = 0.005, double search_omega = (M_PI / 20.0),
    int wait_max_count = 10, int search_max_count = 60, std::string tgt_loc_ns = "/deep_learning");
};

#endif /* PULSE_VISUAL_SERVOING_GUBBI_CLASS_H */
