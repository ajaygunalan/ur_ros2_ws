/**
 * \file ur5e_gubbi_sweep_class.hpp
 *
 * \brief Class to sweep UR5e in pre-determined pattern across a uniform
 * surface.
 *
 * \details This class is derived from Ur5ePosControlClass.
 *
 * \author Mardava Gubbi <mgubbi1@jhu.edu>
 */
#ifndef PULSE_UR5E_GUBBI_SWEEP_CLASS_HPP
#define PULSE_UR5E_GUBBI_SWEEP_CLASS_HPP

#include <std_msgs/Int8.h>

#include "pulse_common_utils/eigen_math_utils.hpp"
#include "pulse_control_utils/ur5e_pos_control_class.hpp"

enum Ur5eGubbiSweepFsmState
{
  FSM_SELECT_INITIAL_POSE = 0,
  FSM_GENERATE_POSE_GRID = 1,
  FSM_MOVE_ROBOT = 2,
  FSM_WAIT_FOR_IMAGES = 3,
  FSM_WAIT_FOR_NEXT_SWEEP = 4
};

class Ur5eGubbiSweepClass : public Ur5ePosControlClass
{
protected:
  /**
   * \brief Flag indicating whether or not images corresponding to the current
   * robot pose have been acquired by the ultrasound scanner
   */
  bool img_acquired;

  /**
   * \brief Flag indicating whether or not new user inputs are available to be
   * processed by the sweep FSM
   */
  bool new_user_input;

  /**
   * \brief Data provided by the user (via keyboard input)
   */
  char user_input_data;

  /**
   * \brief The displacement between consecutive poses along the elevation
   * dimension of the probe
   */
  double elev_step_size;

  /**
   * \brief The displacement between consecutive poses along the lateral
   * dimension of the probe
   */
  double lat_step_size;

  /**
   * \brief Pose held by the robot while waiting for the next sweep operation to
   * begin
   */
  Eigen::Isometry3d next_sweep_wait_pose;

  /**
   * \brief Initial pose of the sweep operation, selected by the user
   */
  Eigen::Isometry3d sweep_initial_pose;

  /**
   * \brief Initial pose of the sweep operation, selected by the user
   */
  Eigen::Isometry3d sweep_img_pose_curr;

  /**
   * \brief Number of poses in grid along elevation dimension of probe
   */
  int num_elev_steps;

  /**
   * \brief Number of poses in grid along lateral dimension of probe
   */
  int num_lat_steps;

  /**
   * \brief Number of poses in generated grid
   */
  int num_poses;

  /**
   * \brief Pose counter for current sweep operation
   */
  int pose_counter;

  /**
   * \brief Publisher for FSM state (primarily for debugging purposes)
   */
  ros::Publisher fsm_pub;

  /**
   * \brief Subscriber for image acquisition state (from ultrasound scanner)
   */
  ros::Subscriber img_acq_sub;

  /**
   * \brief Subscriber for user inputs via keyboard
   */
  ros::Subscriber user_input_sub;

  /**
   * \brief Timer for FSM execution
   */
  ros::Timer fsm_timer;

  /**
   * \brief Current FSM state
   */
  Ur5eGubbiSweepFsmState fsm_state;

public:
  /**
   * \brief Constructor for Ur5eGubbiSweepClass
   *
   * \arg[in] nh NodeHandle object
   * \arg[in] action_server_name Name of action server for robot motion commands
   * \arg[in] ee_name Name of frame corresponding to robot end effector
   * \arg[in] delta_t Time step of individual robot commands
   * \arg[in] joint_omega_max Maximum joint angular velocity to be commanded
   * \arg[in] joint_alpha_max Maximum joint angular acceleration to be commanded
   * \arg[in] robot_ns Namespace corresponding to robot related information
   */
  Ur5eGubbiSweepClass(
    ros::NodeHandle& nh, std::string action_server_name, std::string ee_name,
    double delta_t, double ee_vel_max = 0.1, double k_p = 1.0, double k_i = 0.0,
    double k_d = 0.0, double joint_omega_max = 0.1,
    double joint_alpha_max = M_PI / 2.0, double t_err_max = 1e-4,
    int n_steps_max = 100, std::string robot_ns = "/robot");

  /**
   * \brief Generate the grid of imaging poses from the initial user-set pose
   *
   * TODO: Make the spacing configurable.
   */
  void fsmGeneratePoseGrid(void);

  /**
   * \brief Move the robot to the pose specified by the current system state
   */
  void fsmMoveRobot(void);

  /**
   * \brief Wait for the user to move the robot to the desired initial pose
   */
  void fsmSelectInitialPose(void);

  /**
   * \brief Wait for response from scanner indicating images have been saved
   */
  void fsmWaitForImages(void);

  /**
   * \brief Wait for a user input indicating the beginning of the next sweep
   */
  void fsmWaitForNextSweep(void);

  /**
   * \brief Callback function for FSM execution
   *
   * \arg[in] e The timer event (not used in function but required by ROS)
   */
  void fsmTimerCallback(const ros::TimerEvent& e);

  /**
   * \brief Callback function for image acquisition subscription
   *
   * \arg[in] msg The message containing a flag indicating whether or not the
   * images corresponding to the current robot pose have been acquired
   */
  void imgAcquisitionSubscriberCallback(const std_msgs::Int8 msg);

  /**
   * \brief Callback function for user input subscription
   *
   * \arg[in] msg The message containing the user input
   */
  void userInputSubscriberCallback(const std_msgs::Int8 msg);
};

#endif /* PULSE_UR5E_GUBBI_SWEEP_CLASS_HPP */
