#ifndef PULSE_GUBBI_CLASS_HPP
#define PULSE_GUBBI_CLASS_HPP

#include <cmath>
#include <random>
#include <Eigen/Eigen>

#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/WrenchStamped.h>
#include <tf2_ros/transform_listener.h>

#include "pulse_control_utils/ur5e_base_class.hpp"
#include "pulse_estimation_filters/lkf_velocity_class.hpp"
#include "pulse_vs_msgs/Kalman.h"
#include "pulse_vs_msgs/KalmanArray.h"
#include "pulse_vs_msgs/Segmentation2D.h"
#include "pulse_vs_msgs/Segmentation2DArray.h"

enum VisualServoFsmState
{
  // Initial state of visual servoing system before first message (force-torque reading or point source location) is
  // received
  VS_INITIALIZE = 0,
  // The robot is not in contact with the surface, so discard point source location estimates and move the robot down.
  VS_NO_CONTACT = 1,
  // Laterally center the probe above the closest mid-point of estimated point source location pairs
  VS_TRACK_X = 2,
  // Rotate the probe about the axial dimension to obtain more information about the point source location.
  VS_TRACK_Y = 3,
  // Search for the point source in a pre-determined pattern (typically spiral shaped)
  VS_SEARCH = 4,
  // Hold the visual servoing system stationary indefinitely (either no recent valid poses have been received or th
  // robot just made contact with the surface after lifting off).
  VS_FREEZE = 5
};

/**
 *
 * \brief Visual servoing class to be used with 3D photoacoustic point source localization systems
 *
 */
class GubbiVisualServoingClass : public Ur5eBaseClass
{
protected:
  /**
   * \brief Flag indicating whether or not to ignore elevation displacement from point source localization system
   */
  bool ignore_elevation;

  /**
   * \brief Flag indicating whether or not to consider only a single detection from each image (maximum confidence score)
   */
  bool single_det;

  /**
   * \brief Current contact force along axial dimension of transducer
   */
  double force_f_z_curr;

  /**
   * \brief Desired contact force along axial dimension of transducer
   */
  double force_f_z_goal;

  /**
   * \brief Maximum force reading indicating no contact
   */
  double force_f_z_max;

  /**
   * \brief Threshold of tolerance between current and goal contact forces before commanding movement in axial direction
   */
  double force_f_z_thresh;

  /**
   *
   * \brief Current sum of pairwise distances between LKF tracks currently in the tracking state
   *
   * \details This is to be reset each time an LKF track enters or exits the tracking state or the guidance algorithm
   * switches from one track to another.
   *
   */
  double lkf_track_dist_curr;

  /**
   *
   * \brief Previous sum of pairwise distances between LKF tracks currently in the tracking state
   *
   * \details This is to be reset each time an LKF track enters or exits the tracking state or the guidance algorithm
   * switches from one track to another.
   *
   */
  double lkf_track_dist_prev;

  /**
   *
   * \brief Derivative of sum of pairwise distances between LKF tracks currently in the tracking state
   *
   * \details This is to be reset each time an LKF track enters or exits the tracking state or the guidance algorithm
   * switches from one track to another.
   *
   */
  double lkf_track_dist_deriv;

  /**
   *
   * \brief Coefficient of first order filter to be applied to sum of pairwise distances between LKF tracks currently in
   * the tracking state
   *
   * \details This is required to account for noise in the source detections, which cannot be smoothed out in the LKF.
   *
   */
  double lkf_track_dist_filter_coeff;

  /**
   * \brief Expansion rate of spiral search pattern
   */
  double search_r_dot;

  /**
   * \brief Angular speed of spiral search pattern
   */
  double search_omega;

  /**
   * \brief Number of iterations for which search has been run.
   */
  int search_num_iter;

  /**
   * \brief Sign of direction in which to move wrist 3 for search.
   */
  double search_wrist_3_goal;

  /**
   * \brief Number of iterations for which VS_TRACK_Y has been run.
   */
  int track_y_num_iter;

  /**
   * \brief Sign of direction in which to move wrist 3 for VS_TRACK_Y.
   */
  double track_y_wrist_3_goal;

  /**
   * \brief Homogeneous transform from robot base frame to probe frame
   */
  Eigen::Isometry3d base_pose_probe_eig;

  /**
   * \brief Commanded transform from probe frame to robot base frame computed by guidance algorithm
   */
  Eigen::Isometry3d probe_cmd_pose_base_eig;

  /**
   * \brief Homogeneous transform from probe frame to robot base frame
   */
  Eigen::Isometry3d probe_pose_base_eig;

  /**
   * \brief Pose of center of commanded search pattern in robot base frame
   */
  Eigen::Isometry3d search_center_base_eig;

  /**
   * \brief Index of the LKF track currently being followed by the guidance algorithm
   */
  int guidance_lkf_track_id;

  /**
   * \brief Number of iterations for which the current LKF track has been followed by the guidance algorithm
   */
  int guidance_lkf_track_num_iter;

  /**
   * \brief Counter tracking the number of consecutive iterations of the multi-track LKF with no valid tracks.
   */
  int invalid_pose_counter;

  /**
   * \brief Maximum value of counter of invalid poses while pausing robot movement before starting search
   */
  int pause_max_invalid_count;

  /**
   * \brief Maximum value of counter of invalid poses while search before freezing robot movement
   */
  int search_max_invalid_count;

  /**
   *
   * \brief Network output annotation class index corresponding to point sources
   *
   * \details This is usually zero, but may change depending on the precise manner in which the point source
   * localization system was trained.
   *
   */
  int src_class_id;

  /**
   * \brief Publisher of guidance outputs
   */
  ros::Publisher guidance_pub;

  /**
   * \brief Publisher of multi-track LKF outputs
   */
  ros::Publisher lkf_track_pub;

  /**
   * \brief Subscriber to ROS topic containing network outputs
   */
  ros::Subscriber detection_sub;

  /**
   * \brief Subscriber to ROS topic containing processed FT-sensor outputs
   */
  ros::Subscriber force_torque_sub;

  /**
   * \brief Time at which most recent search was initialized
   */
  ros::Time t_search_start;

  /**
   * \brief ROS timer for robotic control loop
   */
  ros::Timer ctrl_timer;

  /**
   * \brief Name of robot base frame
   */
  std::string base_frame_name;

  /**
   * \brief Name of probe frame
   */
  std::string probe_frame_name;

  int guidance_lkf_dist_buff_len;
  int guidance_lkf_dist_buff_id;
  std::vector<double> guidance_lkf_dist_buff;
  std::vector<double> guidance_lkf_dist_deriv_buff;

  /**
   * \brief Current tracks
   */
  std::vector<pulse_est::LkfVelocity*> lkf_tracks;

  /**
   * \brief TF related variables to propagate estimated states
   */
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener* tf_listener;

  /**
   * \brief Current state of finite state machine controlling guidance algorithm of visual servoing system
   */
  VisualServoFsmState fsm_state;

  /**
   *
   * \brief Compute the sum of pair-wise distances among LKF tracks in the tracking state
   *
   * \details This function computes the sum of pair-wise distances among LKF tracks in the tracking state, applies a
   * first order filter to the computed sum, and computes the derivative of that sum. This derivative is then used by
   * the guidance algorithm to determine which LKF track is to be followed by the probe.
   *
   */
  void computeLkfTrackingDistance(void);

  /**
   *
   * \brief Execute the guidance algorithm
   *
   * \details This function checks the value of the guidance FSM state and calls the corresponding to function to track
   * the point source, search for the point source, or hold the probe stationary.
   *
   */
  void executeGuidanceFsm(void);

  /**
   *
   * \brief Extract the source positions in the base frame from the input message
   *
   * \arg[in] det_msg Message containing detections output by photoacoustic point source localization system
   * \arg[out] det_pos_base_mat [4 x N] matrix containing point source localization outputs in robot base frame
   * \arg[out] num_src_det Number of point source localization outputs contained in `det_pos_base_mat` (i.e., positive
   * and negative source elevation displacements wherever applicable)
   *
   */
  void extractSourcePositions(
    pulse_vs_msgs::Segmentation2DArray det_msg, Eigen::MatrixXd& det_pos_base_mat, int& num_src_det);

  /**
   *
   * \brief Hold the visual servoing system stationary indefinitely (no recent valid poses have been received).
   *
   */
  void guidanceFsmFreeze(void);

  /**
   *
   * \brief Publish the initial transform between the probe and base frame
   *
   */
  void guidanceFsmInitialize(void);

  /**
   *
   * \brief Transition checks for the finite state machine controlling the guidance algorithms
   *
   * \details This function assumes that we are receiving messages of the type `KalmanArray` with estimation filter
   * FSM states defined in the package `pulse_estimation_filters`. Briefly, the estimation filter outputs one or more
   * tracks, each of which can exist one of the states LKF_TRACKING, LKF_INITIALIZE, LKF_MISSED_ONE, LKF_MISSED_TWO,
   * or LKF_DELETE, in decreasing order of priority. The guidance FSM transitions between states based on the number
   * of estimation filter tracks in each state defined above [2].
   *
   */
  void guidanceFsmTransitionCheck(void);

  /**
   *
   * \brief Search for the point source
   *
   */
  void guidanceFsmSearch(void);

  /**
   *
   * \brief Track and center the probe above the estimated point source location.
   *
   */
  void guidanceFsmTrack(void);

  /**
   *
   * \brief Publish the state and output of the guidance algorithm
   *
   */
  void publishTrackingGuidance(void);

  /**
   *
   * \brief Publish the states and outputs of the current LKF tracks
   *
   */
  void publishLkfTracks(void);

  /**
   *
   * \brief Predict the current possible source positions and velocities using a multi-track LKF
   *
   * \arg[in] det_pos_base_mat [4 x N] matrix containing point source localization outputs in robot base frame
   * \arg[in] num_src_det Number of point source localization outputs contained in `det_pos_base_mat` (i.e., positive
   * and negative source elevation displacements wherever applicable)
   *
   */
  void predictSourcePositions(void);

  /**
   *
   * \brief Reset the first order filter applied to the pairwise distances of LKF tracks in the tracking state
   *
   * \details This function resets the filter and the derivative of the sum of pair-wise distances among LKF tracks in
   * the tracking state. This is required if:
   * (1) one or more LKF tracks enters or exits the tracking state, or
   * (2) the guidance algorithm switches from one LKF track to another.
   *
   */
  void resetLkfTrackingDistanceFilter(void);

  /**
   *
   * \brief Reset the buffer containing the outputs of the LKF pairwise distance filter used by the guidance algorithm
   *
   */
  void resetGuidanceTrackingDistanceBuffer(void);

  /**
   *
   * \brief Update the current possible source positions and velocities tracked with a multi-track LKF
   *
   * \arg[in] det_pos_base_mat [4 x N] matrix containing point source localization outputs in robot base frame
   * \arg[in] num_src_det Number of point source localization outputs contained in `det_pos_base_mat` (i.e., positive
   * and negative source elevation displacements wherever applicable)
   *
   */
  void updateSourcePositions(Eigen::MatrixXd det_pos_base_mat, int num_src_det);

public:
  /**
   *
   * \brief Construct an object of the type `GubbiVisualServoingClass`
   *
   * \arg[in] nh The ROS node handler
   *
   */
  GubbiVisualServoingClass(
    ros::NodeHandle& nh, std::string action_server_name, std::string ee_name, double delta_t, double joint_omega_max,
    double joint_alpha_max);

  /**
   *
   * \brief Callback function for control loop timer
   *
   * \arg[in] e ROS timer event (unused)
   *
   */
  void controlTimerCallback(const ros::TimerEvent& e);

  /**
   *
   * \brief Callback function for subscriber to network outputs
   *
   * \arg[in] det_msg Message containing detections output by photoacoustic point source localization system
   *
   */
  void detectionSubscriberCallback(pulse_vs_msgs::Segmentation2DArray det_msg);

  /**
   *
   * \brief Callback function for subscriber to processed FT-sensor outputs
   *
   * \arg[in] ft_msg Message containing processed force-torque readings in probe frame
   *
   */
  void forceTorqueSubscriberCallback(geometry_msgs::WrenchStamped ft_msg);
};

#endif /* PULSE_GUBBI_CLASS_HPP */
