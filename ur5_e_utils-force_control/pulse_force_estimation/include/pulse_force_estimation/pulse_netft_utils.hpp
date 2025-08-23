/**
 *
 * \file pulse_netft_utils.hpp
 *
 * \brief Utilities for the NET-FT sensor
 *
 * \details Ideally this class would be derived from the NetftUtils class in the
 * package `netft_utils`. However, that class is impossible to derive in its
 * default state, with no pre-defined libraries and a main function right in the
 * class implementation. I do not intend to modify that code, so this copy has
 * been made for that purpose.
 *
 * \author Mardava Gubbi <mgubbi1@jhu.edu>
 */
#ifndef PULSE_NETFT_UTILS_HPP
#define PULSE_NETFT_UTILS_HPP

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/WrenchStamped.h"
#include "netft_utils/SetBias.h"
#include "netft_utils/SetMax.h"
#include "netft_utils/SetThreshold.h"
#include "netft_utils/SetToolData.h"
#include "netft_utils/SetFilter.h"
#include "netft_utils/GetDouble.h"
#include "netft_utils/Cancel.h"
#include "lpfilter.h"
#include <math.h>

/**
 * This program takes force/torque data and applies transforms to usable data
 */
class PulseNetftUtils
{
public:
  PulseNetftUtils(ros::NodeHandle nh);
  ~PulseNetftUtils();

  void initialize(void);
  void
  setUserInput(std::string world, std::string ft, double force, double torque);
  void update(void);

protected:
  // ROS node handler
  ros::NodeHandle nh;

  // Low pass filter and related variables
  LPFilter* lpf_h;
  bool lpf_is_on;
  double lpf_delta_t;
  double lpf_cutoff_freq;
  bool lpf_new;

  // Transform listener
  tf::TransformListener* tf_listener;

  // Current transform from FT sensor frame to world frame
  tf::StampedTransform tf_tool_to_world;
  std::string tf_world_frame;
  std::string tf_tool_frame;

  // Wrenches used to hold force/torque and bias data
  // Wrench containing the current bias data in tool frame
  geometry_msgs::WrenchStamped bias_tool;
  // Wrench containing the bias at a measurement pose (to measure the weight)
  geometry_msgs::WrenchStamped weight_bias;
  // Wrench containing the raw data from the FT sensor (world frame)
  geometry_msgs::WrenchStamped raw_data_world;
  // Wrench containing the raw data from the FT sensor (tool frame)
  geometry_msgs::WrenchStamped raw_data_tool;
  // Wrench containing the processed (biased and thresholded) data (world frame)
  geometry_msgs::WrenchStamped proc_data_world;
  // Wrench containing the processed (biased and thresholded) data (tool frame)
  geometry_msgs::WrenchStamped proc_data_tool;
  // Wrench of all zeros for convenience
  geometry_msgs::WrenchStamped zero_wrench;
  // Wrench containing thresholds
  geometry_msgs::WrenchStamped threshold;

  // Variables associated with gravity compensation
  // Flag indicating whether or not latest readings from sensor were gravity
  // compensated.
  bool grav_comp_new;
  // Flag indicating whether or not readings from sensor are to be gravity
  // compensated.
  bool grav_comp_is_on;
  // Measured weight of payload. Used in gravity compensation
  double grav_comp_payload_weight;
  // The z-coordinate to payload center-of-mass (sensor frame) (x- and y-
  // coordinates assumed to be zero)
  double grav_comp_payload_lever_arm;

  // Variables associated with fixed orientation biasing
  // Flag indicating whether or not the sensor is to be biased.
  bool bias_is_on;
  // Flag indicating whether or not latest readings from sensor were biased.
  bool bias_new;

  // Variables used to monitor FT violations and send cancel move messages
  // Number of times to send cancel message when max force is exceeded
  static const int CANCEL_COUNT_MAX = 5;
  // Number of cycles to wait after cancel message before sending again
  static const int CANCEL_WAIT_MAX = 100;
  // Counter for times to send cancel message when max force is exceeded
  int cancel_count;
  // Counter of cycles to wait after cancel message before sending again
  int cancel_wait;
  netft_utils::Cancel cancel_msg;

  // Default max force limit to send cancel when FT is biased
  double force_max_b;
  // Default max torque limit to send cancel when FT is biased
  double torque_max_b;
  // Default max force limit to send cancel when FT is unbiased
  double force_max_u;
  // Default max torque limit to send cancel when FT is unbiased
  double torque_max_u;

  // ROS subscribers
  ros::Subscriber raw_data_sub;

  // ROS publishers
  ros::Publisher netft_raw_world_data_pub;
  ros::Publisher netft_proc_world_data_pub;
  ros::Publisher netft_proc_tool_data_pub;
  ros::Publisher netft_cancel_pub;

  // ROS services
  ros::ServiceServer bias_service;
  ros::ServiceServer grav_comp_service;
  ros::ServiceServer set_max_service;
  ros::ServiceServer theshold_service;
  ros::ServiceServer weight_bias_service;
  ros::ServiceServer get_weight_service;
  ros::ServiceServer filter_service;

  ros::Timer update_timer;

  // Callback methods

  /**
   *
   * \brief Callback function to run when a new raw data point is received
   *
   * \arg[in] data The message object containing the raw sensor data
   */
  void netftCallback(const geometry_msgs::WrenchStamped::ConstPtr& data);

  /**
   *
   * \brief Compute and store the sensor bias
   *
   * \details Set the readings from the sensor to zero at this instant and
   * continue to apply the bias on future readings. This doesn't account for
   * gravity i.e. it will not change if the sensor's orientation changes. Run
   * this method when the sensor is stationary to avoid inertial effects.
   *
   * \arg[in] req The request object containing the bias command
   * \arg[out] res The response object containing the status of the executed
   * function
   */
  bool setFixedOrientationBias(
    netft_utils::SetBias::Request& req, netft_utils::SetBias::Response& res);

  /**
   *
   * \brief Compute and store the payload weight and lever arm length
   *
   * \details Calculate the payload's mass and center of mass so gravity can be
   * compensated for, even as the sensor changes orientation. It is assumed that
   * the center of mass of the payload is located on the z-axis of the sensor
   * and the z-axis of the world frame points upwards. Run this method when the
   * sensor is stationary to avoid inertial effects. Also ensure that the
   * payload is horizontal for accurate weight computation.
   *
   * \arg[in] req The request object containing the gravity compensation command
   * \arg[out] res The response object containing the status of the executed
   * function
   */
  bool setGravityCompensation(
    netft_utils::SetBias::Request& req, netft_utils::SetBias::Response& res);

  /**
   *
   * \brief Set the maximum values for unbiased force and torque values
   *
   * \arg[in] req The request object containing the maximum values
   * \arg[out] res The response object containing the status of the executed
   * function
   */
  bool setMaxValues(
    netft_utils::SetMax::Request& req, netft_utils::SetMax::Response& res);

  /**
   *
   * \brief Set the weight bias
   *
   * \arg[in] req The request object containing the weight bias
   * \arg[out] res The response object containing the status of the executed
   * function
   */
  bool setWeightBias(
    netft_utils::SetBias::Request& req, netft_utils::SetBias::Response& res);

  /**
   *
   * \brief Get the weight bias
   *
   * \arg[in] req The request object
   * \arg[out] res The response object containing the weight bias
   */
  bool getWeight(
    netft_utils::GetDouble::Request& req, netft_utils::GetDouble::Response& res);

  /**
   *
   * \brief Set the threshold
   *
   * \arg[in] req The request object containing the threshold
   * \arg[out] res The response object containing the status of the executed
   * function
   */
  bool setThreshold(
    netft_utils::SetThreshold::Request& req,
    netft_utils::SetThreshold::Response& res);

  /**
   *
   * \brief Set the filter parameters
   *
   * \arg[in] req The request object containing the filter parameters
   * \arg[out] res The response object containing the status of the executed
   * function
   */
  bool setFilter(
    netft_utils::SetFilter::Request& req, netft_utils::SetFilter::Response& res);

  // Convenience methods
  static void copyWrench(
    geometry_msgs::WrenchStamped& in, geometry_msgs::WrenchStamped& out,
    geometry_msgs::WrenchStamped& bias);

  /**
   *
   * \brief Apply a threshold, zeroing out values with magnitudes lower than the
   * threshold.
   *
   * \arg[in|out] value The value to be thresholded
   * \arg[in] thresh The threshold to be applied
   */
  static void applyThresholdDouble(double& value, double thresh);

  /**
   *
   * \brief Apply a threshold, zeroing out values with magnitudes lower than the
   * threshold.
   *
   * \arg[in|out] wrench The wrench to be thresholded
   * \arg[in] thresh The threshold to be applied
   */
  static void applyThresholdWrenchStamped(
    geometry_msgs::WrenchStamped& data, geometry_msgs::WrenchStamped& thresh);

  void transformFrame(
    geometry_msgs::WrenchStamped in_data,
    geometry_msgs::WrenchStamped& out_data, char target_frame);

  void checkForceLimitViolation(void);
  void printForceLimitViolation(
    double f_mag, double f_max, double t_mag, double t_max);

  void updateCallback(const ros::TimerEvent& e);
};

#endif /* PULSE_NETFT_UTILS_HPP */
