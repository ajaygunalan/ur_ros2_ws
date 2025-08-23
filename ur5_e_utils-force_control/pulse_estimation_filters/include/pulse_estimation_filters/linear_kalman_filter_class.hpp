#ifndef PULSE_LINEAR_KALMAN_FILTER_CLASS_HPP
#define PULSE_LINEAR_KALMAN_FILTER_CLASS_HPP

#include <string>

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <tf2_eigen/tf2_eigen.h>

#include "pulse_estimation_filters/chi_squared.hpp"

namespace pulse_est
{
enum LinearKalmanFsmState
{
  // Newly initialized LKF track, to be deleted if not associated with a detection in the next input
  LKF_INITIALIZE = 0,
  // Valid LKF track which was associated with a detection in the previous input
  LKF_TRACKING = 1,
  // Valid LKF track which was not associated with a detection in the previous input
  LKF_MISSED_ONE = 2,
  // Valid LKF track which was not associated with a detection in the previous two consecutive inputs
  LKF_MISSED_TWO = 3,
  // Invalid LKF track which is to be deleted
  LKF_DELETE = -1
};

class LinearKalmanFilter
{
protected:
  /**
   *
   * \brief Gate threshold `gamma`
   *
   * \details Eq. (2.3.2-2) in [1]
   *
   */
  double gate_thresh;

  /**
   *
   * \brief Smoothing filter coefficient between 0 and 1.
   *
   * \details A higher value implies more trust in the measurements
   *
   */
  double smooth_coeff;

  /**
   *
   * \brief Measurement matrix `H`
   *
   * \details Eq. (5.2.1-1) in [2]
   *
   */
  Eigen::MatrixXd meas_h_mat;

  /**
   * \brief Covariance matrix of measurement noise
   */
  Eigen::MatrixXd meas_noise_cov_mat;

  /**
   * \brief Measurement prediction covariance matrix `S` in equation S = H * P * H' + R
   */
  Eigen::MatrixXd meas_pred_cov_mat;

  /**
   * \brief Matrix `A` in equation x_k = A * x_{k-1} + w_{k-1}
   */
  Eigen::MatrixXd proc_a_mat;

  /**
   * \brief Covariance matrix of process noise
   */
  Eigen::MatrixXd proc_noise_cov_mat;

  /**
   * \brief Gain matrix `K` of LKF
   */
  Eigen::MatrixXd lkf_gain_mat;

  /**
   * \brief Matrix containing predicted state covariance of LKF
   */
  Eigen::MatrixXd p_pred;

  /**
   * \brief Vector containing previous state of LKF (used by smoothing filter)
   */
  Eigen::MatrixXd x_prev;

  /**
   * \brief Measurement prediction vector
   */
  Eigen::MatrixXd meas_pred_vec;

  /**
   * \brief Time at which current prediction/measurement was made
   *
   * \details This is required to incorporate velocity and acceleration into the process model.
   */
  ros::Time t_predict_curr;

  /**
   * \brief Time at which current smoothed output was generated
   *
   * \details This is required to incorporate velocity and acceleration into the process model.
   */
  ros::Time t_smooth_curr;

  /**
   *
   * \brief Compute predicted state
   *
   * \details This assumes all matrices are set to the expected values.
   *
   */
  void computePredictedState(void);

public:
  /**
   * \brief Flag indicating whether this LKF track is being followed by the guidance module
   */
  bool guidance_following;

  /**
   * \brief Matrix containing current state covariance of LKF
   */
  Eigen::MatrixXd p_curr;

  /**
   * \brief Vector containing current state of LKF
   */
  Eigen::MatrixXd x_curr;

  /**
   * \brief Vector containing predicted state of LKF
   */
  Eigen::MatrixXd x_pred;

  /**
   * \brief Number of iterations for which Kalman filter has been running in the valid state
   */
  int track_length;

  /**
   * \brief Current state of LKF track
   */
  LinearKalmanFsmState lkf_fsm_state;

  /**
   *
   * \brief Construct an object of the type `LinearKalmanFilter`
   *
   */
  LinearKalmanFilter(double smooth_coeff);

  /**
   *
   * \brief Check the input measurement to see if it lies within the current gate defined using the measurement
   * prediction covariance matrix
   *
   * \arg[in] meas_vec Vector containing measurement in base frame
   * \arg[out] meas_pred_dist Distance between input and predicted measurements scaled by measurement prediction
   * covariance matrix
   *
   */
  bool gate(Eigen::Vector3d meas_vec, double& meas_pred_dist);

  /**
   *
   * \brief Perform the prediction step of the LKF
   *
   * \details Each derived class is expected to set the corresponding matrices prior to calling `computePredictedState`.
   *
   */
  virtual void predict(void) = 0;

  /**
   *
   * \brief Perform the update step of the LKF with the associated source detection
   *
   * \arg[in] meas_in_gate Flag indicating whether or not a measurement exists within the current gate of the LKF
   * \arg[in] meas_vec Vector containing measurement in base frame if it exists within the current gate, zero otherwise
   *
   */
  void updateWithMeasurement(Eigen::Vector3d meas_vec);

  /**
   *
   * \brief Perform the update step of the LKF without an associated source detection input measurement
   *
   */
  void updateWithoutMeasurement(void);
};
}  // namespace pulse_est

#endif /* PULSE_LINEAR_KALMAN_FILTER_CLASS_HPP */
