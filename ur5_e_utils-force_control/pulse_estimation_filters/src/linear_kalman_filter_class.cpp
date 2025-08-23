/**
 *
 * \file linear_kalman_filter_class.cpp
 *
 * \brief Linear Kalman Filter base class
 *
 * \references
 * 1. Bar-Shalom, Yaakov, and Xiao-Rong Li. Multitarget-multisensor tracking: principles and techniques. Vol. 19.
 * Storrs, CT: YBS publishing, 1995.
 * 2. Bar-Shalom, Yaakov, X. Rong Li, and Thiagalingam Kirubarajan. Estimation with applications to tracking and
 * navigation: theory algorithms and software. John Wiley & Sons, 2004.
 * 3. Gubbi and Bell, IEEE T-UFFC 2024
 *
 * \author Mardava Gubbi <mgubbi1@jhu.edu>
 *
 */
#include "pulse_estimation_filters/linear_kalman_filter_class.hpp"

pulse_est::LinearKalmanFilter::LinearKalmanFilter(double smooth_coeff)
  : lkf_fsm_state(LKF_INITIALIZE), track_length(0), smooth_coeff(smooth_coeff), guidance_following(false)
{
}

/*
 * \brief Check the input measurement to see if it lies within the defined gate using the measurement prediction
 * covariance matrix
 */
bool pulse_est::LinearKalmanFilter::gate(Eigen::Vector3d meas_vec, double& meas_pred_dist)
{
  Eigen::Vector3d meas_err_vec;

  // Compute the innovation (i.e., difference between measurement and predicted measurement).
  meas_err_vec = meas_vec - this->meas_pred_vec;

  // Compute the distance between the actual and predicted measurements.
  meas_pred_dist = meas_err_vec.transpose() * this->meas_pred_cov_mat.inverse() * meas_err_vec;

  ROS_DEBUG(
    "|| [%.2f, %.2f, %.2f] - [%.2f, %.2f, %.2f] || = %.2f [>=< %.2f]", 1e3 * meas_vec(0), 1e3 * meas_vec(1),
    1e3 * meas_vec(2), 1e3 * this->meas_pred_vec(0), 1e3 * this->meas_pred_vec(1), 1e3 * this->meas_pred_vec(2),
    1e3 * meas_pred_dist, 1e3 * this->gate_thresh);

  // Compare the computed distance with the gate threshold.
  return (meas_pred_dist < this->gate_thresh);
}

/*
 * \brief Perform the prediction step of the LKF
 */
void pulse_est::LinearKalmanFilter::computePredictedState(void)
{
  // Predict the next state and measurement vectors with the corresponding covariance matrices.
  this->x_pred = this->proc_a_mat * this->x_curr;
  this->p_pred = (this->proc_a_mat * this->p_curr * this->proc_a_mat.transpose()) + this->proc_noise_cov_mat;
  this->meas_pred_vec = this->meas_h_mat * this->x_pred;
  this->meas_pred_cov_mat = (this->meas_h_mat * this->p_pred * this->meas_h_mat.transpose()) + this->meas_noise_cov_mat;

  // Compute the Kalman gain as well, as this can be done without an actual measurement.
  this->lkf_gain_mat = this->p_pred * this->meas_h_mat.transpose() * this->meas_pred_cov_mat.inverse();
}

/*
 * \brief Perform the update step of the LKF with the associated source detection
 */
void pulse_est::LinearKalmanFilter::updateWithMeasurement(Eigen::Vector3d meas_vec)
{
  // Update the state, the state covariance matrix, and the corresponding time using the input measurement.
  ROS_DEBUG("Updating with measurement...");
  this->x_curr = this->x_pred + (this->lkf_gain_mat * (meas_vec - this->meas_pred_vec));
  this->p_curr = this->p_pred - (this->lkf_gain_mat * this->meas_h_mat * this->p_pred);
  this->track_length++;

  if(this->track_length > 1)
  {
    // We need more than one measurement to properly track both position and velocity.
    this->lkf_fsm_state = LinearKalmanFsmState::LKF_TRACKING;
  }
  else
  {
    // No operation
  }

  this->t_predict_curr = ros::Time::now();
}

/*
 * \brief Perform the update step of the LKF without an associated source detection input measurement
 */
void pulse_est::LinearKalmanFilter::updateWithoutMeasurement(void)
{
  ROS_WARN("Updating without measurement...");
  this->x_curr = this->x_pred;
  this->p_curr = this->p_pred;

  switch(this->lkf_fsm_state)
  {
    case LinearKalmanFsmState::LKF_TRACKING:
      this->lkf_fsm_state = LinearKalmanFsmState::LKF_MISSED_ONE;
      this->track_length++;
      break;
    case LinearKalmanFsmState::LKF_MISSED_ONE:
      this->lkf_fsm_state = LinearKalmanFsmState::LKF_MISSED_TWO;
      this->track_length++;
      break;
    default:
      this->lkf_fsm_state = LinearKalmanFsmState::LKF_DELETE;
  }

  this->t_predict_curr = ros::Time::now();
}
