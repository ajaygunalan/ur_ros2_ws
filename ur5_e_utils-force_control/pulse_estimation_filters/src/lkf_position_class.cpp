/**
 *
 * \file lkf_position_class.cpp
 *
 * \brief Linear Kalman Filter estimating only position without velocity or acceleration
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
#include "pulse_estimation_filters/lkf_position_class.hpp"

pulse_est::LkfPosition::LkfPosition(Eigen::Vector3d x_in, double smooth_coeff) : LinearKalmanFilter(smooth_coeff)
{
  Eigen::Vector3d meas_noise_std_vec;
  int i0;
  int i1;

  // Initialize measurement noise covariance matrix. Use numbers corresponding to System A in [3] with a scaling factor
  // of 4.0 because the paper contains absolute errors (smaller variance.) while here we need signed errors.
  // TODO: Parameterize this to accommodate different point source localization systems.
  meas_noise_std_vec(0) = 4.0e-3;
  meas_noise_std_vec(1) = 4.0e-3;
  meas_noise_std_vec(2) = 4.0e-3;

  this->meas_noise_cov_mat = Eigen::MatrixXd::Zero(3, 3);

  for(i0 = 0; i0 < 3; i0++)
  {
    for(i1 = 0; i1 < 3; i1++)
    {
      this->meas_noise_cov_mat(i0, i1) = meas_noise_std_vec(i0) * meas_noise_std_vec(i1);
    }
  }

  // Initialize the process noise covariance matrix. For now, we define the state as the position, and model motion
  // using the process noise. We assume a maximum speed of 3 cm/s which corresponds to three standard deviations. We
  // also assume a detection rate of 5 Hz, which results in a standard deviation of 2 mm along each dimension. We
  // assume non-zero cross correlation, which we set to 1 % of the variance values. This may change later.
  this->proc_noise_cov_mat = Eigen::MatrixXd::Zero(3, 3);

  // Also initialize the state covariance matrix.
  this->p_curr = Eigen::MatrixXd::Zero(3, 3);

  for(i0 = 0; i0 < 3; i0++)
  {
    for(i1 = 0; i1 < 3; i1++)
    {
      // NOTE: We probably need different values for the velocity and acceleration terms, using ((x_2 - x_1) / delta_t)
      // to get the standard deviation for the velocity. But for now, stick with this to test.
      if(i0 == i1)
      {
        this->proc_noise_cov_mat(i0, i1) = 4e-6;
        this->p_curr(i0, i1) = 1e-4;
      }
      else
      {
        this->proc_noise_cov_mat(i0, i1) = 4e-8;
        this->p_curr(i0, i1) = 1e-6;
      }
    }
  }

  // Initialize the matrix `A`, but do not set it, because it varies with time.
  this->proc_a_mat = Eigen::MatrixXd::Zero(3, 3);

  // Set the matrix `H`.
  this->meas_h_mat = Eigen::MatrixXd::Identity(3, 3);

  // Initialize the state vector using the input position.
  this->x_curr = x_in;
  this->x_prev = this->x_curr;

  // Set the gate threshold using Table 2.3.2-1 in [1].
  this->gate_thresh = 11.4;
}

/*
 * \brief Perform the prediction step of the LKF
 */
void pulse_est::LkfPosition::predict(void)
{
  int i0;

  for(i0 = 0; i0 < 3; i0++)
  {
    // Position components
    this->proc_a_mat(i0, i0) = 1.0;
  }

  this->computePredictedState();
}
