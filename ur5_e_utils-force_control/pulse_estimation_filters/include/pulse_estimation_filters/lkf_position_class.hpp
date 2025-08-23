/**
 *
 * \file lkf_position_class.hpp
 *
 * \brief Linear Kalman Filter
 *
 * \details This class assumes that all states and measurements are provided in the same frame.
 *
 * \author Mardava Gubbi <mgubbi1@jhu.edu>
 *
 * \references
 * 1. Bar-Shalom, Yaakov, and Xiao-Rong Li. Multitarget-multisensor tracking: principles and techniques. Vol. 19.
 * Storrs, CT: YBS publishing, 1995.
 * 2. Bar-Shalom, Yaakov, X. Rong Li, and Thiagalingam Kirubarajan. Estimation with applications to tracking and
 * navigation: theory algorithms and software. John Wiley & Sons, 2004.
 * 3. Gubbi and Bell, IEEE T-UFFC 2024
 *
 */
#ifndef PULSE_LKF_POSITION_CLASS_HPP
#define PULSE_LKF_POSITION_CLASS_HPP

#include <string>

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <tf2_eigen/tf2_eigen.h>

#include "pulse_estimation_filters/chi_squared.hpp"
#include "pulse_estimation_filters/linear_kalman_filter_class.hpp"

namespace pulse_est
{
class LkfPosition : public LinearKalmanFilter
{
public:
  /**
   *
   * \brief Construct an object of the type `LinearKalmanFilter`
   *
   * \arg[in] x_in Input position to be used to initialize LKF
   *
   */
  LkfPosition(Eigen::Vector3d x_in, double smooth_coeff);

  /**
   *
   * \brief Perform the prediction step of the LKF
   *
   */
  void predict(void) override;
};
}  // namespace pulse_est

#endif /* PULSE_LKF_POSITION_CLASS_HPP */
