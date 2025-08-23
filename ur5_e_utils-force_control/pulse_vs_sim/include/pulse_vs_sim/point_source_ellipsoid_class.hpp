/**
 *
 * \file point_source_ellipsoid_class.hpp
 *
 * \author Mardava Gubbi <mgubbi1@jhu.edu>
 *
 */
#ifndef PULSE_POINT_SOURCE_ELLIPSOID_CLASS_HPP
#define PULSE_POINT_SOURCE_ELLIPSOID_CLASS_HPP

#include "pulse_vs_sim/point_source_base_class.hpp"

namespace vs_sim
{
/**
 *
 * \brief Move a point source in independent sinusoidal patterns in x-, y-, and
 * z-dimensions
 *
 */
class PointSourceEllipsoid : protected PointSourceBase
{
protected:
  /**
   * \brief Flag indicating whether or not to add process noise to point source control system
   */
  bool add_process_noise;

  /**
   * \brief Center of locus of commanded point source position [m]
   */
  Eigen::Vector3d pt_src_pos_center;

  /**
   * \brief Semi-axes of ellipsoidal locus of commanded point source position [m]
   */
  Eigen::Vector3d pt_src_pos_semi_axes;

  /**
   * \brief Temporal frequency of locus of commanded point source position [Hz]
   */
  Eigen::Vector3d pt_src_pos_freq;

  /**
   * \brief Random number generation engine
   */
  std::default_random_engine rand_num_gen;

  /**
   * \brief Gaussian distribution-based random number generator modeling state
   * transition uncertainty
   */
  std::normal_distribution<double>* process_noise;

public:
  /**
   *
   * \brief Construct an object of the type `PointSourceEllipsoid`
   *
   * \arg[in] nh The ROS node handler
   *
   */
  PointSourceEllipsoid(ros::NodeHandle& nh);

  /**
   *
   * \brief Callback function to publish instantaneous transform from point source frame to base frame.
   */
  void ctrlTimerCallback(const ros::TimerEvent& e) override;
};
}  // namespace vs_sim

#endif /* PULSE_POINT_SOURCE_ELLIPSOID_CLASS_HPP */
