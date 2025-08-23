/**
 *
 * \file gubbi_force_control_class.hpp
 *
 * \brief This file contains a tool frame-based force control method which ignores friction, instead computing the
 * desired orientation of the probe as axially anti-parallel to the input force vector.
 *
 * \author Mardava Gubbi <mgubbi1@jhu.edu>
 *
 */
#ifndef GUBBI_FORCE_CONTROL_CLASS_HPP
#define GUBBI_FORCE_CONTROL_CLASS_HPP

#include "pulse_force_control/yoshikawa_force_control_class.hpp"

namespace pfc
{
class GubbiForceControlClass : public YoshikawaForceControlClass
{
protected:
  /**
   *
   * \brief Estimate the local normal of the unknown constraint surface as parallel
   * to the force vector.
   *
   * \returns A quaternion representing the rotation from the tool frame `T` (in
   * which the normal to the surface forms the z-axis) to the current probe frame `P`.
   *
   */
  Eigen::Quaterniond estimateLocalNormal(void) override;

public:
  /**
   *
   * \brief Construct an object of the type GubbiForceControlClass
   *
   * \arg[in] nh NodeHandle object
   * \arg[in] base_frame_name The name of the robot base frame `B`
   * \arg[in] probe_frame_name The name of the probe frame `P`
   * \arg[in] tool_frame_name The name of the tool frame `T`
   *
   * TODO: Come up with a better name for the tool frame `T`.
   *
   */
  GubbiForceControlClass(
    ros::NodeHandle& nh, std::string base_frame_name = "base_link", std::string probe_frame_name = "p42v_link1",
    std::string tool_frame_name = "p42v_link1_fc");
};
}  // namespace pfc

#endif /* GUBBI_FORCE_CONTROL_CLASS_HPP */
