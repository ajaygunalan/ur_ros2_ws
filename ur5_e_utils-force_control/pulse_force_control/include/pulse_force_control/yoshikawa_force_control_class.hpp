/**
 *
 * \file yoshikawa_force_control_class.hpp
 *
 * \brief This file implements the force control method described by Yoshikawa
 * et al. [1]
 *
 * \details The variables used in the mathematical derivations of the paper are
 * too generic to be used in their exact form, but references to equations in
 * the paper are provided where applicable to simplify reading this code.
 *
 * See also `tool_frame_force_control_class.hpp`
 *
 * References:
 * 1. Yoshikawa, Tsuneo, and Akio Sudou. "Dynamic hybrid position/force control
 * of robot manipulators-on-line estimation of unknown constraint." IEEE
 * Transactions on Robotics and Automation (1993).
 *
 * \author Mardava Gubbi <mgubbi1@jhu.edu>
 *
 */
#ifndef YOSHIKAWA_FORCE_CONTROL_CLASS_HPP
#define YOSHIKAWA_FORCE_CONTROL_CLASS_HPP

#include "pulse_force_control/tool_frame_force_control_class.hpp"

/*
 * \brief Minimum value of contact counter to transition from FSM state
 * `pre-contact` to FSM state `contact`
 */
#define MIN_CONTACT_COUNTER_VAL 10

namespace pfc
{
/**
 *
 * \brief Class implementing task frame estimation and hybrid position-force
 * control system described by Yoshikawa et al. [1]
 *
 */
class YoshikawaForceControlClass : public ToolFrameForceControlClass
{
protected:
  /*
   * \brief Deadbands to accommodate small errors in force-torque readings and
   * prevent loss of target during photoacoustic-based visual servoing.
   */
  double f_xy_deadband;

  /**
   *
   * \brief Estimate the local normal of the unknown constraint surface
   * as described in Section III of Yoshikawa et al.
   *
   * \returns A quaternion representing the rotation from the tool frame `T` (in
   * which the normal to the surface forms the z-axis) to the current probe frame `P`.
   *
   */
  virtual Eigen::Quaterniond estimateLocalNormal(void);

public:
  /**
   *
   * \brief Constructor for YoshikawaForceControlClass
   *
   * \arg[in] nh NodeHandle object
   * \arg[in] base_frame_name The name of the robot base frame `B`
   * \arg[in] probe_frame_name The name of the probe frame `P`
   * \arg[in] tool_frame_name The name of the tool frame `T`
   *
   * TODO: Come up with a better name for the tool frame `T`.
   *
   */
  YoshikawaForceControlClass(
    ros::NodeHandle& nh, std::string base_frame_name = "base_link", std::string probe_frame_name = "p42v_link1",
    std::string tool_frame_name = "p42v_link1_fc");

  /**
   *
   * \brief Callback function for force control loop
   *
   * \details This function checks the force-torque error and computes the
   * desired tool frame `T` relative to the current tool frame `P`. If the
   * translational z-component of the force error is above a threshold, the
   * rotation of `T` relative to `P` is zero and the focus is on reducing this
   * single error component (i.e., making contact with the surface). Otherwise,
   * quaternion is computed and subjected to a saturation filter to avoid
   * dangerous rotations. The desired tool frame `T` is then published via a TF
   * publisher.
   *
   * \arg[in] e ROS timer event variable (typically not used in function)
   *
   */
  void forceControlLoopTimerCallback(const ros::TimerEvent& e) override;
};
}  // namespace pfc

#endif /* YOSHIKAWA_FORCE_CONTROL_CLASS_HPP */
