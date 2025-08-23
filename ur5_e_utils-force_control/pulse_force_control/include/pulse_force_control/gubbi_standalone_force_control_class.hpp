#ifndef GUBBI_STANDALONE_FORCE_CONTROL_CLASS_HPP
#define GUBBI_STANDALONE_FORCE_CONTROL_CLASS_HPP

#include "pulse_force_control/gubbi_force_control_class.hpp"

namespace pfc
{
class GubbiStandaloneForceControlClass : public pfc::GubbiForceControlClass
{
public:
  /**
   * \brief Constructor for GubbiStandaloneForceControlClass
   *
   * \arg[in] nh NodeHandle object
   * \arg[in] base_frame_name The name of the robot base frame `B`
   * \arg[in] probe_frame_name The name of the probe frame `P`
   * \arg[in] tool_frame_name The name of the tool frame `T`
   *
   * TODO: Come up with a better name for the tool frame `T`.
   */
  GubbiStandaloneForceControlClass(
    ros::NodeHandle& nh, std::string base_frame_name = "base_link", std::string probe_frame_name = "p42v_link1",
    std::string tool_frame_name = "p42v_link1_fc");

  /**
   * \brief Callback function for control loop timer
   *
   * \arg[in] e ROS timer event variable (typically not used in function)
   */
  void forceControlLoopTimerCallback(const ros::TimerEvent& e) override;
};
}  // namespace pfc

#endif /* GUBBI_STANDALONE_FORCE_CONTROL_CLASS_HPP */
