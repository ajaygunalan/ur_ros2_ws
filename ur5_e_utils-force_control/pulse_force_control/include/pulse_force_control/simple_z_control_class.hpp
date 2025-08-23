/**
 *
 * \file simple_z_control.hpp
 *
 * \brief Move the robot up and down to maintain a desired force in the translational z-dimension.
 *
 * \author Mardava Gubbi <mgubbi1@jhu.edu>
 *
 */
#ifndef PULSE_SIMPLE_Z_CONTROL_CLASS_HPP
#define PULSE_SIMPLE_Z_CONTROL_CLASS_HPP

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>

#include "pulse_control_utils/ur5e_base_class.hpp"
#include "pulse_common_utils/eigen_string_utils.hpp"

class SimpleZControlClass : public Ur5eBaseClass
{
protected:
  double f_z_goal;
  double f_z_curr;
  double f_z_thresh;

  ros::Subscriber force_sensor_sub;
  ros::Timer force_control_timer;

public:
  SimpleZControlClass(
    ros::NodeHandle& nh, std::string action_server_name, std::string ee_name = "p42v_link1", double delta_t = 0.1,
    double joint_omega_max = (M_PI / 10.0));

  void forceControlTimerCallback(const ros::TimerEvent& e);

  void forceSensorSubscriberCallback(geometry_msgs::WrenchStamped msg);
};

#endif /* PULSE_SIMPLE_Z_CONTROL_CLASS_HPP */
