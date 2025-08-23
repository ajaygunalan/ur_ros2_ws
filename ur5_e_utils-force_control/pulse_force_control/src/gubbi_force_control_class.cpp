#include "pulse_force_control/gubbi_force_control_class.hpp"

pfc::GubbiForceControlClass::GubbiForceControlClass(
  ros::NodeHandle& nh, std::string base_frame_name,
  std::string probe_frame_name, std::string tool_frame_name)
  : pfc::YoshikawaForceControlClass(
    nh, base_frame_name, probe_frame_name, tool_frame_name)
{
}

/*
 * \brief Estimate the local normal of the unknown constraint surface as
 * anti-parallel to the force vector.
 */
Eigen::Quaterniond pfc::GubbiForceControlClass::estimateLocalNormal(void)
{
  ROS_DEBUG("GubbiForceControlClass::estimateLocalNormal");
  Eigen::Quaterniond q_p_t;

  // Define the z-axis of the probe frame `P` in itself.
  Eigen::Vector3d z_p_p;
  z_p_p.setZero();
  z_p_p(2) = 1.0;

  // Define the z-axis of the tool frame `T` in the probe frame `P`.
  Eigen::Vector3d z_p_t = -this->ft_curr.block(0, 0, 3, 1);

  if(z_p_t.norm() > 0.0)
  {
    z_p_t = z_p_t / z_p_t.norm();

    Eigen::Vector3d rot_axis = hatOperator(z_p_p) * z_p_t;

    if(rot_axis.norm() > 0.0)
    {
      rot_axis = rot_axis / rot_axis.norm();

      double cos_rot_angle = z_p_p.transpose() * z_p_t;
      double rot_angle = acos(cos_rot_angle);

      q_p_t.vec() = rot_axis * sin(rot_angle / 2.0);
      q_p_t.w() = cos(rot_angle / 2.0);
    }
    else
    {
      ROS_DEBUG("Probe perfectly aligned with force vector, setting rotation "
                "to zero...");
      q_p_t.x() = 0.0;
      q_p_t.y() = 0.0;
      q_p_t.z() = 0.0;
      q_p_t.w() = 1.0;
    }
  }
  else
  {
    ROS_WARN("Zero-norm vector received for tool frame z-axis, setting "
             "rotation to zero...");
    q_p_t.x() = 0.0;
    q_p_t.y() = 0.0;
    q_p_t.z() = 0.0;
    q_p_t.w() = 1.0;
  }

  return q_p_t;
}

