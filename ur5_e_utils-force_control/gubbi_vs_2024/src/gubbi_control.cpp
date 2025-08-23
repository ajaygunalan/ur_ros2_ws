#include "gubbi_vs_2024/gubbi_class.hpp"

void GubbiVisualServoingClass::controlTimerCallback(const ros::TimerEvent& e)
{
  Eigen::Isometry3d probe_cmd_pose_probe_eig;
  Eigen::VectorXd v_cmd = Eigen::MatrixXd::Zero(6, 1);

  // Current transform from probe frame to robot base frame
  geometry_msgs::TransformStamped base_pose_probe_msg;

  // Current transform from robot base frame to probe frame
  geometry_msgs::TransformStamped probe_pose_base_msg;

  try
  {
    ROS_DEBUG(
      "Fetching transforms between frames '%s' and '%s'...", this->base_frame_name.c_str(),
      this->probe_frame_name.c_str());

    // Update the transform from the base frame to the probe frame.
    base_pose_probe_msg = this->tf_buffer.lookupTransform(this->probe_frame_name, this->base_frame_name, ros::Time(0));
    this->base_pose_probe_eig = tf2::transformToEigen(base_pose_probe_msg);

    // Update the transform from the probe frame to the base frame.
    probe_pose_base_msg = this->tf_buffer.lookupTransform(this->base_frame_name, this->probe_frame_name, ros::Time(0));
    this->probe_pose_base_eig = tf2::transformToEigen(probe_pose_base_msg);

    ROS_DEBUG(
      "Performing control for state %d with current contact force = %.2f N...", this->fsm_state, this->force_f_z_curr);

    // Determine whether or not to perform force control
    switch(this->fsm_state)
    {
      case VS_INITIALIZE:
        break;
      case VS_NO_CONTACT:
        ROS_DEBUG("No contact - moving probe downward...");
        v_cmd(2) = 0.01;
        break;
      default:
        if(
          (this->force_f_z_curr > this->force_f_z_goal + this->force_f_z_thresh)
          || (this->force_f_z_curr < this->force_f_z_goal - this->force_f_z_thresh))
        {
          v_cmd(2) = 0.005 * (this->force_f_z_curr - this->force_f_z_goal);

          if(v_cmd(2) < -0.02)
          {
            v_cmd(2) = -0.02;
          }
          else if(v_cmd(2) > 0.02)
          {
            v_cmd(2) = 0.02;
          }
          else
          {
            // No operation
          }
        }
        else
        {
          // No operation
        }

        ROS_DEBUG("Force control commanded velocity v_z = %.2f mm/s", 1.0e3 * v_cmd(2));
    }

    // Determine whether or not to perform tracking.
    switch(this->fsm_state)
    {
      case VS_TRACK_X:
        // TODO: Track the selected LKF track.
        probe_cmd_pose_probe_eig = base_pose_probe_eig * this->probe_cmd_pose_base_eig;
        v_cmd(0) = 0.8 * probe_cmd_pose_probe_eig(0, 3);
        v_cmd(1) = 0.8 * probe_cmd_pose_probe_eig(1, 3);
        ROS_INFO("Tracking velocity v_{x,y} = [%.2f, %.2f] mm/s", 1.0e3 * v_cmd(0), 1.0e3 * v_cmd(1));

        break;
      case VS_TRACK_Y:
        if(this->track_y_wrist_3_goal > 0)
        {
          v_cmd(5) = 0.15;
        }
        else
        {
          v_cmd(5) = -0.15;
        }

        break;
      case VS_SEARCH:
        if(this->search_wrist_3_goal > 0)
        {
          v_cmd(5) = 0.15;
        }
        else
        {
          v_cmd(5) = -0.15;
        }

        break;
      default:
        // Do not move the probe in the lateral or elevation dimensions.
        break;
    }
  }
  catch(tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
  }

  if(v_cmd.norm() > 1.0e-6)
  {
    if(v_cmd.block<3, 1>(0, 0).norm() > 15.0e-3)
    {
      v_cmd.block<3, 1>(0, 0) = v_cmd.block<3, 1>(0, 0) * 15.0e-3 / (v_cmd.block<3, 1>(0, 0).norm());
    }
    else
    {
      // No operation
    }

    ROS_DEBUG(
      "Setting body cartesian velocity to [%.2f, %.2f, %.2f] mm/s...", 1.0e3 * v_cmd(0), 1.0e3 * v_cmd(1),
      1.0e3 * v_cmd(2));
    this->setBodyCartesianVelocity(v_cmd);
    this->sendGoalAndWait();
  }
  else
  {
    ROS_DEBUG("Commanded velocity is zero, canceling all goals...");
    this->cancelAllGoals();
  }
}
