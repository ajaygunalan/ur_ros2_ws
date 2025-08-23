/**
 * \file ur5e_move_group_visual_servoing_class.cpp
 *
 * \brief Visual servoing system with force control
 *
 * \details See Murray, Li, and Sastry
 *
 * \author Mardava Gubbi <mgubbi1@jhu.edu>
 */
#include "gubbi_icra_2021/ur5e_move_group_visual_servoing_class.hpp"

ur5e_moveit_visual_servoing::Ur5eMoveGroupVisualServoingClass::
  Ur5eMoveGroupVisualServoingClass(
    ros::NodeHandle& nh, bool force_control, std::string sensor_frame_name,
    std::string probe_frame_name)
  : mgi::Ur5eMoveGroupInterfaceClass(nh, probe_frame_name)
  , force_control(force_control)
  , sensor_frame_name(sensor_frame_name)
{
  if(!this->nh.getParam("fsm_period", this->fsm_period))
  {
    ROS_WARN("Could not find parameter 'fsm_period', setting FSM period to "
             "default value (50 ms)...");
    this->fsm_period = 0.05;
  }

  this->fsm_timer = this->nh.createTimer(
    ros::Duration(this->fsm_period),
    &Ur5eMoveGroupVisualServoingClass::fsmTimerCallback, this);
}

/*
 * \brief Execute the FSM at a fixed periodicity
 */
void ur5e_moveit_visual_servoing::Ur5eMoveGroupVisualServoingClass::
  fsmTimerCallback(const ros::TimerEvent& e)
{
  bool success;
  int i0, i1, i2;
  moveit::core::MoveItErrorCode move_exe_err;
  std::vector<double> joint_values;
  geometry_msgs::PoseStamped pose_target;
  moveit::planning_interface::MoveGroupInterface::Plan test_plan;
  std::stringstream traj_ss;
  trajectory_msgs::JointTrajectoryPoint traj_point;

  joint_values.push_back(M_PI / 2.0);
  joint_values.push_back(-M_PI / 2.0);
  joint_values.push_back(M_PI / 2.0);
  joint_values.push_back(-M_PI / 2.0);
  joint_values.push_back(-M_PI / 2.0);
  joint_values.push_back(0);

  switch(this->fsm_state)
  {
    case ur5e_moveit_visual_servoing::MOTION_TEST:
      this->mgi_obj->setPlannerId("PTP");
      ROS_INFO("Planning rotational motion...");
      this->mgi_obj->setStartStateToCurrentState();
      this->mgi_obj->setJointValueTarget(joint_values);
      success =
        (this->mgi_obj->plan(test_plan)
         == moveit::core::MoveItErrorCode::SUCCESS);

      if(success)
      {
        ROS_INFO("Successfully planned robot motion. Moving robot...");
        move_exe_err = this->mgi_obj->execute(test_plan);
        ROS_INFO_STREAM(
          "Completed execution of robot motion plan with error code "
          << move_exe_err << ".");
      }
      else
      {
        ROS_WARN("Failed to plan robot motion.");
        // TODO: Add some error throwing here.
      }

      this->mgi_obj->setPlannerId("LIN");
      // this->mgi_obj->setMaxVelocityScalingFactor(0.15);

      ROS_INFO("Planning translational motion...");
      this->mgi_obj->setStartStateToCurrentState();
      pose_target = this->mgi_obj->getCurrentPose();
      pose_target.pose.position.z -= 0.05;
      this->mgi_obj->setPoseTarget(pose_target);
      success =
        (this->mgi_obj->plan(test_plan)
         == moveit::core::MoveItErrorCode::SUCCESS);

      if(success)
      {
        ROS_DEBUG("Successfully planned robot motion as follows.");

        for(i0 = 0; i0 < test_plan.trajectory_.joint_trajectory.points.size();
            ++i0)
        {
          traj_point = test_plan.trajectory_.joint_trajectory.points[i0];
          traj_ss
            << test_plan.trajectory_.joint_trajectory.points[i0].time_from_start
            << std::endl
            << "j_p: [";

          for(i1 = 0; i1 < traj_point.positions.size(); ++i1)
          {
            traj_ss << boost::format("%.2f, ") % traj_point.positions[i1];
          }

          traj_ss << "]" << std::endl << "j_v: [";

          for(i1 = 0; i1 < traj_point.velocities.size(); ++i1)
          {
            traj_ss << boost::format("%.2f, ") % traj_point.velocities[i1];
          }

          traj_ss << "]" << std::endl << "j_a: [";

          for(i1 = 0; i1 < traj_point.accelerations.size(); ++i1)
          {
            traj_ss << boost::format("%.2f, ") % traj_point.accelerations[i1];
          }

          traj_ss << "]";
          ROS_DEBUG_STREAM(traj_ss.str());
          traj_ss = std::stringstream();
        }

        ROS_INFO("Successfully planned robot motion. Moving robot...");
        this->mgi_obj->asyncExecute(test_plan);
      }
      else
      {
        ROS_WARN("Failed to plan robot motion.");
      }

      this->fsm_state = ur5e_moveit_visual_servoing::STATIONARY;
      break;
    case ur5e_moveit_visual_servoing::STATIONARY:
      // TODO: Hold the robot stationary.
      ROS_INFO(
        "Holding robot stationary after completion of current movement...");
      break;
    case ur5e_moveit_visual_servoing::PRE_CONTACT:
      // TODO: Move along z-axis of probe to make contact with surface.
      break;
    case ur5e_moveit_visual_servoing::TRACKING:
      // TODO: Maintain force control if required and position tracking of point
      // source.
      break;
    case ur5e_moveit_visual_servoing::SEARCHING:
      // TODO: Search while maintaining force control, incrementing a counter to
      // transition to the `STATIONARY` state.
      break;
    default:
      ROS_ERROR_STREAM("Unknown FSM state " << this->fsm_state << ".");
  }
}

/*
 * \brief Compute the force error for each processed force-torque reading
 */
void ur5e_moveit_visual_servoing::Ur5eMoveGroupVisualServoingClass::
  ftProcProbeCallback(const geometry_msgs::WrenchStamped& msg)
{
  tf::wrenchMsgToEigen(msg.wrench, this->ft_reading_probe);
}

void ur5e_moveit_visual_servoing::Ur5eMoveGroupVisualServoingClass::bboxCallback(
  const vision_msgs::Detection2DArray& msg)
{
  // To obtain the point source location from bounding boxes, we require the
  // image dimensions [px], pixel dimensions [m], and confidence score threshold, if applicable.
}
