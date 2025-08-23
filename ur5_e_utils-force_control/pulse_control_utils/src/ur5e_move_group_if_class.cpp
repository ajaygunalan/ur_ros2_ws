/**
 *
 * \file ur5e_move_group_if_class.cpp
 *
 * \brief Class for motion planning using `MoveGroupInterface`
 *
 * \author Mardava Gubbi <mgubbi1@jhu.edu>
 *
 */
#include "pulse_control_utils/ur5e_move_group_if_class.hpp"

/*
 * \brief Construct an object of the type Ur5eMoveGroupInterfaceClass.
 */
mgi::Ur5eMoveGroupInterfaceClass::Ur5eMoveGroupInterfaceClass(
  ros::NodeHandle& nh, std::string probe_frame_name)
  : nh(nh), probe_frame_name(probe_frame_name)
{
  // Initialize the move group interface required for motion control.
  this->mgi_obj =
    new moveit::planning_interface::MoveGroupInterface(mgi::PLANNING_GROUP);
  this->mgi_obj->startStateMonitor();
  this->mgi_obj->setMaxVelocityScalingFactor(0.5);
  this->mgi_obj->setMaxAccelerationScalingFactor(0.1);
  this->jmg_obj =
    this->mgi_obj->getCurrentState()->getJointModelGroup(mgi::PLANNING_GROUP);
  this->moveit_ref_frame_name = this->mgi_obj->getPoseReferenceFrame();

  // Fetch the name of the probe frame `P`, if not provided in the constructor.
  if(this->probe_frame_name.empty())
  {
    this->probe_frame_name = this->mgi_obj->getEndEffectorLink();
  }

  // Initialize and populate vector of joint names.
  this->joint_names_vec.push_back("shoulder_pan_joint");
  this->joint_names_vec.push_back("shoulder_lift_joint");
  this->joint_names_vec.push_back("elbow_joint");
  this->joint_names_vec.push_back("wrist_1_joint");
  this->joint_names_vec.push_back("wrist_2_joint");
  this->joint_names_vec.push_back("wrist_3_joint");
}

/*
 * \brief Plan and execute motion to bring the robot to the given `joint_q_vec`.
 */
moveit::core::MoveItErrorCode
mgi::Ur5eMoveGroupInterfaceClass::moveRobotJointTarget(
  std::vector<double>& joint_q_vec)
{
  bool set_target;
  moveit::core::MoveItErrorCode output;
  moveit::planning_interface::MoveGroupInterface::Plan motion_plan;

  // Use the `PTP` motion planner for the joint vector-based motion planning,
  // with the current state as the start state, and the goal state provided to
  // the function.
  this->mgi_obj->setPlannerId("PTP");
  this->mgi_obj->setStartStateToCurrentState();
  set_target =
    this->mgi_obj->setJointValueTarget(this->joint_names_vec, joint_q_vec);

  if(set_target)
  {
    // Attempt a motion plan using the Pilz industrial motion planner.
    output = this->mgi_obj->plan(motion_plan);

    if(output == moveit::core::MoveItErrorCode::SUCCESS)
    {
      // Execute the successfully generated motion plan.
      output = this->mgi_obj->execute(motion_plan);

      if(output != moveit::core::MoveItErrorCode::SUCCESS)
      {
        ROS_ERROR_STREAM("Error moving robot: " << output);
      }
    }
    else
    {
      ROS_ERROR_STREAM("Motion planning error: " << output);
    }
  }
  else
  {
    // In the absence of a more specific error code, we use the generic FAILURE
    // to denote an inability to set the goal target.
    ROS_ERROR("Could not set joint value target.");
    output = moveit::core::MoveItErrorCode::FAILURE;
  }

  return output;
}

/*
 * \brief Plan and execute motion to bring the robot to the given `eig_goal_p`
 * relative to the current robot probe frame `P`.
 */
moveit::core::MoveItErrorCode
mgi::Ur5eMoveGroupInterfaceClass::moveRobotProbeFrame(
  Eigen::Isometry3d eig_goal_p)
{
  bool set_target;
  Eigen::Isometry3d eig_goal_b;
  Eigen::Isometry3d eig_init_b_p;
  geometry_msgs::PoseStamped geo_msg_init_b_p;
  int i0;
  moveit::core::MoveItErrorCode output;
  moveit::planning_interface::MoveGroupInterface::Plan motion_plan;

  // Use the `LIN` motion planner for probe frame-based motion planning with the
  // current state as the start state, and the goal state provided to the
  // function transformed from the probe frame `P` to the robot base frame `B`.
  this->mgi_obj->setPlannerId("LIN");
  this->mgi_obj->setStartStateToCurrentState();

  // Fetch the current transformation from the probe frame `P` to the robot base
  // frame `B`, required for the transformation mentioned above.
  geo_msg_init_b_p = this->mgi_obj->getCurrentPose();
  tf::poseMsgToEigen(geo_msg_init_b_p.pose, eig_init_b_p);
  eig_goal_b = eig_init_b_p * eig_goal_p;

  // Set the transformed goal pose in the robot base frame `B` as the target
  // for the motion plan.
  set_target = this->mgi_obj->setPoseTarget(eig_goal_b);

  if(set_target)
  {
    output = this->mgi_obj->plan(motion_plan);

    if(output == moveit::core::MoveItErrorCode::SUCCESS)
    {
      // TODO: Print out details of generated motion plan.
      ROS_DEBUG(
        "Successfully generated motion plan with %ld trajectory points in %.2f "
        "ms. "
        "Executing motion plan...",
        motion_plan.trajectory_.joint_trajectory.points.size(),
        1e3 * motion_plan.planning_time_);

      for(i0 = 0; i0 < motion_plan.trajectory_.joint_trajectory.points.size();
          i0++)
      {
        ROS_DEBUG_STREAM(
          ""
          << "Trajectory point " << i0 << " time from start: "
          << 1e3
            * motion_plan.trajectory_.joint_trajectory.points[i0]
                .time_from_start.toSec());
      }

      output = this->mgi_obj->execute(motion_plan);

      if(output != moveit::core::MoveItErrorCode::SUCCESS)
      {
        ROS_ERROR_STREAM("Error moving robot: " << output);
      }
    }
    else
    {
      ROS_ERROR_STREAM("Motion planning error: " << output);
    }
  }
  else
  {
    // In the absence of a more specific error code, we use the generic FAILURE
    // to denote an inability to set the goal target.
    ROS_ERROR("Could not set joint value target.");
    output = moveit::core::MoveItErrorCode::FAILURE;
  }

  return output;
}
