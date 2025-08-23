#include "pulse_control_utils/ur5e_base_class.hpp"

Ur5eBaseClass::Ur5eBaseClass(
  ros::NodeHandle& nh, std::string action_server_name, std::string ee_name, double delta_t, double joint_omega_max,
  double joint_alpha_max, std::string robot_ns)
  : actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(action_server_name, true)
  , nh(nh)
  , ee_name(ee_name)
  , delta_t(delta_t)
  , joint_omega_max(joint_omega_max)
  , joint_alpha_max(joint_alpha_max)
{
  ROS_INFO("Node namespace: %s", this->nh.getNamespace().c_str());
  bool server_found = false;
  int i0;

  this->ur5e_a2 = 0.425;
  this->ur5e_a3 = 0.3922;
  this->ur5e_d4 = 0.1333;
  this->ur5e_d5 = 0.0997;
  this->ur5e_d6 = 0.0996;

  this->joint_traj_msg_id = 0;

  this->robot_model_loader.reset(new robot_model_loader::RobotModelLoader("robot_description"));
  this->robot_model = this->robot_model_loader->getModel();

  this->planning_scene_monitor.reset(new planning_scene_monitor::PlanningSceneMonitor(this->robot_model_loader));
  this->planning_scene_monitor->startSceneMonitor();
  this->planning_scene_monitor->startWorldGeometryMonitor();
  this->planning_scene_monitor->startStateMonitor();

  this->joint_model_group = this->robot_model->getJointModelGroup("manipulator");
  this->joint_names = this->joint_model_group->getVariableNames();

  this->follow_joint_traj_goal_msg.goal_time_tolerance = ros::Duration(0.1);

  this->follow_joint_traj_goal_msg.trajectory.header.frame_id = ee_name;
  this->follow_joint_traj_goal_msg.trajectory.joint_names.insert(
    this->follow_joint_traj_goal_msg.trajectory.joint_names.begin(), this->joint_names.begin(), this->joint_names.end());

  this->cart_vel_pub = this->nh.advertise<geometry_msgs::Vector3Stamped>("ee_v", 10);

  this->body_jacobian = Eigen::MatrixXd(6, 6);
  this->joint_alpha = Eigen::VectorXd(6);
  this->joint_omega_curr = Eigen::VectorXd(6);
  this->joint_omega_prev = Eigen::VectorXd(6);
  this->joint_q_curr = Eigen::VectorXd(6);
  this->joint_q_prev = Eigen::VectorXd(6);

  for(i0 = 0; i0 < 6; ++i0)
  {
    this->joint_alpha(i0) = 0.0;
    this->joint_omega_curr(i0) = 0.0;
    this->joint_omega_prev(i0) = 0.0;
    this->joint_q_curr(i0) = 0.0;
    this->joint_q_prev(i0) = 0.0;
  }

  this->joint_t_prev = ros::Time::now();
  this->joint_t_curr = ros::Time::now();

  // Wait for the action server.
  ROS_INFO_STREAM(
    ""
    << "Waiting for response from action server on topic '" << action_server_name << "'...");

  while(1)
  {
    server_found = this->waitForServer(ros::Duration(1.0));

    if(!server_found)
    {
      ROS_WARN_STREAM(
        "No response received. Continuing to wait for response from action "
        "server on topic '"
        << action_server_name << "'...");

      // Ideally this should be enforced with a timer, but this is a
      // soft-realtime requirement.
      ros::Duration(1.0).sleep();
    }
    else
    {
      ROS_DEBUG_STREAM("Response received from action server.");

      break;
    }
  }

  // Wait for robot state to be updated inside class.
  ROS_DEBUG_STREAM("Pausing prior to subscriber initialization...");
  ros::Duration(0.2).sleep();

  // Initialize subscribers at the end of the function to ensure that there are
  // no clashes.
  ROS_DEBUG_STREAM("Initializing subscribers...");
  this->joint_states_sub = this->nh.subscribe("/joint_states", 10, &Ur5eBaseClass::jointStatesCallback, this);
  this->netft_cancel_sub = this->nh.subscribe("/netft/cancel", 10, &Ur5eBaseClass::netftCancelCallback, this);
  ROS_INFO("Node namespace: %s", this->nh.getNamespace().c_str());
}

bool Ur5eBaseClass::getSpatialJacobian(Eigen::MatrixXd& m)
{
  Eigen::Vector3d ref_point(0.0, 0.0, 0.0);

  // TODO: Figure out if PSM is being used correctly to initialize robot state.
  this->robot_state.reset(new moveit::core::RobotState(
    planning_scene_monitor::LockedPlanningSceneRO(this->planning_scene_monitor)->getCurrentState()));
  this->robot_state->update();

  ROS_DEBUG_STREAM(
    "Fetching spatial Jacobian for link '" << this->joint_model_group->getLinkModelNames().back().c_str() << "'...");

  return this->robot_state->getJacobian(
    this->joint_model_group, this->robot_state->getLinkModel(this->joint_model_group->getLinkModelNames().back()),
    ref_point, m);
}

bool Ur5eBaseClass::getSpatialJacobian(Eigen::MatrixXd& m, std::string link_name)
{
  Eigen::Vector3d ref_point(0.0, 0.0, 0.0);

  // TODO: Figure out if PSM is being used correctly to initialize robot state.
  this->robot_state.reset(new moveit::core::RobotState(
    planning_scene_monitor::LockedPlanningSceneRO(this->planning_scene_monitor)->getCurrentState()));
  this->robot_state->update();

  ROS_DEBUG_STREAM("Fetching spatial Jacobian for link '" << link_name << "'...");

  return this->robot_state->getJacobian(
    this->joint_model_group, this->robot_state->getLinkModel(link_name), ref_point, m);
}

const Eigen::Isometry3d& Ur5eBaseClass::getGlobalLinkTransform(void)
{
  // TODO: Figure out if planning scene monitor is being used correctly to
  // initialize robot state.
  this->robot_state.reset(new moveit::core::RobotState(
    planning_scene_monitor::LockedPlanningSceneRO(this->planning_scene_monitor)->getCurrentState()));
  this->robot_state->update();

  return this->robot_state->getGlobalLinkTransform(this->ee_name);
}

Eigen::MatrixXd Ur5eBaseClass::getAdjointInverse(void)
{
  Eigen::MatrixXd adj_inv(6, 6);
  Eigen::MatrixXd fwd_kin_r;
  Eigen::MatrixXd p_hat_r;
  const Eigen::Isometry3d& fwd_kin = this->getGlobalLinkTransform();

  fwd_kin_r = fwd_kin.rotation().transpose();
  p_hat_r = -fwd_kin_r * this->skewSymmetric(fwd_kin.translation());
  adj_inv << fwd_kin_r, p_hat_r, Eigen::MatrixXd::Zero(3, 3), fwd_kin_r;

  return adj_inv;
}

Eigen::MatrixXd Ur5eBaseClass::skewSymmetric(Eigen::VectorXd v)
{
  Eigen::MatrixXd m(3, 3);
  m << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;

  return m;
}

const std::string Ur5eBaseClass::getRobotModelFrame(void)
{
  return this->robot_model->getModelFrame();
}

const std::vector<std::string>& Ur5eBaseClass::getJointModelGroupVariableNames(void)
{
  return this->joint_model_group->getVariableNames();
}

double Ur5eBaseClass::getPlanningSceneMonitorStateUpdateFrequency(void)
{
  return this->planning_scene_monitor->getStateUpdateFrequency();
}

void Ur5eBaseClass::copyJointGroupPositions(std::vector<double>& v)
{
  // TODO: Figure out if PSM is being used correctly to initialize robot state.
  this->robot_state.reset(new moveit::core::RobotState(
    planning_scene_monitor::LockedPlanningSceneRO(this->planning_scene_monitor)->getCurrentState()));
  this->robot_state->update();
  this->robot_state->copyJointGroupPositions(this->joint_model_group, v);
}

/*
 * \brief Callback function for joint states topic from robot.
 */
void Ur5eBaseClass::jointStatesCallback(const sensor_msgs::JointState& msg)
{
  int i0, i1;

  // Update previous time stamps, joint angles, and joint angular velocities.
  this->joint_t_prev = this->joint_t_curr;
  this->joint_t_curr = msg.header.stamp;

  this->joint_q_prev = this->joint_q_curr;
  this->joint_omega_prev = this->joint_omega_curr;

  for(i0 = 0; i0 < msg.name.size(); ++i0)
  {
    for(i1 = 0; i1 < this->joint_names.size(); ++i1)
    {
      // Match joint names in the member vector and the message content to
      // ensure that the values are correctly assigned to each joint.
      if(msg.name[i0] == this->joint_names[i1])
      {
        this->joint_q_curr(i1) = msg.position[i0];
        this->joint_omega_curr(i1) = msg.velocity[i0];

        break;
      }
      else
      {
        // No operation
      }
    }
  }

  // The message does not contain joint angular accelerations, so compute these
  // using the time stamps.
  this->joint_alpha =
    (this->joint_omega_curr - this->joint_omega_prev) / (this->joint_t_curr.toSec() - this->joint_t_prev.toSec());

  // Use the joint angular velocities to compute the end effector velocity and
  // acceleration.
}

/*
 * \brief Callback function for cancel command from force-torque sensor
 */
void Ur5eBaseClass::netftCancelCallback(const netft_utils::Cancel& msg)
{
  if(msg.toCancel)
  {
    // Call the base action client class function to cancel all goals
    // transmitted to the robot.
    ROS_WARN_STREAM("Received new motion cancel message from NET-FT sensor...");
    this->cancelAllGoals();
  }
  else
  {
    // No operation
  }
}

/*
 * \brief Send the generated joint trajectory goal to the action server.
 */
void Ur5eBaseClass::sendGoal(void)
{
  ROS_DEBUG("Sending goal to client...");
  SimpleActionClient::sendGoal(this->follow_joint_traj_goal_msg);
  this->follow_joint_traj_goal_msg.trajectory.points.clear();
}

/*
 * \brief Send the generated joint trajectory goal to the action server and
 * wait for the response.
 */
void Ur5eBaseClass::sendGoalAndWait(void)
{
  ROS_DEBUG("Sending goal to client and waiting...");
  SimpleActionClient::sendGoalAndWait(this->follow_joint_traj_goal_msg);
  this->follow_joint_traj_goal_msg.trajectory.points.clear();
}

void Ur5eBaseClass::setBodyCartesianVelocity(Eigen::VectorXd v)
{
  ros::Time t_start = ros::Time::now();
  const Eigen::Isometry3d& fwd_kin = this->getGlobalLinkTransform();
  ros::Time t_end = ros::Time::now();
  ros::Duration t_duration = t_end - t_start;

  Eigen::Matrix3d rot_mat;
  Eigen::Matrix3d p_hat;
  Eigen::Matrix4d fwd_kin_mat;
  Eigen::MatrixXd adj_t(6, 6);
  Eigen::Vector3d p_vec;
  Eigen::VectorXd v_s(6);
  int i0;

  ROS_DEBUG("Transform fetch: %.3f ms", 1.0e3 * (t_duration.toSec()));

  fwd_kin_mat = fwd_kin.matrix();
  rot_mat = fwd_kin_mat.block(0, 0, 3, 3);
  p_vec = fwd_kin_mat.block(0, 3, 3, 1);
  p_hat = hatOperator(p_vec);

  adj_t.setZero();
  adj_t.block(0, 0, 3, 3) = rot_mat;
  // adj_t.block(0, 3, 3, 3) = p_hat * rot_mat;
  adj_t.block(3, 3, 3, 3) = rot_mat;
  v_s = adj_t * v;
  this->setSpatialCartesianVelocity(v_s);
}

void Ur5eBaseClass::setJointPosition(std::vector<double> q)
{
  std::vector<double> joint_pos;
  this->copyJointGroupPositions(joint_pos);
  Eigen::VectorXd q_curr = Eigen::VectorXd::Map(joint_pos.data(), joint_pos.size());
  Eigen::VectorXd q_new = Eigen::VectorXd::Map(q.data(), q.size());
  Eigen::VectorXd joint_times = (q_new - q_curr) / this->joint_omega_max;
  double joint_time = joint_times.cwiseAbs().maxCoeff();

  trajectory_msgs::JointTrajectoryPoint jtp_msg;
  jtp_msg.time_from_start = ros::Duration(joint_time);

  for(int i0 = 0; i0 < q.size(); ++i0)
  {
    jtp_msg.positions.push_back(q[i0]);
  }

  this->follow_joint_traj_goal_msg.trajectory.points.push_back(jtp_msg);
}

void Ur5eBaseClass::setJointVelocity(std::vector<double> q_dot)
{
  std::vector<double> joint_pos;
  this->copyJointGroupPositions(joint_pos);
  Eigen::VectorXd q_curr = Eigen::VectorXd::Map(joint_pos.data(), joint_pos.size());
  Eigen::VectorXd joint_vel = Eigen::VectorXd::Map(q_dot.data(), q_dot.size());
  Eigen::VectorXd q_next = q_curr + (joint_vel * this->delta_t);
  trajectory_msgs::JointTrajectoryPoint jtp_msg;

  jtp_msg.time_from_start = ros::Duration(this->delta_t);

  for(int i0 = 0; i0 < q_dot.size(); ++i0)
  {
    jtp_msg.positions.push_back(q_next[i0]);
  }

  this->follow_joint_traj_goal_msg.trajectory.points.push_back(jtp_msg);
}

void Ur5eBaseClass::setSpatialCartesianVelocity(Eigen::VectorXd v)
{
  double j_det;
  Eigen::MatrixXd jacobian;
  Eigen::MatrixXd j_inv;
  Eigen::MatrixXd q_dot;
  geometry_msgs::Vector3Stamped msg;

  ros::Time t_start = ros::Time::now();
  this->getSpatialJacobian(jacobian);
  ros::Time t_end = ros::Time::now();
  ros::Duration t_duration = t_end - t_start;
  j_det = abs(jacobian.determinant());

  ROS_DEBUG("Jacobian fetch: %.3f ms", 1.0e3 * (t_duration.toSec()));

  if(j_det > 0.05)
  {
    j_inv = jacobian.inverse();
    q_dot = j_inv * v;
    ROS_DEBUG(
      "\n\tq_dot: [%.3e, %.3e, %.3e, %.3e, %.3e, %.3e]", q_dot(0), q_dot(1), q_dot(2), q_dot(3), q_dot(4), q_dot(5));

    std::vector<double> joint_vel(q_dot.data(), q_dot.data() + q_dot.cols() * q_dot.rows());
    this->setJointVelocity(joint_vel);

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "spatial";
    msg.vector.x = v(0);
    msg.vector.y = v(1);
    msg.vector.z = v(2);
    this->cart_vel_pub.publish(msg);
  }
  else
  {
    ROS_ERROR_STREAM("Robot is close to singular position [|J| = " << j_det << "]");
    throw std::runtime_error("Singular position");
  }
}
