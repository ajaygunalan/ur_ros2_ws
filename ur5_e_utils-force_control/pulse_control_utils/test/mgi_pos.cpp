#include "pulse_control_utils/ur5e_move_group_if_class.hpp"

class Ur5eMoveGroupInterfaceTestClass : public mgi::Ur5eMoveGroupInterfaceClass
{
public:
  /**
   * \brief Constructor for class Ur5eMoveGroupInterfaceTestClass
   */
  Ur5eMoveGroupInterfaceTestClass(
    ros::NodeHandle& nh, std::string probe_frame_name = "p42v_link1");

  /**
   * \brief Move the robot to the vertical home position
   */
  void moveToHomePosition(void);

  /**
   * \brief Move the robot to the joint position specified by the input joint
   * names and vectors.
   *
   * \arg[in] joint_names_vec Vector of joint names
   * \arg[in] joint_q_vec Vector of joint angles corresponding to joint names
   */
  void moveToJointPosition(
    std::vector<std::string> joint_names_vec, std::vector<double> joint_q_vec);

  /**
   * \brief Move the robot to the starting position for the given motion tests
   */
  void moveToStartPosition(void);

  /**
   * \brief Rotate the robot about the x-dimension of the probe frame `P`
   *
   * \arg[in] angle The signed angle by which the robot must be rotated [rad]
   */
  void rotateProbeXDimension(double angle);

  /**
   * \brief Rotate the robot about the y-dimension of the probe frame `P`
   *
   * \arg[in] angle The signed angle by which the robot must be rotated [rad]
   */
  void rotateProbeYDimension(double angle);

  /**
   * \brief Rotate the robot about the z-dimension of the probe frame `P`
   *
   * \arg[in] angle The signed angle by which the robot must be rotated [rad]
   */
  void rotateProbeZDimension(double angle);

  /**
   * \brief Translate the robot along the x-dimension of the probe frame `P`
   *
   * \arg[in] displacement The signed displacement by which the robot must be
   * translated [m]
   */
  void translateProbeXDimension(double displacement);

  /**
   * \brief Translate the robot along the y-dimension of the probe frame `P`
   *
   * \arg[in] displacement The signed displacement by which the robot must be
   * translated [m]
   */
  void translateProbeYDimension(double displacement);

  /**
   * \brief Translate the robot along the z-dimension of the probe frame `P`
   *
   * \arg[in] displacement The signed displacement by which the robot must be
   * translated [m]
   */
  void translateProbeZDimension(double displacement);
};

/*
 * \brief Constructor for class Ur5eMoveGroupInterfaceTestClass
 */
Ur5eMoveGroupInterfaceTestClass::Ur5eMoveGroupInterfaceTestClass(
  ros::NodeHandle& nh, std::string probe_frame_name)
  : mgi::Ur5eMoveGroupInterfaceClass(nh, probe_frame_name)
{
  this->mgi_obj->setMaxAccelerationScalingFactor(0.1);
}

/*
 * \brief Move the robot to the vertical home position
 */
void Ur5eMoveGroupInterfaceTestClass::moveToHomePosition(void)
{
  std::vector<double> joint_q_vec;
  std::vector<std::string> joint_names_vec;

  joint_names_vec.push_back("shoulder_pan_joint");
  joint_names_vec.push_back("shoulder_lift_joint");
  joint_names_vec.push_back("elbow_joint");
  joint_names_vec.push_back("wrist_1_joint");
  joint_names_vec.push_back("wrist_2_joint");
  joint_names_vec.push_back("wrist_3_joint");

  joint_q_vec.push_back(0.0);
  joint_q_vec.push_back(-M_PI / 2.0);
  joint_q_vec.push_back(0.0);
  joint_q_vec.push_back(-M_PI / 2.0);
  joint_q_vec.push_back(0.0);
  joint_q_vec.push_back(0.0);

  this->moveToJointPosition(joint_names_vec, joint_q_vec);
}

/*
 * \brief Move the robot to the joint position specified by the input joint
 * names and vectors.
 */
void Ur5eMoveGroupInterfaceTestClass::moveToJointPosition(
  std::vector<std::string> joint_names_vec, std::vector<double> joint_q_vec)
{
  bool set_target;
  moveit::core::MoveItErrorCode plan_err;
  moveit::core::MoveItErrorCode move_err;
  moveit::planning_interface::MoveGroupInterface::Plan motion_plan;

  ROS_DEBUG("Setting planner ID for joint angular motion...");
  this->mgi_obj->setPlannerId("PTP");

  ROS_DEBUG("Setting motion planning start state to current state...");
  this->mgi_obj->setStartStateToCurrentState();

  ROS_DEBUG("Setting desired joint value targets...");
  set_target = this->mgi_obj->setJointValueTarget(joint_names_vec, joint_q_vec);

  if(set_target)
  {
    ROS_DEBUG("Target successfully set, planning robot motion...");

    plan_err = this->mgi_obj->plan(motion_plan);

    if(plan_err == moveit::core::MoveItErrorCode::SUCCESS)
    {
      ROS_DEBUG("Planning successful, moving robot...");
      move_err = this->mgi_obj->execute(motion_plan);

      if(move_err == moveit::core::MoveItErrorCode::SUCCESS)
      {
        ROS_DEBUG("Motion successful.");
      }
      else
      {
        ROS_ERROR_STREAM("Error moving robot: " << move_err);
      }
    }
    else
    {
      ROS_ERROR_STREAM("Motion planning error: " << plan_err);
    }
  }
  else
  {
    ROS_ERROR("Could not set joint value target.");
  }
}

/*
 * \brief Move the robot to the starting position for the given motion tests
 */
void Ur5eMoveGroupInterfaceTestClass::moveToStartPosition(void)
{
  std::vector<double> joint_q_vec;
  std::vector<std::string> joint_names_vec;

  joint_names_vec.push_back("shoulder_pan_joint");
  joint_names_vec.push_back("shoulder_lift_joint");
  joint_names_vec.push_back("elbow_joint");
  joint_names_vec.push_back("wrist_1_joint");
  joint_names_vec.push_back("wrist_2_joint");
  joint_names_vec.push_back("wrist_3_joint");

  joint_q_vec.push_back(M_PI / 2.0);
  joint_q_vec.push_back(-M_PI / 2.0);
  joint_q_vec.push_back(M_PI / 2.0);
  joint_q_vec.push_back(-M_PI / 2.0);
  joint_q_vec.push_back(-M_PI / 2.0);
  joint_q_vec.push_back(0.0);

  ROS_DEBUG("Performing PTP-based robot movement towards starting position...");
  this->moveToJointPosition(joint_names_vec, joint_q_vec);

  ROS_DEBUG("Performing LIN-based robot movement towards starting position...");
  this->translateProbeZDimension(0.1);
}

/*
 * \brief Rotate the robot about the x-dimension of the probe frame `P`
 */
void Ur5eMoveGroupInterfaceTestClass::rotateProbeXDimension(double angle)
{
  Eigen::Isometry3d eig_motion;
  Eigen::Quaterniond q_motion;

  q_motion.x() = sin(angle / 2.0);
  q_motion.y() = 0.0;
  q_motion.z() = 0.0;
  q_motion.w() = cos(angle / 2.0);

  eig_motion.setIdentity();
  eig_motion.rotate(q_motion);

  this->moveRobotProbeFrame(eig_motion);
}

/*
 * \brief Rotate the robot about the y-dimension of the probe frame `P`
 */
void Ur5eMoveGroupInterfaceTestClass::rotateProbeYDimension(double angle)
{
  Eigen::Isometry3d eig_motion;
  Eigen::Quaterniond q_motion;

  q_motion.x() = 0.0;
  q_motion.y() = sin(angle / 2.0);
  q_motion.z() = 0.0;
  q_motion.w() = cos(angle / 2.0);

  eig_motion.setIdentity();
  eig_motion.rotate(q_motion);

  this->moveRobotProbeFrame(eig_motion);
}

/*
 * \brief Rotate the robot about the z-dimension of the probe frame `P`
 */
void Ur5eMoveGroupInterfaceTestClass::rotateProbeZDimension(double angle)
{
  Eigen::Isometry3d eig_motion;
  Eigen::Quaterniond q_motion;

  q_motion.x() = 0.0;
  q_motion.y() = 0.0;
  q_motion.z() = sin(angle / 2.0);
  q_motion.w() = cos(angle / 2.0);

  eig_motion.setIdentity();
  eig_motion.rotate(q_motion);

  this->moveRobotProbeFrame(eig_motion);
}

/*
 * \brief Translate the robot along the x-dimension of the probe frame `P`
 */
void Ur5eMoveGroupInterfaceTestClass::translateProbeXDimension(
  double displacement)
{
  Eigen::Isometry3d eig_motion;
  Eigen::Vector3d t_motion;

  t_motion.setZero();
  t_motion(0) = displacement;

  eig_motion.setIdentity();
  eig_motion.translate(t_motion);

  this->moveRobotProbeFrame(eig_motion);
}

/*
 * \brief Translate the robot along the y-dimension of the probe frame `P`
 */
void Ur5eMoveGroupInterfaceTestClass::translateProbeYDimension(
  double displacement)
{
  Eigen::Isometry3d eig_motion;
  Eigen::Vector3d t_motion;

  t_motion.setZero();
  t_motion(1) = displacement;

  eig_motion.setIdentity();
  eig_motion.translate(t_motion);

  this->moveRobotProbeFrame(eig_motion);
}

/*
 * \brief Translate the robot along the z-dimension of the probe frame `P`
 */
void Ur5eMoveGroupInterfaceTestClass::translateProbeZDimension(
  double displacement)
{
  Eigen::Isometry3d eig_motion;
  Eigen::Vector3d t_motion;

  t_motion.setZero();
  t_motion(2) = displacement;

  eig_motion.setIdentity();
  eig_motion.translate(t_motion);

  this->moveRobotProbeFrame(eig_motion);
}

int main(int argc, char** argv)
{
  // Initialize the node and declare the node handler.
  ros::init(argc, argv, "mgi_pos_test_node");
  ros::NodeHandle nh("~");

  // Start an async spinner to avoid move group interface object complaining
  // about time sync with UR5e.
  // https://github.com/ros-planning/moveit/issues/1187
  // Use more than one thread to avoid MoveIt's move group interface's plan()
  // function freezing.
  // https://github.com/jacknlliu/development-issues/issues/44
  ros::AsyncSpinner spinner(0);
  spinner.start();

  // Instantiate the MoveGroupInterface-based motion planning test object.
  Ur5eMoveGroupInterfaceTestClass u(nh);

  u.moveToHomePosition();
  u.moveToStartPosition();

  ros::Duration(1.0).sleep();
  u.translateProbeXDimension(0.05);
  ros::Duration(1.0).sleep();
  u.translateProbeXDimension(-0.05);

  ros::Duration(1.0).sleep();
  u.translateProbeYDimension(0.05);
  ros::Duration(1.0).sleep();
  u.translateProbeYDimension(-0.05);

  ros::Duration(1.0).sleep();
  u.translateProbeZDimension(0.05);
  ros::Duration(1.0).sleep();
  u.translateProbeZDimension(-0.05);

  ros::Duration(1.0).sleep();
  u.rotateProbeXDimension(10.0 * M_PI / 180.0);
  ros::Duration(1.0).sleep();
  u.rotateProbeXDimension(-10.0 * M_PI / 180.0);

  ros::Duration(1.0).sleep();
  u.rotateProbeYDimension(10.0 * M_PI / 180.0);
  ros::Duration(1.0).sleep();
  u.rotateProbeYDimension(-10.0 * M_PI / 180.0);

  ros::Duration(1.0).sleep();
  u.rotateProbeZDimension(10.0 * M_PI / 180.0);
  ros::Duration(1.0).sleep();
  u.rotateProbeZDimension(-10.0 * M_PI / 180.0);

  // Wait for the node to shutdown.
  ros::waitForShutdown();
  delete(&u);

  return 0;
}
