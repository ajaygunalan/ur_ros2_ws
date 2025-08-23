#include "pulse_control_utils/ur5e_base_class.hpp"

int main(int argc, char** argv)
{
  std::string ee_name = "tool0";
  double delta_t;
  double joint_omega_max;
  ros::init(argc, argv, "joint_pos_test_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);
  std::string action_server_name;

  spinner.start();

  ROS_INFO_STREAM("Fetching action client topic...");
  nh.getParam("/action_server_name", action_server_name);
  ROS_INFO_STREAM("Fetching time-step for motion commands...");
  nh.getParam("/delta_t", delta_t);
  nh.getParam("/joint_omega_max", joint_omega_max);

  ROS_INFO("Starting test...");

  Ur5eBaseClass u(nh, action_server_name, ee_name, delta_t, joint_omega_max);

  ROS_INFO_STREAM("Setting joint positions...");

  std::vector<double> q;
  q.push_back(0.0);
  q.push_back(-M_PI / 2.0);
  q.push_back(M_PI / 2.0);
  q.push_back(-M_PI / 2.0);
  q.push_back(0.0);
  q.push_back(0.0);

  u.setJointPosition(q);

  ROS_INFO_STREAM("Sending goal...");
  u.sendGoalAndWait();

  ROS_INFO_STREAM("Joint trajectory executed.");

  ros::shutdown();

  return 0;
}
