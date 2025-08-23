#include "pulse_control_utils/ur5e_base_class.hpp"

int main(int argc, char** argv)
{
  std::string ee_name = "tool0";
  double delta_t;
  double joint_omega_max;
  int i0;

  ros::init(argc, argv, "joint_vel_test_node");
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

  ROS_INFO_STREAM("Setting joint velocity...");

  std::vector<double> q_dot;
  q_dot.push_back(0.1);
  q_dot.push_back(0.2);
  q_dot.push_back(0.0);
  q_dot.push_back(0.1);
  q_dot.push_back(0.2);
  q_dot.push_back(0.1);

  for(i0 = 0; i0 < 3; ++i0)
  {
    u.setJointVelocity(q_dot);

    ROS_INFO_STREAM("Sending goal...");
    u.sendGoalAndWait();
  }

  ROS_INFO_STREAM("Joint trajectory executed.");

  ros::shutdown();

  return 0;
}
