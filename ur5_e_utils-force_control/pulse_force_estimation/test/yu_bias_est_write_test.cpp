#include "pulse_force_estimation/yu_bias_est_base_class.hpp"

class YuBiasEstWriteClass : protected ft::YuBiasEstBaseClass
{
protected:
  void estimateGravitationalForcesRobotBaseFrame(void) override;
  void setRobotCmdJointAngles(void) override;

public:
  YuBiasEstWriteClass(
    ros::NodeHandle& nh, std::string base_frame_name = "base_link", std::string ee_frame_name = "tool0",
    std::string sensor_frame_name = "netft_link1", std::string probe_frame_name = "p42v_link1");
};

YuBiasEstWriteClass::YuBiasEstWriteClass(
  ros::NodeHandle& nh, std::string base_frame_name, std::string ee_frame_name, std::string sensor_frame_name,
  std::string probe_frame_name)
  : YuBiasEstBaseClass(nh, base_frame_name, ee_frame_name, sensor_frame_name, probe_frame_name)
{
  // We do not want the timer running.
  this->fsm_timer.stop();

  this->writeEstimationsToFile();
}

void YuBiasEstWriteClass::estimateGravitationalForcesRobotBaseFrame(void)
{
}

void YuBiasEstWriteClass::setRobotCmdJointAngles(void)
{
}

int main(int argc, char** argv)
{
  // Initialize the node and declare the node handler.
  ros::init(argc, argv, "yu_bias_est_node");
  ros::NodeHandle nh("~");

  // Start an async spinner to avoid move group interface object complaining about time sync with UR5e.
  // https://github.com/ros-planning/moveit/issues/1187
  // Use more than one thread to avoid MoveIt's move group interface's plan() function freezing.
  // https://github.com/jacknlliu/development-issues/issues/44
  ros::AsyncSpinner spinner(0);
  spinner.start();

  // Instantiate the bias compensation and gravity estimation object.
  YuBiasEstWriteClass y(nh);

  // Wait for the node to shutdown.
  ros::waitForShutdown();

  return 0;
}
