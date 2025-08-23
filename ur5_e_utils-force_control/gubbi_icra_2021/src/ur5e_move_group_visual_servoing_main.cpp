#include "gubbi_icra_2021/ur5e_move_group_visual_servoing_class.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visual_servoing_node");
  ros::NodeHandle nh("~");

  ros::AsyncSpinner spinner(2);
  spinner.start();

  ur5e_moveit_visual_servoing::Ur5eMoveGroupVisualServoingClass u(nh);

  ros::waitForShutdown();
  return 0;
}
