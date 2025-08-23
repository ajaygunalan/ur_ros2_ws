#include "pulse_vs_sim/sim_img_publish_class.hpp"

int main(int argc, char** argv)
{
  // Initialize the node and declare the node handler.
  ros::init(argc, argv, "sim_img_publish_node");
  ros::NodeHandle nh("~");

  // The AsyncSpinner was originally used to deal with MoveIt-based control of the UR5e robot. In this situation, it
  // probably will not have a significant effect either way.
  ros::AsyncSpinner spinner(0);
  spinner.start();

  PaimdbSimImgPublisher psip(nh);

  // Wait for the node to shutdown.
  ros::waitForShutdown();

  return 0;
}
