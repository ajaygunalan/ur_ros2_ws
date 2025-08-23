#include "pulse_estimation_filters/particle_filter_vel_noise_class.hpp"

int main(int argc, char** argv)
{
  // Initialize the node and declare the node handler.
  ros::init(argc, argv, "particle_filter_vel_noise_node");
  ros::NodeHandle nh("~");

  // Start an async spinner to avoid move group interface object complaining about time sync with UR5e.
  // https://github.com/ros-planning/moveit/issues/1187
  // Use more than one thread to avoid MoveIt's move group interface's plan() function freezing.
  // https://github.com/jacknlliu/development-issues/issues/44
  ros::AsyncSpinner spinner(0);
  spinner.start();

  pulse_est::ParticleFilterVelNoise pf(nh);

  ros::waitForShutdown();

  return 0;
}
