#include "pulse_estimation_filters/fake_invalid_target_pose_pub_class.hpp"

int main(int argc, char** argv)
{
  // Initialize the node and declare the node handler.
  ros::init(argc, argv, "filter");
  ros::NodeHandle nh("~");

  // Start an async spinner to avoid move group interface object complaining about time sync with UR5e.
  // https://github.com/ros-planning/moveit/issues/1187
  // Use more than one thread to avoid MoveIt's move group interface's plan() function freezing.
  // https://github.com/jacknlliu/development-issues/issues/44
  ros::AsyncSpinner spinner(0);
  spinner.start();

  vs_sim::FakeInvalidTargetPosePublisher f(nh);

  ros::waitForShutdown();

  return 0;
}
