#include "gubbi_icra_2021/bbox_fcvs_class.hpp"

int main(int argc, char** argv)
{
  // Initialize the node and declare the node handler.
  ros::init(argc, argv, "visual_servoing_node");
  ros::NodeHandle nh("~");

  // Start an async spinner to avoid move group interface object complaining
  // about time sync with UR5e.
  // https://github.com/ros-planning/moveit/issues/1187
  // Use more than one thread to avoid MoveIt's move group interface's plan()
  // function freezing.
  // https://github.com/jacknlliu/development-issues/issues/44
  ros::AsyncSpinner spinner(0);
  spinner.start();

  // Instantiate the bounding box- and force control-based visual servoing object.
  vs::BoundingBoxForceControlVisualServoingClass b(nh);

  // Wait for the node to shutdown.
  ros::waitForShutdown();

  return 0;
}
