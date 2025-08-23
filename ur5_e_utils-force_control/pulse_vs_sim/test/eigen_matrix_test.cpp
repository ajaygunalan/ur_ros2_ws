#include <ros/ros.h>
#include <Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>

class EigenMatrixTestClass
{
protected:
  ros::NodeHandle nh;

public:
  EigenMatrixTestClass(ros::NodeHandle& nh);
};

EigenMatrixTestClass::EigenMatrixTestClass(ros::NodeHandle& nh) : nh(nh)
{
  Eigen::Isometry3d base_trans_probe;
  Eigen::Isometry3d base_trans_src;
  Eigen::Isometry3d probe_trans_src;
  Eigen::Quaterniond base_quat_probe(sqrt(0.5), 0, 0, sqrt(0.5));
  Eigen::Vector3d base_pos_probe;
  Eigen::Vector3d probe_pos_src;

  base_trans_probe.setIdentity();

  base_pos_probe(0) = 1.0;
  base_pos_probe(1) = 0.0;
  base_pos_probe(2) = 0.0;
  base_trans_probe.translate(base_pos_probe);
  ROS_INFO_STREAM("`base_trans_probe` after translation:" << std::endl << base_trans_probe.matrix());

  base_trans_probe.rotate(base_quat_probe);
  ROS_INFO_STREAM("`base_trans_probe` after rotation:" << std::endl << base_trans_probe.matrix());

  probe_pos_src(0) = 1.0;
  probe_pos_src(1) = -1.0;
  probe_pos_src(2) = 0.0;
  base_trans_probe.translate(probe_pos_src);
  ROS_INFO_STREAM("`base_trans_probe` at source position:" << std::endl << base_trans_probe.matrix());
}

int main(int argc, char** argv)
{
  // Initialize the node and declare the node handler.
  ros::init(argc, argv, "eigen_matrix_test");
  ros::NodeHandle nh("~");

  // Start an async spinner to avoid move group interface object complaining about time sync with UR5e.
  // https://github.com/ros-planning/moveit/issues/1187
  // Use more than one thread to avoid MoveIt's move group interface's plan() function freezing.
  // https://github.com/jacknlliu/development-issues/issues/44
  ros::AsyncSpinner spinner(0);
  spinner.start();

  EigenMatrixTestClass emt(nh);

  ros::waitForShutdown();
  delete(&emt);

  return 0;
}
