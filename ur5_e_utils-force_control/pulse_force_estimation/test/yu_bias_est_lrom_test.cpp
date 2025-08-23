/**
 * \file yu_bias_est_lrom_test.cpp
 *
 * \brief Test the LROM-based bias and gravity estimation algorithms using
 * inputs read from a given ROS bag file.
 *
 * \author Mardava Gubbi <mgubbi1@jhu.edu>
 */
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Wrench.h>

#include "pulse_force_estimation/yu_bias_est_lrom_class.hpp"

class YuBiasEstLromTestClass : public YuBiasEstLromClass
{
protected:
  /*
   * \brief Counter of F/T readings from debug topic /yu_bias_est/ft_readings
   */
  int ft_reading_msg_count;

  /*
   * \brief Counter of robot poses from debug topic /yu_bias_est/robot_poses
   */
  int robot_pose_msg_count;

  std::string ros_bag_file_name;

  void readRosBagFile(void)
  {
    Eigen::Isometry3d eig_e_b;
    Eigen::Matrix<double, 6, 1> ft_reading;
    int i0;
    std::string msg_topic;
    rosbag::Bag bag;

    ROS_DEBUG_STREAM("Opening bag file '" << this->ros_bag_file_name << "'...");
    bag.open(this->ros_bag_file_name);

    for(rosbag::MessageInstance const msg_inst : rosbag::View(bag))
    {
      msg_topic = msg_inst.getTopic();

      if(msg_topic.compare("/yu_bias_est/ft_readings") == 0)
      {
        geometry_msgs::Wrench::ConstPtr msg_ptr =
          msg_inst.instantiate<geometry_msgs::Wrench>();

        if(msg_ptr != nullptr)
        {
          ROS_DEBUG(
            "[%3d/%3ld] Populating list of F/T readings...",
            this->ft_reading_msg_count, this->ft_readings_vec.size());
          tf::wrenchMsgToEigen(*msg_ptr, ft_reading);

          for(i0 = 0; i0 < 6; i0++)
          {
            this->ft_readings_vec[this->ft_reading_msg_count][i0] =
              ft_reading(i0);
          }

          this->ft_reading_msg_count++;
        }
      }
      else if(msg_topic.compare("/yu_bias_est/robot_poses") == 0)
      {
        geometry_msgs::Transform::ConstPtr msg_ptr =
          msg_inst.instantiate<geometry_msgs::Transform>();

        if(msg_ptr != nullptr)
        {
          ROS_DEBUG(
            "[%2d/%2ld] Populating list of robot poses...",
            this->robot_pose_msg_count / NUM_FT_SAMPLES_PER_POSE,
            this->eig_e_b_vec.size());
          tf::transformMsgToEigen(*msg_ptr, eig_e_b);
          this->eig_e_b_vec[this->robot_pose_msg_count / NUM_FT_SAMPLES_PER_POSE] =
            eig_e_b;
          this->robot_pose_msg_count++;
        }
      }
      else
      {
        ROS_ERROR_STREAM(
          ""
          << "Unknown topic '" << msg_topic << "' in bag file '"
          << this->ros_bag_file_name << "'.");
      }
    }

    ROS_DEBUG("Closing ROS bag file handler...");
    bag.close();
  }

public:
  YuBiasEstLromTestClass(ros::NodeHandle& nh, std::string ros_bag_file_name)
    : YuBiasEstLromClass(nh)
    , ft_reading_msg_count(0)
    , robot_pose_msg_count(0)
    , ros_bag_file_name(ros_bag_file_name)
  {
    // Stop the timer as there is no robot to be moved.
    ROS_DEBUG("Stopping timer...");
    this->fsm_timer.stop();

    this->readRosBagFile();

    ROS_DEBUG("Estimating sensor bias and gravitational force...");
    this->estimateSensorBiasGravitationalForceUpdate();
  }
};

int main(int argc, char** argv)
{
  std::string ros_bag_file_name;

  // Initialize the node and declare the node handler.
  ros::init(argc, argv, "yu_bias_est_lrom_test_node");
  ros::NodeHandle nh("~");

  // Start an async spinner to avoid move group interface object complaining
  // about time sync with UR5e.
  // https://github.com/ros-planning/moveit/issues/1187
  // Use more than one thread to avoid MoveIt's move group interface's plan()
  // function freezing.
  // https://github.com/jacknlliu/development-issues/issues/44
  ros::AsyncSpinner spinner(0);
  spinner.start();

  if(!nh.getParam("ros_bag_file_name", ros_bag_file_name))
  {
    ROS_ERROR("Could not fetch parameter 'ros_bag_file_name'. Exiting...");
    ros::shutdown();
  }

  // Instantiate the bias compensation and gravity estimation object.
  YuBiasEstLromTestClass y(nh, ros_bag_file_name);

  // Wait for the node to shutdown.
  ros::waitForShutdown();
  delete(&y);

  return 0;
}
