/**
 *
 * \file gubbi.cpp
 *
 * \brief Test the photoacoustic visual servoing system developed by members of
 * the PULSE Lab.
 *
 * \details See Gubbi and Bell, "Deep Learning-Based Photoacoustic Visual
 * Servoing: Using Outputs from Raw Sensor Data as Inputs to a Robot
 * Controller", IEEE ICRA 2021.
 *
 * This node publishes dummy messages to the topics /deep_learning/target_pose
 * and /amplitude/target_pose to test the robotic control system implemented in
 * this package. This node can test the target tracking capability of the system
 * with one- or multi-dimensional sinusoidally oscillating targets. In addition,
 * this node can test the search pattern of the system by publishing messages
 * with the frame_id field set to "invalid".
 *
 * \author Mardava Gubbi <mgubbi1@jhu.edu>
 *
 */
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

class GubbiDummyPublisher
{
protected:
  bool axes_enabled[3];
  bool valid_pose;
  double tgt_pose_amplitude;
  double tgt_pose_omega;
  ros::NodeHandle nh;
  ros::Publisher pub;
  ros::Timer timer;

public:
  GubbiDummyPublisher(
    ros::NodeHandle& nh, std::string tgt_pose_topic_name, double publish_time,
    double tgt_pose_amplitude, double tgt_pose_omega, bool axes_enabled[3],
    bool valid_pose);

  void timerCallback(const ros::TimerEvent& e);
};

GubbiDummyPublisher::GubbiDummyPublisher(
  ros::NodeHandle& nh, std::string tgt_pose_topic_name, double publish_time,
  double tgt_pose_amplitude, double tgt_pose_omega, bool axes_enabled[3],
  bool valid_pose)
  : nh(nh)
  , tgt_pose_amplitude(tgt_pose_amplitude)
  , tgt_pose_omega(tgt_pose_omega)
  , valid_pose(valid_pose)
{
  int i0;

  for(i0 = 0; i0 < 3; ++i0)
  {
    this->axes_enabled[i0] = axes_enabled[i0];
  }

  this->pub =
    this->nh.advertise<geometry_msgs::PoseStamped>(tgt_pose_topic_name, 10);
  this->timer = this->nh.createTimer(
    ros::Duration(publish_time), &GubbiDummyPublisher::timerCallback, this);
}

void GubbiDummyPublisher::timerCallback(const ros::TimerEvent& e)
{
  double tgt_pose;
  int i0;
  geometry_msgs::PoseStamped msg;
  ros::Time t = ros::Time::now();

  tgt_pose = this->tgt_pose_amplitude * sin(this->tgt_pose_omega * t.toSec());

  // Fill in the message header based on whether we are testing the tracking
  // or the search pattern (determined in the launch file).
  if(this->valid_pose)
  {
    msg.header.frame_id = "probe";
  }
  else
  {
    msg.header.frame_id = "invalid";
  }

  msg.header.stamp = t;

  // Fill in each axis which is to be tested.
  if(this->axes_enabled[0])
  {
    msg.pose.position.x = tgt_pose;
  }
  else
  {
    msg.pose.position.x = 0.0;
  }

  if(this->axes_enabled[1])
  {
    msg.pose.position.y = tgt_pose;
  }
  else
  {
    msg.pose.position.y = 0.0;
  }

  if(this->axes_enabled[2])
  {
    msg.pose.position.z = tgt_pose;
  }
  else
  {
    msg.pose.position.z = 0.0;
  }

  // The target is a point target, so do not bother with orientation for now.
  msg.pose.orientation.w = 1.0;
  msg.pose.orientation.x = 0.0;
  msg.pose.orientation.y = 0.0;
  msg.pose.orientation.z = 0.0;

  ROS_DEBUG_STREAM("Publishing generated message...");
  this->pub.publish(msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gubbi_dummy_publisher_node");
  ros::NodeHandle nh;
  bool axes_enabled[3];
  bool test_search;
  double publish_time;
  double tgt_pose_amplitude;
  double tgt_pose_omega;
  std::string tgt_pose_topic_name;

  if(nh.hasParam("/gubbi_test/test_x"))
  {
    nh.getParam("/gubbi_test/test_x", axes_enabled[0]);
  }
  else
  {
    axes_enabled[0] = false;
  }

  if(nh.hasParam("/gubbi_test/test_y"))
  {
    nh.getParam("/gubbi_test/test_y", axes_enabled[1]);
  }
  else
  {
    axes_enabled[1] = false;
  }

  if(nh.hasParam("/gubbi_test/test_z"))
  {
    nh.getParam("/gubbi_test/test_z", axes_enabled[2]);
  }
  else
  {
    axes_enabled[2] = false;
  }

  if(nh.hasParam("/gubbi_test/test_search"))
  {
    nh.getParam("/gubbi_test/test_search", test_search);
  }
  else
  {
    test_search = false;
  }

  if(nh.hasParam("/gubbi_test/tgt_pose_amplitude"))
  {
    nh.getParam("/gubbi_test/tgt_pose_amplitude", tgt_pose_amplitude);
  }
  else
  {
    // Default sinusoidal oscillation amplitude of 5 cm.
    tgt_pose_amplitude = 0.05;
  }

  if(nh.hasParam("/gubbi_test/tgt_pose_omega"))
  {
    nh.getParam("/gubbi_test/tgt_pose_omega", tgt_pose_omega);
  }
  else
  {
    // Default sinusoidal oscillation period of 10 s.
    tgt_pose_omega = 2.0 * M_PI / 10.0;
  }

  if(nh.hasParam("/visual_servoing/tgt_loc_ns"))
  {
    nh.getParam("/visual_servoing/tgt_loc_ns", tgt_pose_topic_name);
  }
  else
  {
    tgt_pose_topic_name = "/deep_learning";
  }

  tgt_pose_topic_name += "/target_pose";

  if(nh.hasParam("/gubbi_test/publish_time"))
  {
    nh.getParam("/gubbi_test/publish_time", publish_time);
  }
  else
  {
    // Default publish time matching pulse repetition rate of Opotek PHOCUS
    // Mobile laser (10 Hz).
    publish_time = 0.1;
  }

  GubbiDummyPublisher g(
    nh, tgt_pose_topic_name, publish_time, tgt_pose_amplitude, tgt_pose_omega,
    axes_enabled, !test_search);

  ros::spin();

  return 0;
}
