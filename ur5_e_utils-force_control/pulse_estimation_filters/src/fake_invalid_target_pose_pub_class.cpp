#include "pulse_estimation_filters/fake_invalid_target_pose_pub_class.hpp"

vs_sim::FakeInvalidTargetPosePublisher::FakeInvalidTargetPosePublisher(ros::NodeHandle& nh) : nh(nh)
{
  this->tgt_pos_pub = this->nh.advertise<pulse_vs_msgs::KalmanArray>("target_pose", 1);
  this->pub_timer = this->nh.createTimer(ros::Duration(0.1), &FakeInvalidTargetPosePublisher::timerCallback, this);
}

void vs_sim::FakeInvalidTargetPosePublisher::timerCallback(const ros::TimerEvent& ev)
{
  pulse_vs_msgs::KalmanArray tgt_pos_msg;

  tgt_pos_msg.header.stamp = ros::Time::now();
  tgt_pos_msg.header.frame_id = "invalid";

  ROS_DEBUG("Publishing fake message containing invalid target pose...");
  this->tgt_pos_pub.publish(tgt_pos_msg);
}
