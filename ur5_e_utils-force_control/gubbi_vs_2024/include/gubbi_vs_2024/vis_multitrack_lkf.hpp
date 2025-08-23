#ifndef PULSE_VIS_MULTI_TRACK_LKF_HPP
#define PULSE_VIS_MULTI_TRACK_LKF_HPP

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "pulse_vs_msgs/KalmanArray.h"

class VisMultiTrackLkf
{
protected:
  ros::NodeHandle nh;
  ros::Subscriber filter_sub;
  ros::Publisher marker_pos_pub;
  ros::Publisher marker_vel_pub;

public:
  VisMultiTrackLkf(ros::NodeHandle& nh);

  void filterSubscriberCallback(pulse_vs_msgs::KalmanArray msg);
};

#endif /* PULSE_VIS_MULTI_TRACK_LKF_HPP */
