#include "gubbi_vs_2024/vis_multitrack_lkf.hpp"

VisMultiTrackLkf::VisMultiTrackLkf(ros::NodeHandle& nh) : nh(nh)
{
  std::string lkf_topic_name;

  if(this->nh.hasParam("lkf_topic_name"))
  {
    this->nh.getParam("lkf_topic_name", lkf_topic_name);
  }
  else
  {
    lkf_topic_name = "/filter/target_pose";
  }

  this->marker_pos_pub = this->nh.advertise<visualization_msgs::Marker>("filter_pos_markers", 1);
  this->marker_vel_pub = this->nh.advertise<visualization_msgs::Marker>("filter_vel_markers", 1);
  this->filter_sub = this->nh.subscribe(lkf_topic_name, 1, &VisMultiTrackLkf::filterSubscriberCallback, this);
}

void VisMultiTrackLkf::filterSubscriberCallback(pulse_vs_msgs::KalmanArray msg)
{
  int i0;
  geometry_msgs::Point p;

  for(i0 = 0; i0 < msg.filters.size(); i0++)
  {
    visualization_msgs::Marker marker_msg;
    marker_msg.header.frame_id = msg.header.frame_id;
    marker_msg.header.stamp = ros::Time::now();
    marker_msg.ns = "multitrack_lkf_position";
    marker_msg.id = i0;
    marker_msg.type = visualization_msgs::Marker::SPHERE;
    marker_msg.lifetime = ros::Duration(0.25);
    marker_msg.action = visualization_msgs::Marker::ADD;
    marker_msg.pose.position.x = msg.filters[i0].kalman_state[0];
    marker_msg.pose.position.y = msg.filters[i0].kalman_state[1];
    marker_msg.pose.position.z = msg.filters[i0].kalman_state[2];
    marker_msg.pose.orientation.x = 0.0;
    marker_msg.pose.orientation.y = 0.0;
    marker_msg.pose.orientation.z = 0.0;
    marker_msg.pose.orientation.w = 1.0;

    // The current covariance field is vectorized from a 9x9 matrix with position covariances in the top left corner.
    // We extract (0, 0), (1, 1), and (2, 2) as elements 0, 10, and 20, respectively.
    marker_msg.scale.x = 5.0 * sqrt(msg.filters[i0].kalman_covariance[0]);
    marker_msg.scale.y = 5.0 * sqrt(msg.filters[i0].kalman_covariance[10]);
    marker_msg.scale.z = 5.0 * sqrt(msg.filters[i0].kalman_covariance[20]);

    if(msg.filters[i0].fsm_state == "LKF_INITIALIZE")
    {
      marker_msg.color.r = 1.0;
      marker_msg.color.g = 1.0;
      marker_msg.color.b = 1.0;
      marker_msg.color.a = 0.2;
    }
    else if(msg.filters[i0].fsm_state == "LKF_TRACKING")
    {
      marker_msg.color.r = 0.0;
      marker_msg.color.g = 1.0;
      marker_msg.color.b = 0.0;
      marker_msg.color.a = 1.0;
    }
    else if(msg.filters[i0].fsm_state == "LKF_MISSED_ONE")
    {
      marker_msg.color.r = 0.0;
      marker_msg.color.g = 0.5;
      marker_msg.color.b = 0.5;
      marker_msg.color.a = 0.2;
    }
    else if(msg.filters[i0].fsm_state == "LKF_MISSED_TWO")
    {
      marker_msg.color.r = 0.0;
      marker_msg.color.g = 0.0;
      marker_msg.color.b = 1.0;
      marker_msg.color.a = 0.2;
    }
    else if(msg.filters[i0].fsm_state == "LKF_DELETE")
    {
      marker_msg.color.r = 1.0;
      marker_msg.color.g = 0.0;
      marker_msg.color.b = 0.0;
      marker_msg.color.a = 0.2;
    }
    else
    {
      ROS_ERROR("Unknown LKF state '%s'.", msg.filters[i0].fsm_state.c_str());
    }

    this->marker_pos_pub.publish(marker_msg);

    // Now publish the velocity as a line.
    marker_msg.ns = "multitrack_lkf_velocity";
    marker_msg.type = visualization_msgs::Marker::LINE_STRIP;
    marker_msg.pose.position.x = 0.0;
    marker_msg.pose.position.y = 0.0;
    marker_msg.pose.position.z = 0.0;
    marker_msg.pose.orientation.x = 0.0;
    marker_msg.pose.orientation.y = 0.0;
    marker_msg.pose.orientation.z = 0.0;
    marker_msg.pose.orientation.w = 1.0;

    p.x = msg.filters[i0].kalman_state[0];
    p.y = msg.filters[i0].kalman_state[1];
    p.z = msg.filters[i0].kalman_state[2];
    marker_msg.points.push_back(p);
    p.x += msg.filters[i0].kalman_state[3];
    p.y += msg.filters[i0].kalman_state[4];
    p.z += msg.filters[i0].kalman_state[5];
    marker_msg.points.push_back(p);

    // LINE_STRIP markers use only the x component of scale, for the line width.
    marker_msg.scale.x = 0.001;
    marker_msg.scale.y = 0.0;
    marker_msg.scale.z = 0.0;

    this->marker_vel_pub.publish(marker_msg);
  }
}

int main(int argc, char** argv)
{
  // Initialize the node and declare the node handler.
  ros::init(argc, argv, "vis_multitrack_lkf_node");
  ros::NodeHandle nh("~");

  // Start an async spinner to avoid move group interface object complaining about time sync with UR5e.
  // https://github.com/ros-planning/moveit/issues/1187
  // Use more than one thread to avoid MoveIt's move group interface's plan() function freezing.
  // https://github.com/jacknlliu/development-issues/issues/44
  ros::AsyncSpinner spinner(0);
  spinner.start();

  VisMultiTrackLkf vis(nh);

  ros::waitForShutdown();

  return 0;
}
