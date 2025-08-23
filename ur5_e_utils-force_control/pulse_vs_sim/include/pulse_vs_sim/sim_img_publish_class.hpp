#ifndef SIM_IMG_PUBLISH_CLASS_HPP
#define SIM_IMG_PUBLISH_CLASS_HPP

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

class PaimdbSimImgPublisher
{
protected:
  int num_sim_img;
  int sim_img_id;
  int zero_pad_width_px;
  ros::NodeHandle nh;
  image_transport::ImageTransport* it;
  image_transport::Publisher sim_img_pub;
  ros::Timer sim_img_timer;
  std::string sim_img_dir_name;

public:
  PaimdbSimImgPublisher(ros::NodeHandle& nh);
  ~PaimdbSimImgPublisher();

  void timerCallback(const ros::TimerEvent& e);
};

#endif /* SIM_IMG_PUBLISH_CLASS_HPP */
