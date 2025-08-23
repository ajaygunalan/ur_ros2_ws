#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sstream>
#include <sensor_msgs/image_encodings.h>

int img_ct;

void img_callback(const sensor_msgs::ImageConstPtr& img)
{
  cv_bridge::CvImagePtr cv_img;
  // Convert ROS image to CV image and save
  try
  {
    cv_img = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  std::stringstream ss;
  ss.str("");  // Clear stringstream
  ss << "/home/jsu30/Documents/ur5_imgs"
     << "/img_" << std::setw(6) << std::setfill('0') << img_ct << ".png";

  ROS_INFO_STREAM("Writing image to " << ss.str() << " ...");
  if(!cv::imwrite(ss.str(), cv_img->image))
  {
    ROS_WARN_STREAM("Error writing image to: " << ss.str());
  }
  else
  {
    // No operation
  }
  img_ct++;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "save_topic_to_img_node");
  ros::NodeHandle nh("~");
  ros::Subscriber sub = nh.subscribe("/disp_img", 1000, img_callback);
  ros::spin();
  return 0;
}
