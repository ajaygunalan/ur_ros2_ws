#include "pulse_vs_sim/pa_chdat_gen_class.hpp"

namespace vs_sim
{
class RefDownshiftTest : protected PaChannelDataGenerator
{
public:
  /**
   *
   * \brief Construct an object of the type `PaChannelDataGenerator`
   *
   * \arg[in] nh The ROS node handle
   *
   */
  RefDownshiftTest(ros::NodeHandle& nh);

  /**
   *
   * \brief Timer callback function to execute photoacoustic channel data generation and publishing
   *
   * \arg[in] e The ROS timer event (unused)
   *
   */
  void timerCallback(const ros::TimerEvent& e);
};
}  // namespace vs_sim

vs_sim::RefDownshiftTest::RefDownshiftTest(ros::NodeHandle& nh) : PaChannelDataGenerator(nh)
{
  this->pub_timer.stop();
  this->pub_timer = this->nh.createTimer(ros::Duration(1.0), &RefDownshiftTest::timerCallback, this);
}

void vs_sim::RefDownshiftTest::timerCallback(const ros::TimerEvent& e)
{
  // Original and processed images of channel data frames corresponding to source, reflection artifact, and output.
  cv::Mat output_img;
  cv::Mat output_img_rescaled;
  cv::Mat ref_img;
  cv::Mat ref_img_shifted;

  double output_min_val;
  double output_max_val;
  cv::Point output_min_loc;
  cv::Point output_max_loc;

  int num_downshift_px = 100;

  // Image message to be published to ROS topic.
  sensor_msgs::ImagePtr img_msg;

  Eigen::Vector3d ref_pos_probe;
  ref_pos_probe(0) = 0.0;
  ref_pos_probe(1) = 0.0;
  ref_pos_probe(2) = 30e-3;

  // Generate the photoacoustic channel data frame corresponding to the given location.
  this->getChannelDataFrame(ref_pos_probe, ref_img, 0);

  // Axially downshift the channel data frame by a fixed amount.
  ROS_DEBUG("Axially downshifting reflection artifact by %d px...", num_downshift_px);
  this->getChannelDataFrame(ref_pos_probe, ref_img_shifted, 100);

  // Concatenate the images.
  ROS_DEBUG("Concatenating original and axially downshifted images...");
  cv::hconcat(ref_img, ref_img_shifted, output_img);

  // Rescale, typecast, and publish the resulting MONO-8 image.
  ROS_DEBUG("Rescaling and typecasting channel data frame...");
  cv::minMaxLoc(output_img, &output_min_val, &output_max_val, &output_min_loc, &output_max_loc);
  output_img = output_img - output_min_val;
  output_img = output_img * (255.0 / (output_max_val - output_min_val));
  output_img.convertTo(output_img_rescaled, CV_8UC1);

  img_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", output_img_rescaled).toImageMsg();

  ROS_DEBUG("Publishing channel data frame...");
  this->pa_chdat_pub.publish(img_msg);
}

int main(int argc, char** argv)
{
  // Initialize the node and declare the node handler.
  ros::init(argc, argv, "pa_chdat_gen_test");
  ros::NodeHandle nh("~");

  // Start an async spinner to avoid move group interface object complaining about time sync with UR5e.
  // https://github.com/ros-planning/moveit/issues/1187
  // Use more than one thread to avoid MoveIt's move group interface's plan() function freezing.
  // https://github.com/jacknlliu/development-issues/issues/44
  ros::AsyncSpinner spinner(0);
  spinner.start();

  vs_sim::RefDownshiftTest pcdgt(nh);

  ros::waitForShutdown();
  delete(&pcdgt);

  return 0;
}
