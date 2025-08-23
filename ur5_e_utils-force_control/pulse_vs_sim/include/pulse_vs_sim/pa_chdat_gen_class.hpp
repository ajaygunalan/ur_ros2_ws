/**
 *
 * \file pa_chdat_gen_class.hpp
 *
 * \brief Photoacoustic channel data frame generation
 *
 * \references
 * 1. Allman et al., IEEE Transactions on Medical Imaging, 2018.
 *
 * \author Mardava Gubbi <mgubbi1@jhu.edu>
 *
 */
#ifndef PULSE_PA_CHDAT_GEN_CLASS_HPP
#define PULSE_PA_CHDAT_GEN_CLASS_HPP

#include <Eigen/Geometry>
#include <highfive/highfive.hpp>
#include <random>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <tf2_ros/transform_listener.h>

#define PULSE_SRC_REF_POS_STR_LEN 16u

namespace vs_sim
{
class PaChannelDataGenerator
{
protected:
  /**
   * \brief Time period of timer for image generation [s]
   */
  double delta_t;

  /**
   * \brief Speed of sound to be used in simulation
   */
  double sim_speed_of_sound;

  /**
   * \brief Sampling frequency of ultrasound transducer
   */
  double trans_samp_freq;

  /**
   * \brief Desired height of output images [px]
   */
  int img_height_px;

  /**
   * \brief Desired height of output images [px]
   */
  int img_width_px;

  /**
   * \brief Image transport object handler
   *
   * \details Not sure if this is useful, and apparently it clashes with ROS bag recording.
   */
  // image_transport::ImageTransport* img_nh;

  /**
   * \brief ROS node handler
   */
  ros::NodeHandle nh;

  /**
   * \brief ROS publisher for photoacoustic channel data frames
   */
  // image_transport::Publisher pa_chdat_pub;
  ros::Publisher pa_chdat_pub;

  /**
   * \brief ROS timer to execute generation and publishing of photoacoustic channel data frames
   */
  ros::Timer pub_timer;

  /**
   * \brief Directory containing simulated dataset from which channel data frames are to be generated
   */
  std::string dataset_dir_name;

  /**
   * \brief Names of TF frames corresponding to ultrasound transducer and point source.
   */
  std::string probe_frame_name;
  std::string pt_src_frame_name;

  /**
   * \brief Vectors containing simulated lateral, elevation, and axial positions to be used to generate channel data
   */
  std::vector<double> pt_src_x_vec;
  std::vector<double> pt_src_y_vec;
  std::vector<double> pt_src_z_vec;

  /**
   * \brief Random number generator and distributions for reflection artifact position generation and amplitude scaling
   */
  std::default_random_engine rand_num_gen;
  std::normal_distribution<double>* ref_pos_distribution;
  std::uniform_real_distribution<double>* amp_scale_distribution;

  /**
   * \brief Transform buffer required to get current point source location relative to probe
   */
  tf2_ros::Buffer tf_buffer;

  /**
   * \brief Transform listener required to get current point source location relative to probe
   */
  tf2_ros::TransformListener* tf_listener;

  /**
   *
   * \brief Get the channel data frame corresponding to the given source or reflection artifact position.
   *
   * \arg[in] src_ref_pos The position of the source or reflection artifact to be simulated
   * \arg[in] output_img The corresponding photoacoustic channel data frame corresponding to the given position
   * \arg[in] num_downshift_px The number of pixels by which the input image is to be axially downshifted, if artifacts,
   * zero otherwise
   *
   * \return true if the channel data frame was successfully fetched, false otherwise
   *
   */
  bool getChannelDataFrame(Eigen::Vector3d src_ref_pos, cv::Mat& output_img, int num_downshift_px);

  /**
   *
   * \brief Find the index of the element in the vector `v` closest in value to the desired value `x`
   *
   * \arg[in] x The desired value for which the search is to be performed
   * \arg[in] v The vector in which the search is to be performed, assumed to be sorted in ascending order
   *
   */
  int binarySearch(double x, std::vector<double> v);

  /**
   *
   * \brief Axially downshift the channel data frame by the given number of pixels.
   *
   * \details This function models reflection artifacts as described in [1].
   *
   * \arg[in] input_img The image to be axially downshifted
   * \arg[in] output_img The image after axial downshifting
   * \arg[in] num_downshift_px The number of pixels by which the input image is to be axially downshifted
   * \arg[in] mean_amp The mean amplitude in the image with which to populate the top of the downshifted image
   *
   */
  void downshiftChannelDataFrame(cv::Mat& input_img, cv::Mat& output_img, int num_downshift_px, double mean_amp);

public:
  /**
   *
   * \brief Construct an object of the type `PaChannelDataGenerator`
   *
   * \arg[in] nh The ROS node handle
   *
   */
  PaChannelDataGenerator(ros::NodeHandle& nh);

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

#endif /* PULSE_PA_CHDAT_GEN_CLASS_HPP */
