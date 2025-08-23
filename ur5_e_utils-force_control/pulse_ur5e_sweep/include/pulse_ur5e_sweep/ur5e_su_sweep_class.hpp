/**
 *
 * \file ur5e_su_sweep_class.hpp
 *
 * \brief File containing code to scan the UR5e end effector across a given
 * surface.
 *
 * \author Mardava Gubbi <mgubbi1@jhu.edu>
 */
#ifndef PULSE_UR5E_SU_SWEEP_CLASS_HPP
#define PULSE_UR5E_SU_SWEEP_CLASS_HPP

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <math.h>

#include "pulse_control_utils/ur5e_move_group_if_class.hpp"

#include <chrono>
#include <fstream>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <tf2_ros/transform_listener.h>

class Ur5eSuSweepClass : protected mgi::Ur5eMoveGroupInterfaceClass
{
protected:
  ros::Subscriber das_us_img_sub;
  ros::Timer run_timer;

  cv_bridge::CvImagePtr das_us_img;

  geometry_msgs::TransformStamped ur5e_base_tool0_start_tf;
  geometry_msgs::TransformStamped ur5e_base_tool0_curr_tf;

  std::ofstream transform_fh;

  std::stringstream data_path;

  int grid_position_id[2];
  int max_grid_position_id[2];

  double scan_dist = 2.0;
  bool saveImgTfToFile();

  // TODO: Prune.
  std::string foldername;
  std::string data_dir_name;

public:
  Ur5eSuSweepClass(
    ros::NodeHandle& nh, double scan_dist, int max_x_steps, int max_y_steps);

  ~Ur5eSuSweepClass();

  void dasUsImgAcqCallback(const sensor_msgs::ImageConstPtr& img);

  void runCallback(const ros::TimerEvent& e);
};

#endif /* PULSE_UR5E_SCAN_SU_CLASS_HPP */
