#ifndef PULSE_LAB_UR5_E_SU_VISUAL_SERVOING_CLASS_H
#define PULSE_LAB_UR5_E_SU_VISUAL_SERVOING_CLASS_H

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/QR>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32.h>
#include <chrono>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include <boost/format.hpp>
#include <sstream>
#include <iomanip>
#include <string>
#include <cmath>
#include <algorithm>
#include <fstream>

#include "pulse_control_utils/ur5e_base_class.hpp"

using cv::Mat;
using cv::Moments;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::VectorXi;

std::string matType2Str(int type);

class Ur5eSuVisualServoingClass : protected Ur5eBaseClass
{
protected:
  ros::Subscriber us_img_sub;
  ros::Publisher disp_img_pub;
  ros::NodeHandle nh;
  double time_step = 0.1;
  double v_max_x = 0.4;
  double v_max_y = 0.4;
  double v_max_z = 0.4;
  double w_max_x = 3.5;
  double w_max_y = 3.5;
  double w_max_z = 3.5;
  double pixel_x;
  double pixel_z;
  double unknown_loc_vy;
  double max_aligning_wz;
  double aligning_wz;
  double find_tip_vx;
  double eps_pcent;
  double eps_si;
  double si_diff;
  double pcent_xdiff;
  double ptip_xdiff;
  int A0;
  int aligned_ct = 0;
  int align_counter = 0;
  int aligned_ct_max;
  int buf_counter = 0;
  int buf_length = 5;
  int si_star;
  int fsm_state;
  int unknown_loc_counter = 0;
  int needle_filter_threshold;
  int moment_filter_threshold;
  int needle_kernel_size;
  int moment_kernel_size;
  int img_ct = 0;
  bool sim;
  bool save_img;
  bool debug_segmentation;
  bool buf_filled;

  VectorXd ptip = VectorXd(2);
  VectorXd ptip_star = VectorXd(2);
  VectorXd pcenter = VectorXd(2);
  VectorXd pcent_star = VectorXd(2);
  VectorXd lambdas = VectorXd(2);
  VectorXd velocity = VectorXd(6);
  VectorXd tg_areas = VectorXd(buf_length);
  VectorXi delAs = VectorXi(buf_length);
  VectorXd SIs = VectorXd(buf_length);
  MatrixXd eigVectors = MatrixXd(2, 2);
  MatrixXd covI = MatrixXd(2, 2);

  cv_bridge::CvImagePtr us_img;
  Moments img_moments;
  Mat segmented_img;
  Mat preprocessed_img;
  Mat proc_img_needle_u;
  Mat proc_img_moment_u;
  Mat proc_img_needle_f;
  Mat proc_img_moment_f;

  std::stringstream ss;
  std::string save_path;

  int sign(double val);
  void planPath();
  void moveToInitialPosition();
  void moveHome();
  void publishVelocity();

  void fsm();
  void fsmExecution();
  void unknownLocationVelocity();
  void centeringVelocity();
  void aligningVelocity();
  void findTipVelocity();
  void centerTipVelocity();

  void usImgAcqCallback(const sensor_msgs::ImageConstPtr& img);
  void calculateSVD();
  void calculateCovI();
  void calculateMoments();
  void calculatePCenter();
  void calculatePTip();
  void calculateSI();

  void imageSegmentation();
  void imgPreProcessing();
  void rescaleImgRange(Mat& img);
  void needleEnhancingFilter();
  void threshold(Mat img_in, double thresh, Mat& img_out);
  void morphClosing();
  void momentFilter();
  void combineFilteredImgs();
  void saveDebugData();

public:
  Ur5eSuVisualServoingClass(
    ros::NodeHandle& nh, std::string action_server_name, std::string ee_name,
    double delta_t, double joint_omega_max, int needle_filter_threshold,
    int moment_filter_threshold, int needle_kernel_size, int moment_kernel_size,
    bool save_img, std::string save_path, bool debug_segmentation,
    double unknown_loc_vy, double find_tip_vx, double max_aligning_wz);
  ~Ur5eSuVisualServoingClass();
};

#endif /* PULSE_LAB_UR5_E_SU_VISUAL_SERVOING_CLASS_H */
