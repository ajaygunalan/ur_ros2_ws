#ifndef PULSE_VISUAL_SERVOING_MATHIASSEN_CLASS_H
#define PULSE_VISUAL_SERVOING_MATHIASSEN_CLASS_H

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

#include "pulse_control_utils/ur5e_base_class.hpp"

using cv::Mat;
using cv::Moments;
using Eigen::MatrixXd;

std::string matType2Str(int type);

class Ur5eMathiassenVisualServoingClass : protected Ur5eBaseClass
{
protected:
  ros::Subscriber us_img_sub;
  ros::Time Tcurr;
  ros::Time Tprev;
  ros::NodeHandle nh;
  double time_step = 0.1;
  double v_max_x = 0.4;
  double v_max_y = 0.4;
  double v_max_z = 0.4;
  double w_max_x = 3.5;
  double w_max_y = 3.5;
  double w_max_z = 3.5;
  double theta_star;
  double alpha_star;
  double l;
  double l_star;
  double pixel_x;
  double pixel_z;
  double unknown_loc_y_vel;

  int pcent_counter = 0;
  int SI;
  int fsm_state;
  int unknown_loc_counter;
  int needle_filter_threshold;
  int moment_filter_threshold;
  int needle_kernel_size;
  int moment_kernel_size;
  int img_ct;
  double needle_rad;
  double theta;
  double alpha;
  double alpha_prev;
  double alpha_ideal;
  double L;
  double L_prev;
  double L_ideal;
  double prob_dalpha;
  double prob_dL;
  double prob_high;
  double prob_low;
  double r_lambda;
  double r0;
  double k;
  bool sim;
  bool save_img;
  bool debug_segmentation;
  bool force_control;

  MatrixXd ptip = MatrixXd(2, 1);
  MatrixXd ptip_star = MatrixXd(2, 1);
  MatrixXd pcent_star = MatrixXd(2, 1);
  MatrixXd velocity = MatrixXd(6, 1);
  MatrixXd lambdas = MatrixXd(2, 1);
  MatrixXd eigVectors = MatrixXd(2, 2);
  MatrixXd covI = MatrixXd(2, 2);
  MatrixXd y_neg_positions = MatrixXd(1, 5);
  MatrixXd pcenters = MatrixXd(2, 5);

  cv_bridge::CvImagePtr us_img;
  Mat segmented_img;
  Mat preprocessed_img;
  Mat proc_img_needle_u;
  Mat proc_img_moment_u;
  Mat proc_img_needle_f;
  Mat proc_img_moment_f;
  Moments img_moments;
  moveit_visual_tools::MoveItVisualTools* visual_tools;

  std::stringstream ss;
  std::string save_path;

  int sign(double val);
  void fsmExecution();
  void unknownLocationVelocity();
  void estimateDirectionVelocity();
  void unalignedVelocity();
  void nearAlignedVelocity();
  void alignedVelocity();
  void planPath();
  void moveToInitialPosition();
  void moveHome();
  void getYPosition();
  void publishVelocity();
  MatrixXd Lalpha();
  MatrixXd Lpcent();
  MatrixXd Ltheta();
  MatrixXd Ll();

  void usImgAcqCallback(const sensor_msgs::ImageConstPtr& img);
  void fsm();
  void calculateLambdas();
  void calculateCovI();
  void calculateMoments();
  void calculatePCenter();
  void calculatePTip();
  void calculateAlpha();
  void calculateSI();
  void calculateInitialAlphaTheta();
  void calculateTheta();
  void calculateL();
  void bayesianInference(double& curr, double& prev, double& prob_d_edot);
  // void calculateSignedL();
  // void calculateSignedAlpha();
  // void calculateAlphaThetaL();
  void extendedKalmanFilter();
  void imageSegmentation();
  void imgPreProcessing();
  void rescaleImgRange(Mat& img);
  void needleEnhancingFilter();
  void threshold(Mat img_in, double thresh, Mat& img_out);
  void morphClosing();
  void momentFilter();
  void combineFilteredImgs();

public:
  Ur5eMathiassenVisualServoingClass(
    ros::NodeHandle& nh, std::string action_server_name, std::string ee_name,
    double delta_t, double joint_omega_max, int needle_filter_threshold,
    int moment_filter_threshold, int needle_kernel_size, int moment_kernel_size,
    bool save_img, double r_lambda, double r0, double k, double needle_rad,
    bool debug_segmentation, bool force_control);
  ~Ur5eMathiassenVisualServoingClass();
};

#endif /* PULSE_VISUAL_SERVOING_MATHIASSEN_CLASS_H */
