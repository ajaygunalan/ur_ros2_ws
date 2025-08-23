#include "gubbi_icra_2021/mathiassen_class.h"

Ur5eMathiassenVisualServoingClass::Ur5eMathiassenVisualServoingClass(
  ros::NodeHandle& nh, std::string action_server_name, std::string ee_name,
  double delta_t, double joint_omega_max, int needle_filter_threshold,
  int moment_filter_threshold, int needle_kernel_size, int moment_kernel_size,
  bool save_img, double r_lambda, double r0, double k, double needle_rad,
  bool debug_segmentation, bool force_control)
  : Ur5eBaseClass(nh, action_server_name, ee_name, delta_t, joint_omega_max)
  , moment_filter_threshold(moment_filter_threshold)
  , needle_filter_threshold(needle_filter_threshold)
  , moment_kernel_size(moment_kernel_size)
  , needle_kernel_size(needle_kernel_size)
  , save_img(save_img)
  , r_lambda(r_lambda)
  , r0(r0)
  , k(k)
  , needle_rad(needle_rad)
  , debug_segmentation(debug_segmentation)
  , force_control(force_control)
{
  this->sim = (action_server_name == "/arm_controller/follow_joint_trajectory");

  ROS_WARN_STREAM("moment=" << this->moment_filter_threshold);
  ROS_WARN_STREAM("needle=" << this->needle_filter_threshold);
  ROS_WARN_STREAM("moment_kernel=" << this->moment_kernel_size);
  ROS_WARN_STREAM("needle_kernel=" << this->needle_kernel_size);
  ROS_WARN_STREAM("save_img=" << this->save_img);

  // Variable initializations
  this->prob_high = 0.75;
  this->prob_low = 0.25;
  this->pcent_counter = 0;
  this->pcenters = MatrixXd::Zero(2, 5);  // Set pcenters to all 0s
  this->y_neg_positions = MatrixXd::Zero(1, 5);

  this->pixel_x = 0.000385;  // [m/px]
  this->pixel_z = 0.000385;  // [m/px]
  this->alpha_star = 0;
  this->theta_star = 0;                // [rad]
  this->pcent_star(0, 0) = 247 / 2.0;  // Center of the image
  this->pcent_star(1, 0) = 156 / 2.0;
  this->l_star = 0;
  this->ptip_star(0, 0) = 247 / 2.0;  // Center of the image
  this->ptip_star(1, 0) = 156 / 2.0;
  this->unknown_loc_y_vel = 0.002;  // TODO: Put this on the ROS param server.
  this->unknown_loc_counter = 0;

  this->segmented_img = Mat::zeros(156, 247, CV_8U);
  this->proc_img_needle_f =
    Mat::zeros(this->segmented_img.rows, this->segmented_img.cols, CV_32F);
  this->proc_img_needle_u =
    Mat::zeros(this->segmented_img.rows, this->segmented_img.cols, CV_8U);
  this->proc_img_moment_f =
    Mat::zeros(this->segmented_img.rows, this->segmented_img.cols, CV_32F);
  this->proc_img_moment_u =
    Mat::zeros(this->segmented_img.rows, this->segmented_img.cols, CV_8U);

  ROS_INFO_STREAM("Subscribing to US image...");
  // Subscribe to verasonics us_img topic
  this->us_img_sub = this->nh.subscribe(
    "/verasonics/us_img", 1000,
    &Ur5eMathiassenVisualServoingClass::usImgAcqCallback, this);

  // For simulation, set robot to scanning position
  if(this->sim)
  {
    ROS_INFO_STREAM("Moving to initial scanning position...");
    this->moveToInitialPosition();
  }
  else
  {
    // No operation
  }
}

Ur5eMathiassenVisualServoingClass::~Ur5eMathiassenVisualServoingClass()
{
}

void Ur5eMathiassenVisualServoingClass::usImgAcqCallback(
  const sensor_msgs::ImageConstPtr& img)
{
  // Convert ROS image to CV image and save
  try
  {
    this->us_img = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    this->Tprev = this->Tcurr;
    this->Tcurr = img->header.stamp;
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());

    return;
  }

  ROS_INFO_STREAM("Segmenting image...");
  this->imageSegmentation();

  ROS_INFO_STREAM("Calculating image moments...");
  this->calculateMoments();

  this->fsm();
}

void Ur5eMathiassenVisualServoingClass::fsm()
{
  ROS_INFO_STREAM("Running finite state machine...");

  if(this->img_moments.m00 < 1e-6)
  {
    // If M00 = 0, location is unknown (State 1)
    this->fsm_state = 1;
  }
  else
  {
    // Check conditions for advancing to the next state
    this->calculatePCenter();
    this->getYPosition();
    this->calculateLambdas();
    this->calculateL();
    this->calculatePTip();

    switch(this->fsm_state)
    {
      case 1:
        // If previous state was 1, check the sign of the moment 00
        if(this->img_moments.m00 > 1e-6)
        {
          this->fsm_state = 2;
        }
        else
        {
          // No operation
        }

        break;
      case 2:
        this->calculateInitialAlphaTheta();

        // If previous state was 2, check the -velocity_y to advance to state 3
        if(abs(this->velocity(1)) < 0.001)
        {
          this->fsm_state = 3;
        }
        else
        {
          // No operation
        }

        break;
      case 3:
        this->calculateAlpha();
        this->calculateTheta();

        // If previous state was 3, check the lambda ratio to advance to state 4
        // or go back to 2
        if((this->lambdas(1) / this->lambdas(0)) > this->r_lambda)
        {
          this->fsm_state = 4;
        }
        else if((this->lambdas(1) / this->lambdas(0)) < (this->k * this->r0))
        {
          this->fsm_state = 2;
        }
        else
        {
          // No operation
        }

        break;
      case 4:
        // If previous state was 4, check alpha to advance to state 5, or check
        // lambda ratio to go back to 3
        this->calculateAlpha();
        this->calculateTheta();

        if(this->alpha < 0.2)
        {
          this->fsm_state = 5;
        }
        else
        {
          if((this->lambdas(1) / this->lambdas(0)) <= this->r_lambda)
          {
            this->fsm_state = 3;
          }
          else
          {
            // No operation
          }
        }

        break;
      case 5:
        // If previous state was 5, check SI to go back to state 4
        // SI = number of sides the needle touches the US img
        this->calculateSI();
        this->calculateAlpha();
        this->calculateTheta();

        if(this->SI == 0)
        {
          this->fsm_state = 4;
        }
        else
        {
          // No operation
        }

        break;
      default:
        this->fsm_state = 1;

        break;
    }

    if(this->fsm_state != 1)
    {
      this->extendedKalmanFilter();
    }
  }

  this->fsmExecution();
}

void Ur5eMathiassenVisualServoingClass::fsmExecution()
{
  ROS_INFO_STREAM("fsmExecution...");

  switch(this->fsm_state)
  {
    case 1:
      this->unknownLocationVelocity();
      break;

    case 2:
      this->estimateDirectionVelocity();
      break;

    case 3:
      this->unalignedVelocity();
      break;

    case 4:
      this->nearAlignedVelocity();
      break;

    case 5:
      this->alignedVelocity();
      break;
  }

  ROS_INFO("Planning path...");
  if(!this->debug_segmentation)
  {
    if(this->force_control)
    {
      // Remove control of translational z, rotational x, y axes
      this->velocity(2) = 0.0;
      this->velocity(3) = 0.0;
      this->velocity(4) = 0.0;
    }
    else
    {
      // Decrease control of translational z, rotational x, y axes
      this->velocity(2) *= 0.5;
      this->velocity(3) *= 0.5;
      this->velocity(4) *= 0.5;
    }
    this->planPath();
  }
}

void Ur5eMathiassenVisualServoingClass::getYPosition()
{
  const Eigen::Isometry3d fwd_kin = this->getGlobalLinkTransform();
  this->y_neg_positions(this->pcent_counter) = -fwd_kin.translation().y();
}

void Ur5eMathiassenVisualServoingClass::calculateLambdas()
{
  MatrixXd eigVec;
  double temp;

  this->calculateCovI();
  Eigen::JacobiSVD<MatrixXd> svd(this->covI, Eigen::ComputeFullV);
  this->lambdas = svd.singularValues();
  eigVec = svd.matrixV();

  // Sort small to large
  if(this->lambdas(0, 0) > this->lambdas(1, 0))
  {
    temp = this->lambdas(0, 0);
    this->lambdas(0, 0) = this->lambdas(1, 0);
    this->lambdas(1, 0) = temp;
    this->eigVectors.block(0, 0, 2, 1) = eigVec.block(0, 1, 2, 1);
    this->eigVectors.block(0, 1, 2, 1) = eigVec.block(0, 0, 2, 1);
  }
  else
  {
    this->eigVectors = eigVec;
  }
}

void Ur5eMathiassenVisualServoingClass::calculateCovI()
{
  this->covI(0, 0) = (this->img_moments.m20 / this->img_moments.m00)
    - pow(this->pcenters(0, this->pcent_counter), 2);
  this->covI(1, 0) = (this->img_moments.m11 / this->img_moments.m00)
    - (this->pcenters(0, this->pcent_counter)
       * this->pcenters(1, this->pcent_counter));
  this->covI(0, 1) = this->covI(1, 0);
  this->covI(1, 1) = (this->img_moments.m02 / this->img_moments.m00)
    - pow(this->pcenters(1, this->pcent_counter), 2);
}

void Ur5eMathiassenVisualServoingClass::calculateMoments()
{
  this->img_moments = cv::moments(this->segmented_img);
}

void Ur5eMathiassenVisualServoingClass::calculatePCenter()
{
  this->pcent_counter = (this->pcent_counter + 1) % this->pcenters.cols();
  this->pcenters(0, this->pcent_counter) =
    this->img_moments.m10 / this->img_moments.m00;
  this->pcenters(1, this->pcent_counter) =
    this->img_moments.m01 / this->img_moments.m00;
}

void Ur5eMathiassenVisualServoingClass::calculatePTip()
{
  double eig_x;
  double eig_y;
  MatrixXd ptip_1 = MatrixXd::Zero(2, 1);
  MatrixXd ptip_2 = MatrixXd::Zero(2, 1);

  eig_x = (2 * sqrt(this->lambdas(1)) * this->eigVectors(0, 1));
  eig_y = (2 * sqrt(this->lambdas(1)) * this->eigVectors(1, 1));

  ptip_1(0, 0) = this->pcenters(0, this->pcent_counter) + eig_x;
  ptip_1(1, 0) = this->pcenters(1, this->pcent_counter) + eig_y;

  ptip_2(0, 0) = this->pcenters(0, this->pcent_counter) - eig_x;
  ptip_2(1, 0) = this->pcenters(1, this->pcent_counter) - eig_y;

  // Determine ptip to be at the larger z coordinate
  if(ptip_2(1, 0) > ptip_1(1, 0))
  {
    this->ptip = ptip_2;
  }
  else
  {
    this->ptip = ptip_1;
  }

  ROS_DEBUG_STREAM("PTip = [" << this->ptip(0) << ", " << this->ptip(1) << "]");
}

void Ur5eMathiassenVisualServoingClass::calculateAlpha()
{
  this->alpha_prev = this->alpha;
  this->alpha = asin(sqrt(this->lambdas(0) / this->lambdas(1)));
  this->bayesianInference(this->alpha, this->alpha_prev, this->prob_dalpha);
}

void Ur5eMathiassenVisualServoingClass::calculateSI()
{
  Mat nonZero;
  Mat x;
  Mat y;
  Mat r;
  Mat theta;
  Mat x_trans;
  Mat y_trans;
  Mat x2;
  Mat y2;
  Mat r2;
  double r0;
  double x0;
  double y0;
  double dr;
  double dy;
  int si;

  if(this->img_moments.m00 < 1e-6)
  {
    ROS_WARN_STREAM("calculateSI: empty image");
    this->SI = 0;
  }
  else
  {
    // Returns a Point mat
    cv::findNonZero(this->segmented_img, nonZero);
    x = Mat::zeros(nonZero.total(), 1, CV_32F);
    y = Mat::zeros(nonZero.total(), 1, CV_32F);
    r = Mat::zeros(nonZero.total(), 1, CV_32F);
    theta = Mat::zeros(nonZero.total(), 1, CV_32F);

    for(int i = 0; i < nonZero.total(); i++)
    {
      x.at<float>(i) = nonZero.at<cv::Point>(i).x;
      y.at<float>(i) = nonZero.at<cv::Point>(i).y;
    }

    r0 = 232.0;
    x0 = 164.05;
    y0 = -25.46;
    x_trans = x - x0;
    y_trans = y - y0;

    // These are matrices, not scalars.
    pow(x_trans, 2, x2);
    pow(y_trans, 2, y2);
    r2 = x2 + y2;
    cv::sqrt(r2, r);
    cv::phase(y_trans, x_trans, theta);  // phase is atan2 [0, 2pi]
    cv::subtract(
      theta, 2 * M_PI, theta, (theta > M_PI));  // convert to [-pi, pi];
    si = 0;
    dr = 2;
    dy = 2;

    if(cv::sum(y <= dy)[0] > 0)
    {
      si = si + 1;
    }
    else
    {
      // No operation
    }

    if(cv::sum(theta <= (-M_PI / 4 + M_PI / 100))[0] > 0)
    {
      si = si + 1;
    }
    else
    {
      // No operation
    }

    if(cv::sum(theta >= (M_PI / 4 - M_PI / 100))[0] > 0)
    {
      si = si + 1;
    }
    else
    {
      // No operation
    }

    if(cv::sum(r >= (r0 - dr))[0] > 0)
    {
      si = si + 1;
    }
    else
    {
      // No operation
    }

    this->SI = si;
  }
}

void Ur5eMathiassenVisualServoingClass::calculateInitialAlphaTheta()
{
  double delX;
  double delY;
  double delZ;
  double alpha1;
  double theta1;
  int next_counter;

  next_counter = (this->pcent_counter + 1) % this->pcenters.cols();

  delX =
    (this->pcenters(0, this->pcent_counter) - this->pcenters(0, next_counter))
    * this->pixel_x;
  delY =
    (this->pcenters(1, this->pcent_counter) - this->pcenters(1, next_counter))
    * this->pixel_z;
  delZ = this->y_neg_positions(0, this->pcent_counter)
    - this->y_neg_positions(0, next_counter);

  alpha1 = atan2(delZ, (sqrt(pow(delX, 2) + pow(delY, 2))));
  theta1 = atan2(delY, delZ);
  alpha1 = abs(alpha1) * (-this->sign(theta1 * delY));
  this->alpha_prev = this->alpha;
  this->alpha = alpha1;
  this->theta = theta1;
}

void Ur5eMathiassenVisualServoingClass::calculateTheta()
{
  this->calculateCovI();
  this->theta =
    0.5 * atan((2 * this->covI(0, 1)) / (this->covI(0, 0) - this->covI(1, 1)));
}

void Ur5eMathiassenVisualServoingClass::calculateL()
{
  double w;
  w = this->img_moments.m00 / (4 * this->lambdas(1));
  this->L_prev = this->L;
  this->L = sqrt(pow(this->needle_rad, 2) - (pow(w, 2) / 4));
  this->bayesianInference(this->L, this->L_prev, this->prob_dL);
}

void Ur5eMathiassenVisualServoingClass::bayesianInference(
  double& curr, double& prev, double& prob_d_edot)
{
  double k;
  double T;
  double error_dot;
  double prob_low;
  double prob_high;
  double prob_edot_d;
  double prob_d_edot_next;
  double prob_d;
  double N;
  double dir;  // current sign

  // TODO Figure out what k is supposed to be
  k = 1;
  dir = this->sign(prev);
  T = (this->Tcurr - this->Tprev).toSec();
  error_dot = (curr - prev) / T;
  prob_low = 0.5 * exp(-k * abs(error_dot));
  prob_high = 1 - prob_low;
  prob_d = prob_d_edot;
  N = 1e-2;

  // Look at P(d|edot) = P(1|edot) if dprev = -1
  // Look at P(d|edot) = P(-1|edot) if dprev = 1

  if(error_dot < 0)
  {
    prob_edot_d = prob_low;
  }
  else
  {
    prob_edot_d = prob_high;
  }

  prob_d_edot_next = ((prob_edot_d * prob_d) + N)
    / ((prob_edot_d * prob_d) + ((1 - prob_edot_d) * (1 - prob_d)) + N);

  if(prob_d_edot_next > 0.6)
  {
    // Flip the direction if the probability of the other direction is higher
    dir = -1 * dir;
    prob_d_edot = 1 - prob_d_edot_next;
  }
  else
  {
    prob_d_edot = prob_d_edot_next;
  }

  curr = dir * curr;
}

/*
// Calculate alpha and theta depending on the state
void Ur5eMathiassenVisualServoingClass::calculateAlphaThetaL()
{
  switch(this->fsm_state)
  {
    case 2:
      this->calculateInitialAlphaTheta();
      break;
    case 3:
      this->calculateInitialAlphaTheta();
      break;
    case 4:
      this->calculateTheta();
      this->calculateAlpha();
      break;
    case 5:
      this->calculateTheta();
      break;
    default:
      break;
  }
  if(this->fsm_state != 1)
  {
    this->extendedKalmanFilter();
  }
}*/

void Ur5eMathiassenVisualServoingClass::extendedKalmanFilter()
{
  int alpha_sign;
  double T;
  MatrixXd sk(4, 1);
  MatrixXd sk_next(4, 1);
  MatrixXd Lmat(4, 6);
  T = (this->Tcurr - this->Tprev).toSec();
  Lmat = MatrixXd::Zero(4, 6);

  switch(this->fsm_state)
  {
    case 2:
      sk(0, 0) = this->alpha;
      sk(1, 0) = this->theta;
      sk(2, 0) = this->pcenters(0, this->pcent_counter);
      sk(3, 0) = this->pcenters(1, this->pcent_counter);
      alpha_sign = this->sign(this->alpha);

      Lmat(2, 1) = -alpha_sign * cos(this->theta);
      Lmat(3, 1) = -alpha_sign * sin(this->theta);

      // Get updated estimates
      sk_next = sk + T * (Lmat * this->velocity);
      this->alpha = sk_next(0, 0);
      this->theta = sk_next(1, 0);
      this->pcenters(0, this->pcent_counter) = sk_next(2, 0);
      this->pcenters(1, this->pcent_counter) = sk_next(3, 0);

      break;
    case 3:
      sk(0, 0) = this->alpha;
      sk(1, 0) = this->theta;
      sk(2, 0) = this->pcenters(0, this->pcent_counter);
      sk(3, 0) = this->pcenters(1, this->pcent_counter);

      Lmat << this->Lalpha(), this->Ltheta(), this->Lpcent();
      sk_next = sk + T * (Lmat * this->velocity);
      this->alpha = sk_next(0, 0);
      this->theta = sk_next(1, 0);
      this->pcenters(0, this->pcent_counter) = sk_next(2, 0);
      this->pcenters(1, this->pcent_counter) = sk_next(3, 0);

      break;
    case 4:
      if(this->SI != 0)
      {
        sk(0, 0) = this->alpha;
        sk(1, 0) = this->theta;
        sk(2, 0) = this->pcenters(0, this->pcent_counter);
        sk(3, 0) = this->pcenters(1, this->pcent_counter);
        Lmat << this->Lalpha(), this->Ltheta(), this->Lpcent();
        sk_next = sk + T * (Lmat * this->velocity);
        this->alpha = sk_next(0, 0);
        this->theta = sk_next(1, 0);
        this->pcenters(0, this->pcent_counter) = sk_next(2, 0);
        this->pcenters(1, this->pcent_counter) = sk_next(3, 0);

        break;
      }
      else
      {
        // No operation
      }

      break;
    default:
      ROS_DEBUG_STREAM("EKF not executed in state " << this->fsm_state);
  }
}

void Ur5eMathiassenVisualServoingClass::imageSegmentation()
{
  ROS_INFO_STREAM("Pre-processing image...");
  this->imgPreProcessing();
  // cv::imshow("Original Image", this->us_img->image);
  // cv::waitKey(5);

  // Needle Filter
  this->needleEnhancingFilter();
  this->rescaleImgRange(this->proc_img_needle_f);
  this->threshold(
    this->proc_img_needle_f, this->needle_filter_threshold,
    this->proc_img_needle_u);
  this->morphClosing();  // Apply morphological closing to connect the segments

  // Moment Filter
  this->momentFilter();
  this->rescaleImgRange(this->proc_img_moment_f);
  this->threshold(
    this->proc_img_moment_f, this->moment_filter_threshold,
    this->proc_img_moment_u);

  // Combine the 2 filtered images using the OR operation
  this->combineFilteredImgs();

  if(this->save_img)
  {
    this->ss.str("");  // Clear stringstream
    this->ss << this->save_path << "/img_" << std::setw(6) << std::setfill('0')
             << this->img_ct << ".png";

    if(!cv::imwrite(this->ss.str(), this->segmented_img))
    {
      ROS_WARN_STREAM("Error writing image to: " << this->ss.str());
    }
    else
    {
      // No operation
    }
    this->img_ct++;
  }
  else
  {
    // No operation
  }
}

void Ur5eMathiassenVisualServoingClass::imgPreProcessing()
{
  Mat img;
  Mat color_img;
  Mat grey_img;
  Mat squared;
  double mean_squared_power;

  img = this->us_img->image;
  // img.convertTo(color_img, CV_32F);
  cv::cvtColor(img, grey_img, cv::COLOR_BGR2GRAY);
  this->preprocessed_img = grey_img;

  pow(this->preprocessed_img, 2, squared);
  mean_squared_power = sqrt(cv::sum(squared)[0])
    / (this->preprocessed_img.rows * this->preprocessed_img.cols);
  if(mean_squared_power > 0.15)
  {
    // Discard image if mean power is too high
    this->preprocessed_img = Mat::zeros(
      this->preprocessed_img.rows, this->preprocessed_img.cols, CV_8U);
  }
}

void Ur5eMathiassenVisualServoingClass::rescaleImgRange(Mat& img)
{
  double min;
  double max;
  Mat rescaled_img;

  rescaled_img = Mat::zeros(img.rows, img.cols, CV_32F);
  cv::minMaxLoc(img, &min, &max);
  rescaled_img = 255 * img / max;
  img = rescaled_img;
}

void Ur5eMathiassenVisualServoingClass::needleEnhancingFilter()
{
  Mat img;
  Mat in_circle;
  Mat border_circle;
  double circ_center;
  double circ_radius;
  double dist;
  double norm;
  int ddepth;
  int delta;

  img = this->preprocessed_img;
  circ_radius =
    this->needle_kernel_size * (19.0 / 29) / 2;  // scale by original parameters
  circ_center = (this->needle_kernel_size + 1) / 2.0;
  // Create the kernel
  in_circle =
    Mat::zeros(this->needle_kernel_size, this->needle_kernel_size, CV_32F);
  border_circle =
    Mat::zeros(this->needle_kernel_size, this->needle_kernel_size, CV_32F);

  for(int i = 0; i < this->needle_kernel_size; i++)
  {
    for(int j = 0; j < this->needle_kernel_size; j++)
    {
      dist = pow((i - circ_center), 2) + pow((j - circ_center), 2);
      if(dist < pow(circ_radius, 2))
      {
        in_circle.at<float>(i, j) = 1;
      }
      else if(dist - pow(circ_radius, 2) <= 20)
      {
        border_circle.at<float>(i, j) = 1;
      }
    }
  }

  norm = 2.25 / cv::sum(in_circle)[0];
  in_circle = in_circle * norm;
  norm = 2.25 / cv::sum(border_circle)[0];
  border_circle = border_circle * norm;

  Mat kernel = in_circle - border_circle;

  // Filter the image
  ddepth = -1;  // indicates depth of result is the same as source
  cv::Point anchor = cv::Point(-1, -1);  // anchor at the center of the kernel
  delta = 0;  // value to be added to each pixel during correlation
  this->proc_img_needle_f.setTo(cv::Scalar(0));
  cv::filter2D(
    img, this->proc_img_needle_f, ddepth, kernel, anchor, delta,
    cv::BORDER_DEFAULT);
}

void Ur5eMathiassenVisualServoingClass::threshold(
  Mat img_in, double thresh, Mat& img_out)
{
  Mat mask;

  img_out.setTo(cv::Scalar(0));
  mask = (img_in >= thresh);
  img_out.setTo(255, mask);
}

void Ur5eMathiassenVisualServoingClass::morphClosing()
{
  Mat se;
  Mat img_close;

  se = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(9, 9));
  img_close = this->proc_img_needle_u;

  // TODO: See how this differs from using a larger kernel a single time.
  for(int i = 0; i < 4; i++)
  {
    cv::morphologyEx(img_close, img_close, cv::MORPH_CLOSE, se);
  }

  this->proc_img_needle_u = img_close;
}

void Ur5eMathiassenVisualServoingClass::momentFilter()
{
  Mat img;
  Mat prod_img;
  Mat i_mat;
  Mat imatrix;
  Mat j_mat;
  Mat jmatrix;
  Mat kernel;
  int s;
  int ddepth;
  int delta;
  cv::Point anchor;

  this->preprocessed_img.convertTo(img, CV_32F);
  prod_img = Mat::zeros(img.rows, img.cols, CV_32F);
  i_mat = Mat::zeros(img.rows, 1, CV_32F);

  for(int i = 0; i < img.rows; i++)
  {
    i_mat.at<float>(i, 0) = i;
  }

  imatrix = cv::repeat(i_mat, 1, img.cols);

  j_mat = Mat::zeros(1, img.cols, CV_32F);

  for(int j = 0; j < img.cols; j++)
  {
    j_mat.at<float>(0, j) = j;
  }

  jmatrix = cv::repeat(j_mat, img.rows, 1);
  prod_img = img.mul(imatrix);
  prod_img = prod_img.mul(jmatrix);

  s = this->moment_kernel_size;
  kernel =
    Mat::ones((2 * s) + 1, (2 * s) + 1, CV_32F) / (4 * (pow((2 * s) + 1, 2)));
  // Filter the image
  ddepth = -1;
  anchor = cv::Point(-1, -1);
  delta = 0;

  this->proc_img_moment_f = Mat::zeros(img.rows, img.cols, CV_32F);

  cv::filter2D(
    prod_img, this->proc_img_moment_f, ddepth, kernel, anchor, delta,
    cv::BORDER_DEFAULT);
}

void Ur5eMathiassenVisualServoingClass::combineFilteredImgs()
{
  // Initialize variables.
  double std_thresh = 10.0;
  double mean_thresh = 2.0;
  int num_comp;
  int i0;
  int j0;
  int x0;
  int y0;

  Mat combined_img;
  Mat label_img;
  Mat stats;
  Mat centroids;
  Mat mask;
  Mat disp_img;
  Mat target_img;
  Mat sorted_ind;
  Mat bg_mean;
  Mat bg_std;
  Mat bg_mask;
  Mat background_img;

  // Check dimensions of both input images.
  if(
    (this->proc_img_needle_u.rows == this->proc_img_moment_u.rows)
    && (this->proc_img_needle_u.cols == this->proc_img_moment_u.cols))
  {
    cv::bitwise_or(
      (this->proc_img_needle_u == 255), (this->proc_img_moment_u == 255), mask);

    // Use connected components to determine validity of the image
    // TODO: See if combined_img can be replaced with multiplying mask by 255.
    combined_img = Mat::zeros(
      this->proc_img_needle_u.rows, this->proc_img_needle_u.cols, CV_8U);
    combined_img.setTo(cv::Scalar(255), mask);

    // TODO: See if mask (not multiplied by 255) can be supplied to this function.
    num_comp = cv::connectedComponentsWithStats(
      combined_img, label_img, stats, centroids);

    if((num_comp > 1) && (num_comp <= 9))
    {
      // Mat mask(label_img.size(), CV_8UC1, cv::Scalar(0));
      // Segment largest region as the target mask
      sorted_ind = Mat::zeros(num_comp, 1, CV_16U);
      cv::sortIdx(
        stats.col(4), sorted_ind, CV_SORT_EVERY_COLUMN + CV_SORT_DESCENDING);
      disp_img = Mat::zeros(
        4 * this->proc_img_needle_u.rows, 4 * this->proc_img_needle_u.cols,
        CV_8U);
      x0 = 0;
      y0 = 0;
      j0 = 0;
      this->preprocessed_img.copyTo(disp_img(cv::Rect(
        x0, y0, this->preprocessed_img.cols, this->preprocessed_img.rows)));
      x0 += this->preprocessed_img.cols;
      ++j0;

      this->segmented_img.copyTo(disp_img(
        cv::Rect(x0, y0, this->segmented_img.cols, this->segmented_img.rows)));
      x0 += this->segmented_img.cols;
      ++j0;

      this->proc_img_needle_u.copyTo(disp_img(cv::Rect(
        x0, y0, this->proc_img_needle_u.cols, this->proc_img_needle_u.rows)));
      x0 += this->proc_img_needle_u.cols;
      ++j0;

      this->proc_img_moment_u.copyTo(disp_img(cv::Rect(
        x0, y0, this->proc_img_moment_u.cols, this->proc_img_moment_u.rows)));
      ++j0;

      mask.setTo(cv::Scalar(0));
      mask = (label_img == sorted_ind.at<ushort>(1, 0));
      this->segmented_img.setTo(cv::Scalar(0));
      combined_img.copyTo(this->segmented_img, mask);

      // Segment the target image and background image
      target_img = Mat::zeros(combined_img.rows, combined_img.cols, CV_8UC1);
      this->preprocessed_img.copyTo(target_img, mask);
      target_img.copyTo(disp_img(cv::Rect(
        (j0 % 4) * target_img.cols, (j0 / 4) * target_img.rows, target_img.cols,
        target_img.rows)));
      ++j0;

      bg_mask = Mat::zeros(mask.rows, mask.cols, CV_8UC1);
      cv::bitwise_not(mask, bg_mask);
      background_img =
        Mat::zeros(combined_img.rows, combined_img.cols, CV_8UC1);
      this->preprocessed_img.copyTo(background_img, bg_mask);
      background_img.copyTo(disp_img(cv::Rect(
        (j0 % 4) * background_img.cols, (j0 / 4) * background_img.rows,
        background_img.cols, background_img.rows)));
      ++j0;

      for(i0 = 0; i0 < num_comp; ++i0)
      {
        mask.setTo(cv::Scalar(0));
        mask = (label_img == sorted_ind.at<ushort>(i0, 0));
        x0 = (j0 % 4) * mask.cols;
        y0 = (j0 / 4) * mask.rows;
        mask.copyTo(disp_img(cv::Rect(x0, y0, mask.cols, mask.rows)));
        ++j0;
      }

      cv::imshow("Components", disp_img);
      cv::waitKey(5);

      // Compute the mean of the target and background
      cv::meanStdDev(background_img, bg_mean, bg_std);
      ROS_WARN_STREAM("background mean = " << bg_mean.at<double>(0, 0));
      ROS_WARN_STREAM("background std = " << bg_std.at<double>(0, 0));

      if(
        (bg_mean.at<double>(0, 0) > mean_thresh)
        && (bg_std.at<double>(0, 0) > std_thresh))
      {
        // Discard the image if the background has high mean and high variance
        this->segmented_img.setTo(cv::Scalar(0));
      }
      else
      {
        // No operation
      }
    }
    else
    {
      ROS_WARN_STREAM(
        "Poor image: " << num_comp << " segmented regions outside range [1, 9]");
      this->segmented_img.setTo(cv::Scalar(0));
    }
  }
  else
  {
    ROS_ERROR_STREAM(
      "Dimension mismatch: Needle ("
      << this->proc_img_needle_u.rows << " x " << this->proc_img_needle_u.cols
      << ") != Moment (" << this->proc_img_moment_u.rows << " x "
      << this->proc_img_moment_u.cols << ")");
    this->segmented_img.setTo(cv::Scalar(0));
  }
}

void Ur5eMathiassenVisualServoingClass::unknownLocationVelocity()
{
  double prev_y_vel = this->velocity(1);
  this->velocity = MatrixXd::Zero(6, 1);

  if(this->unknown_loc_counter % 100 < 50)
  {
    this->velocity(1) = this->unknown_loc_y_vel;
  }
  else
  {
    this->velocity(1) = -this->unknown_loc_y_vel;
  }

  ROS_INFO_STREAM(
    "FSM [" << this->fsm_state << "] (unknown location):" << std::endl
            << "v = [" << (format("%.3e") % this->velocity(0)) << ", "
            << (format("%.3e") % this->velocity(1)) << ", "
            << (format("%.3e") % this->velocity(2)) << ", "
            << (format("%.3e") % this->velocity(3)) << ", "
            << (format("%.3e") % this->velocity(4)) << ", "
            << (format("%.3e") % this->velocity(5)) << "]");

  this->unknown_loc_counter++;
}

void Ur5eMathiassenVisualServoingClass::estimateDirectionVelocity()
{
  MatrixXd Lz;
  MatrixXd s;
  MatrixXd s_star;
  int sign_alpha;
  double K;

  Lz = MatrixXd::Zero(2, 6);
  sign_alpha = this->sign(this->alpha);
  Lz(0, 1) = -sign_alpha * cos(this->theta);
  Lz(1, 1) = -sign_alpha * sin(this->theta);
  K = 0.5 / 2.0;

  s = MatrixXd::Zero(2, 1);
  s(0, 0) = this->pcenters(0, this->pcent_counter) * this->pixel_x;
  s(1, 0) = this->pcenters(1, this->pcent_counter) * this->pixel_z;

  s_star = MatrixXd::Zero(2, 1);
  s_star(0, 0) = this->pcent_star(0, 0) * this->pixel_x;
  s_star(1, 0) = this->pcent_star(1, 0) * this->pixel_z;

  MatrixXd Lz_inv = Lz.completeOrthogonalDecomposition().pseudoInverse();
  this->velocity = (-K) * Lz_inv * (s - s_star);

  ROS_INFO_STREAM(
    "FSM [" << this->fsm_state << "] (estimate direction):" << std::endl
            << "v = [" << (format("%.3e") % this->velocity(0)) << ", "
            << (format("%.3e") % this->velocity(1)) << ", "
            << (format("%.3e") % this->velocity(2)) << ", "
            << (format("%.3e") % this->velocity(3)) << ", "
            << (format("%.3e") % this->velocity(4)) << ", "
            << (format("%.3e") % this->velocity(5)) << "]");

  this->unknown_loc_counter = 0;
}

void Ur5eMathiassenVisualServoingClass::unalignedVelocity()
{
  MatrixXd L(3, 6);
  MatrixXd K;
  MatrixXd s;
  MatrixXd s_star;
  MatrixXd L_inv;

  L << this->Lalpha(), this->Lpcent();
  K = MatrixXd::Zero(3, 3);
  K(0, 0) = 0.2 / 2.0;
  K(1, 1) = 1 / 2.0;
  K(2, 2) = 1 / 2.0;

  s = MatrixXd::Zero(3, 1);
  s(0, 0) = this->alpha;
  s(1, 0) = this->pcenters(0, this->pcent_counter) * this->pixel_x;
  s(2, 0) = this->pcenters(1, this->pcent_counter) * this->pixel_z;

  s_star = MatrixXd::Zero(3, 1);
  s_star(0, 0) = this->alpha_star;
  s_star(1, 0) = this->pcent_star(0, 0) * this->pixel_x;
  s_star(2, 0) = this->pcent_star(1, 0) * this->pixel_z;

  L_inv = L.completeOrthogonalDecomposition().pseudoInverse();
  this->velocity = L_inv * (-K) * (s - s_star);

  ROS_INFO_STREAM(
    "FSM [" << this->fsm_state << "] (unaligned):" << std::endl
            << "v = [" << (format("%.3e") % this->velocity(0)) << ", "
            << (format("%.3e") % this->velocity(1)) << ", "
            << (format("%.3e") % this->velocity(2)) << ", "
            << (format("%.3e") % this->velocity(3)) << ", "
            << (format("%.3e") % this->velocity(4)) << ", "
            << (format("%.3e") % this->velocity(5)) << "]");

  this->unknown_loc_counter = 0;
}

void Ur5eMathiassenVisualServoingClass::nearAlignedVelocity()
{
  MatrixXd L(4, 6);
  MatrixXd K;
  MatrixXd s;
  MatrixXd s_star;
  MatrixXd L_inv;

  L << this->Lalpha(), this->Ltheta(), this->Lpcent();

  K = MatrixXd::Zero(4, 4);
  K(0, 0) = 0.2 / 2.0;
  K(1, 1) = 0.2 / 2.0;
  K(2, 2) = 0.4 / 2.0;
  K(3, 3) = 0.4 / 2.0;

  s = MatrixXd::Zero(4, 1);
  s(0, 0) = this->alpha;
  s(1, 0) = this->theta;
  s(2, 0) = this->pcenters(0, this->pcent_counter) * this->pixel_x;
  s(3, 0) = this->pcenters(1, this->pcent_counter) * this->pixel_z;

  s_star = MatrixXd::Zero(4, 1);
  s_star(0, 0) = this->alpha_star;
  s_star(1, 0) = this->theta_star;
  s_star(2, 0) = this->pcent_star(0, 0) * this->pixel_x;
  s_star(3, 0) = this->pcent_star(1, 0) * this->pixel_z;

  L_inv = L.completeOrthogonalDecomposition().pseudoInverse();
  this->velocity = L_inv * (-K) * (s - s_star);

  ROS_INFO_STREAM(
    "FSM [" << this->fsm_state << "] (near aligned):" << std::endl
            << "v = [" << (format("%.3e") % this->velocity(0)) << ", "
            << (format("%.3e") % this->velocity(1)) << ", "
            << (format("%.3e") % this->velocity(2)) << ", "
            << (format("%.3e") % this->velocity(3)) << ", "
            << (format("%.3e") % this->velocity(4)) << ", "
            << (format("%.3e") % this->velocity(5)) << "]");

  this->unknown_loc_counter = 0;
}

void Ur5eMathiassenVisualServoingClass::alignedVelocity()
{
  MatrixXd L(4, 6);
  MatrixXd s;
  MatrixXd s_star;
  MatrixXd L_inv;
  double K = 0.3 / 2.0;

  L << this->Ll(), this->Ltheta(), this->Lpcent();  // pcent approx ptip

  s = MatrixXd::Zero(4, 1);
  s(0, 0) = this->l;
  s(1, 0) = this->theta;
  s(2, 0) = this->ptip(0, 0) * this->pixel_x;
  s(3, 0) = this->ptip(1, 0) * this->pixel_z;

  s_star = MatrixXd::Zero(4, 1);
  s_star(0, 0) = this->l_star;
  s_star(1, 0) = this->theta_star;
  s_star(2, 0) = this->ptip_star(0, 0) * this->pixel_x;
  s_star(3, 0) = this->ptip_star(1, 0) * this->pixel_z;

  L_inv = L.completeOrthogonalDecomposition().pseudoInverse();
  this->velocity = L_inv * (-K) * (s - s_star);

  ROS_INFO_STREAM(
    "FSM [" << this->fsm_state << "] (aligned):" << std::endl
            << "v = [" << (format("%.3e") % this->velocity(0)) << ", "
            << (format("%.3e") % this->velocity(1)) << ", "
            << (format("%.3e") % this->velocity(2)) << ", "
            << (format("%.3e") % this->velocity(3)) << ", "
            << (format("%.3e") % this->velocity(4)) << ", "
            << (format("%.3e") % this->velocity(5)) << "]");

  this->unknown_loc_counter = 0;
}

int Ur5eMathiassenVisualServoingClass::sign(double val)
{
  if(val >= 0)
  {
    return 1;
  }
  else
  {
    return -1;
  }
}

MatrixXd Ur5eMathiassenVisualServoingClass::Lalpha()
{
  MatrixXd L = MatrixXd::Zero(1, 6);
  L(0, 1) =
    this->pcenters(0, this->pcent_counter) * this->pixel_x * cos(this->theta);
  L(0, 5) = -cos(this->theta);

  return L;
}

MatrixXd Ur5eMathiassenVisualServoingClass::Lpcent()
{
  MatrixXd L = MatrixXd::Zero(2, 6);

  L(0, 0) = -1;
  L(1, 2) = -1;
  L(0, 4) = -this->pcenters(1, this->pcent_counter) * this->pixel_z;
  L(1, 4) = this->pcenters(0, this->pcent_counter) * this->pixel_x;

  return L;
}

MatrixXd Ur5eMathiassenVisualServoingClass::Ltheta()
{
  MatrixXd L = MatrixXd::Zero(1, 6);

  L(0, 5) = -tan(this->alpha) * sin(this->theta);
  L(0, 4) = 1;

  return L;
}

MatrixXd Ur5eMathiassenVisualServoingClass::Ll()
{
  MatrixXd L = MatrixXd::Zero(1, 6);

  L(0, 1) = this->sign(this->L);

  return L;
}

void Ur5eMathiassenVisualServoingClass::planPath()
{
  Eigen::MatrixXd rotation(6, 6);

  ROS_INFO_STREAM("Transforming velocity to spatial frame...");
  const Eigen::Isometry3d& fwd_kin = this->getGlobalLinkTransform();
  rotation << fwd_kin.rotation(), Eigen::MatrixXd::Zero(3, 3),
    Eigen::MatrixXd::Zero(3, 3), fwd_kin.rotation();

  ROS_INFO_STREAM("Setting cartesian velocity...");
  this->setSpatialCartesianVelocity(rotation * this->velocity);
  ROS_INFO_STREAM("Sending goal...");
  this->sendGoalAndWait();
  ROS_INFO_STREAM("Trajectory executed.");
}

void Ur5eMathiassenVisualServoingClass::moveToInitialPosition()
{
  std::vector<double> q;

  ROS_INFO_STREAM("Setting initial pose...");
  q.push_back(0.0);
  q.push_back(-M_PI / 2.0);
  q.push_back(0.0);
  q.push_back(-M_PI / 2.0);
  q.push_back(0.0);
  q.push_back(0.0);
  this->setJointPosition(q);
  this->sendGoalAndWait();
  ROS_INFO_STREAM("Initial pose executed.");
}

void Ur5eMathiassenVisualServoingClass::publishVelocity()
{
  std::vector<double> velocity;
  std_msgs::Float64MultiArray velocity_msg;

  ROS_INFO_STREAM("Publishing velocity");
  for(int i = 0; i < this->velocity.rows(); i++)
  {
    velocity.push_back(this->velocity(i, 0));
  }
  velocity_msg.data = velocity;
  // this->velocity_pub.publish(velocity_msg);
  ROS_INFO_STREAM("Finished publishing velocity.");
}

std::string matType2Str(int type)
{
  std::string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch(depth)
  {
    case CV_8U:
      r = "8U";
      break;
    case CV_8S:
      r = "8S";
      break;
    case CV_16U:
      r = "16U";
      break;
    case CV_16S:
      r = "16S";
      break;
    case CV_32S:
      r = "32S";
      break;
    case CV_32F:
      r = "32F";
      break;
    case CV_64F:
      r = "64F";
      break;
    default:
      r = "User";
      break;
  }

  r += "C";
  r += (chans + '0');

  return r;
}
