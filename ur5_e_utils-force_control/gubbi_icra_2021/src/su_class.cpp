#include "gubbi_icra_2021/su_class.h"

Ur5eSuVisualServoingClass::Ur5eSuVisualServoingClass(
  ros::NodeHandle& nh, std::string action_server_name, std::string ee_name,
  double delta_t, double joint_omega_max, int needle_filter_threshold,
  int moment_filter_threshold, int needle_kernel_size, int moment_kernel_size,
  bool save_img, std::string save_path, bool debug_segmentation,
  double unknown_loc_vy, double find_tip_vx, double max_aligning_wz)
  : Ur5eBaseClass(nh, action_server_name, ee_name, delta_t, joint_omega_max)
  , moment_filter_threshold(moment_filter_threshold)
  , needle_filter_threshold(needle_filter_threshold)
  , moment_kernel_size(moment_kernel_size)
  , needle_kernel_size(needle_kernel_size)
  , save_img(save_img)
  , debug_segmentation(debug_segmentation)
  , unknown_loc_vy(unknown_loc_vy)
  , find_tip_vx(find_tip_vx)
  , max_aligning_wz(max_aligning_wz)
  , save_path(save_path)
{
  this->sim = (action_server_name == "/arm_controller/follow_joint_trajectory");

  ROS_DEBUG_STREAM("moment=" << this->moment_filter_threshold);
  ROS_DEBUG_STREAM("needle=" << this->needle_filter_threshold);
  ROS_DEBUG_STREAM("moment_kernel=" << this->moment_kernel_size);
  ROS_DEBUG_STREAM("needle_kernel=" << this->needle_kernel_size);
  ROS_DEBUG_STREAM("save_img=" << this->save_img);

  // Variable initializations
  this->pcenter = VectorXd::Zero(2);  // Set pcenters to all 0s
  this->pixel_x = 0.000385;           // [m/px]
  this->pixel_z = 0.000385;           // [m/px]
  this->pcent_star(0) = 247 / 2.0;    // Center of the image
  this->pcent_star(1) = 156 / 2.0;
  this->ptip_star(0) = 247 / 2.0;  // Center of the image
  this->ptip_star(1) = 156 / 2.0;
  this->aligned_ct_max = 15;  // TODO: remove
  this->eps_pcent = 1;
  this->eps_si = 0.4;
  this->A0 = 5;
  this->si_star = 1;
  this->SIs = VectorXd::Zero(this->buf_length);
  this->delAs = VectorXi::Zero(this->buf_length);
  this->tg_areas = VectorXd::Zero(this->buf_length);
  this->buf_filled = false;

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
    "/verasonics/us_img", 1000, &Ur5eSuVisualServoingClass::usImgAcqCallback,
    this);

  this->disp_img_pub = this->nh.advertise<sensor_msgs::Image>("/disp_img", 10);

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

Ur5eSuVisualServoingClass::~Ur5eSuVisualServoingClass()
{
}

void Ur5eSuVisualServoingClass::usImgAcqCallback(
  const sensor_msgs::ImageConstPtr& img)
{
  // Convert ROS image to CV image and save
  try
  {
    this->us_img = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  this->imageSegmentation();
  this->calculateMoments();
  this->calculateSI();
  // Fill up buffer with segmented imgs before executing fsm
  if(this->buf_filled)
  {
    this->calculatePCenter();
    this->calculateSVD();
    this->calculatePTip();
    this->saveDebugData();
    this->fsm();
  }
  else
  {
    // No operation
  }
  if((this->buf_counter == this->buf_length - 1) && !this->buf_filled)
  {
    // Set flag to indicate that buffer has been filled
    this->buf_filled = true;
  }
  else
  {
    // No operation
  }
  this->buf_counter = (this->buf_counter + 1) % this->buf_length;
}

void Ur5eSuVisualServoingClass::fsm()
{
  ROS_INFO_STREAM("Running finite state machine...");
  VectorXi bool_A;

  if(this->img_moments.m00 < 1e-6)
  {
    // If M00 = 0, location is unknown (State 1)
    this->fsm_state = 1;
    this->unknown_loc_counter++;
    this->align_counter = 0;
  }
  else
  {
    switch(this->fsm_state)
    {
      case 1:
        this->align_counter = 0;
        if(abs(this->SIs.mean() - 2) <= this->eps_si)
        {
          // Transition to state 4 if the needle shaft touches 2 boundaries
          this->fsm_state = 4;
        }
        else
        {  // Transition to State 2 (Centering)
          this->fsm_state = 2;
        }
        break;
      case 2:
        this->unknown_loc_counter = 0;
        this->align_counter = 0;
        if(abs(this->SIs.mean() - 2) <= this->eps_si)
        {
          // Transition to state 4 if the needle shaft touches 2 boundaries
          this->fsm_state = 4;
        }
        else
        {
          // No operation
        }
        if(abs(this->pcent_xdiff) <= this->eps_pcent)
        {
          // Transition to State 3 (Aligning) if pcenter is close to the desired position
          this->fsm_state = 3;
        }
        else
        {
          // No operation
        }
        break;
      case 3:
        if(this->SIs(this->buf_counter) == 2)
        {
          // Transition to state 4 if shaft intersects 2 boundaries
          this->fsm_state = 4;
        }
        else if((this->align_counter > 0) && (abs(this->aligning_wz) < (M_PI / 180)))
        {
          // Continue to align if shaft has not intersected 2 boundaries
          this->aligning_wz = 0.25 * this->max_aligning_wz;
        }
        else
        {
          // No operation
        }
        break;
      case 4:
        this->align_counter = 0;

        if(this->si_diff <= this->eps_si)
        {
          // Transition to State 5 if SI reaches the desired value
          this->fsm_state = 5;
        }
        else
        {
          if(this->ptip_xdiff > 10)
          {
            // Transition back to State 2 if the tip is far right away from the center
            this->fsm_state = 2;
          }
          else
          {
            // No operation
          }
        }
        //}
        break;
      case 5:
        if(this->si_diff > this->eps_si)
        {
          // Transition to State 4 if SI deviates from the desired value
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
  }
  this->fsmExecution();
}

void Ur5eSuVisualServoingClass::fsmExecution()
{
  ROS_INFO_STREAM("fsmExecution...");

  switch(this->fsm_state)
  {
    case 1:
      this->unknownLocationVelocity();
      break;

    case 2:
      this->centeringVelocity();
      break;

    case 3:
      this->aligningVelocity();
      break;

    case 4:
      this->findTipVelocity();
      break;

    case 5:
      this->centerTipVelocity();
      break;
  }

  ROS_INFO("Planning path...");
  if(!this->debug_segmentation)
  {
    this->planPath();
  }
}

void Ur5eSuVisualServoingClass::calculateSVD()
{
  MatrixXd eigVec;
  double temp;

  this->calculateCovI();
  Eigen::JacobiSVD<MatrixXd> svd(this->covI, Eigen::ComputeFullV);
  this->lambdas = svd.singularValues();
  eigVec = svd.matrixV();

  // Sort small to large
  if(this->lambdas(0) > this->lambdas(1))
  {
    temp = this->lambdas(0);
    this->lambdas(0) = this->lambdas(1);
    this->lambdas(1) = temp;
    this->eigVectors.block(0, 0, 2, 1) = eigVec.block(0, 1, 2, 1);
    this->eigVectors.block(0, 1, 2, 1) = eigVec.block(0, 0, 2, 1);
  }
  else
  {
    this->eigVectors = eigVec;
  }
}

void Ur5eSuVisualServoingClass::calculateCovI()
{
  this->covI(0, 0) =
    (this->img_moments.m20 / this->img_moments.m00) - pow(this->pcenter(0), 2);
  this->covI(1, 0) = (this->img_moments.m11 / this->img_moments.m00)
    - (this->pcenter(0) * this->pcenter(1));
  this->covI(0, 1) = this->covI(1, 0);
  this->covI(1, 1) =
    (this->img_moments.m02 / this->img_moments.m00) - pow(this->pcenter(1), 2);
}

void Ur5eSuVisualServoingClass::calculateMoments()
{
  ROS_DEBUG_STREAM("Calculating moments...");
  this->img_moments =
    cv::moments(this->segmented_img, true);  // flag to treat image as binary
  this->tg_areas(this->buf_counter) = this->img_moments.m00;
  this->delAs(this->buf_counter) = this->tg_areas(this->buf_counter)
    - this->tg_areas((this->buf_counter + 1) % this->buf_length);
}

void Ur5eSuVisualServoingClass::calculatePCenter()
{
  ROS_DEBUG_STREAM("Calculating PCenter...");
  // Compute Pcenter
  this->pcenter(0) = this->img_moments.m10 / this->img_moments.m00;
  this->pcenter(1) = this->img_moments.m01 / this->img_moments.m00;

  // Compute difference between Pcenter and desired Pcenter
  this->pcent_xdiff = this->pcenter(0) - this->pcent_star(0);
}

void Ur5eSuVisualServoingClass::calculatePTip()
{
  ROS_DEBUG_STREAM("Calculating PTip...");
  double eig_x;
  double eig_y;
  VectorXd ptip_1 = VectorXd::Zero(2);
  VectorXd ptip_2 = VectorXd::Zero(2);

  eig_x =
    (2 * sqrt(this->lambdas(1))
     * this->eigVectors(0, 1));  // cv axes and image axes are flipped
  eig_y = (2 * sqrt(this->lambdas(1)) * this->eigVectors(1, 1));

  ptip_1(0) = this->pcenter(0) + eig_x;
  ptip_1(1) = this->pcenter(1) + eig_y;

  ptip_2(0) = this->pcenter(0) - eig_x;
  ptip_2(1) = this->pcenter(1) - eig_y;

  // Get sign of the desired movement for the find tip state
  if(ptip_1(0) > ptip_2(0))
  {
    this->find_tip_vx =
      this->sign(ptip_1(1) - ptip_2(1)) * abs(this->find_tip_vx);
  }
  else
  {
    this->find_tip_vx =
      this->sign(ptip_2(1) - ptip_1(1)) * abs(this->find_tip_vx);
  }

  // Determine ptip to be at the larger z coordinate
  if(ptip_2(1) > ptip_1(1))
  {
    this->ptip = ptip_2;
  }
  else
  {
    this->ptip = ptip_1;
  }

  // Compute difference between PTip and desired PTip
  this->ptip_xdiff = this->ptip(0) - this->ptip_star(0);
  ROS_DEBUG_STREAM("PTip = [" << this->ptip(0) << ", " << this->ptip(1) << "]");
}

void Ur5eSuVisualServoingClass::calculateSI()
{
  ROS_DEBUG_STREAM("Calculating SI...");
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
  int si;
  double r0;
  double x0;
  double y0;
  double dr;
  double dy;
  double th = M_PI / 5.0;

  if(this->img_moments.m00 < 1e-6)
  {
    ROS_DEBUG_STREAM("calculateSI: empty image");
    this->SIs(this->buf_counter) = 0;
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
    x0 = 247 / 2.0;
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

    if(cv::sum(x_trans <= -40)[0] > 0)
    {
      si = si + 1;
    }
    else
    {
      // No operation
    }

    if(cv::sum(x_trans >= 40)[0] > 0)
    {
      si = si + 1;
    }
    else
    {
      // No operation
    }

    /* Use fan shape as boundary
    if(cv::sum(theta <= (-th / 2.0 + M_PI / 100))[0] > 0)
    {
      si = si + 1;
    }
    else
    {
      // No operation
    }

    if(cv::sum(theta >= (th / 2.0 - M_PI / 100))[0] > 0)
    {
      si = si + 1;
    }
    else
    {
      // No operation
    } */

    if(cv::sum(r >= (r0 - dr))[0] > 0)
    {
      si = si + 1;
    }
    else
    {
      // No operation
    }
    this->SIs(this->buf_counter) = si;
    this->si_diff = abs(this->SIs.mean() - this->si_star);
    ROS_DEBUG_STREAM("SI = " << si << " Mean SI = " << this->SIs.mean());
  }
}

void Ur5eSuVisualServoingClass::imageSegmentation()
{
  ROS_DEBUG_STREAM("Image segmetation...");
  this->imgPreProcessing();

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
}

void Ur5eSuVisualServoingClass::saveDebugData()
{
  std::ofstream file;
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
    this->ss.str("");
    this->ss << this->save_path << "/debug_data.txt";
    file.open(this->ss.str(), std::ios_base::app);  // append to file
    if(file.is_open())
    {
      file << "img_" << std::setw(6) << std::setfill('0') << this->img_ct
           << " m00 = " << this->img_moments.m00
           << " m01 = " << this->img_moments.m01
           << " m10 = " << this->img_moments.m10
           << " m11 = " << this->img_moments.m11
           << " m02 = " << this->img_moments.m02
           << " m20 = " << this->img_moments.m20 << " lambdas = ["
           << this->lambdas(0) << ", " << this->lambdas(1) << "] v = ["
           << this->eigVectors(0, 0) << ", " << this->eigVectors(0, 1) << "; "
           << this->eigVectors(1, 0) << ", " << this->eigVectors(1, 1)
           << "] covI = [" << this->covI(0, 0) << ", " << this->covI(0, 1)
           << "; " << this->covI(1, 0) << ", " << this->covI(1, 1)
           << "] pcent = [" << this->pcenter(0) << ", " << this->pcenter(1)
           << "] ptip = " << this->ptip(0) << ", " << this->ptip(1) << "]\n";
      file.close();
    }
    else
    {
      ROS_WARN_STREAM("Error writing data to " << this->ss.str());
    }
    this->img_ct++;
  }
  else
  {
    // No operation
  }
}

void Ur5eSuVisualServoingClass::imgPreProcessing()
{
  Mat img;
  Mat color_img;
  Mat grey_img;
  Mat squared;
  double mean_squared_power;

  img = this->us_img->image;
  cv::cvtColor(img, grey_img, cv::COLOR_BGR2GRAY);
  this->preprocessed_img = grey_img;

  pow(this->preprocessed_img, 2, squared);
  mean_squared_power = sqrt(cv::sum(squared)[0])
    / (this->preprocessed_img.rows * this->preprocessed_img.cols);
  if(mean_squared_power > 0.15)
  {
    // Discard image if mean power is too high
    ROS_DEBUG_STREAM(
      "Image discarded - mean power = " << mean_squared_power << " is too high");
    this->preprocessed_img = Mat::zeros(
      this->preprocessed_img.rows, this->preprocessed_img.cols, CV_8U);
  }
}

void Ur5eSuVisualServoingClass::rescaleImgRange(Mat& img)
{
  double min;
  double max;
  Mat rescaled_img;

  rescaled_img = Mat::zeros(img.rows, img.cols, CV_32F);
  cv::minMaxLoc(img, &min, &max);
  rescaled_img = 255 * img / max;
  img = rescaled_img;
}

void Ur5eSuVisualServoingClass::needleEnhancingFilter()
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

void Ur5eSuVisualServoingClass::threshold(Mat img_in, double thresh, Mat& img_out)
{
  Mat mask;

  img_out.setTo(cv::Scalar(0));
  mask = (img_in >= thresh);
  img_out.setTo(255, mask);
}

void Ur5eSuVisualServoingClass::morphClosing()
{
  Mat se;
  Mat img_close;

  se = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(15, 15));
  img_close = this->proc_img_needle_u;

  // TODO: See how this differs from using a larger kernel a single time.
  for(int i = 0; i < 4; i++)
  {
    cv::morphologyEx(img_close, img_close, cv::MORPH_CLOSE, se);
  }

  this->proc_img_needle_u = img_close;
}

void Ur5eSuVisualServoingClass::momentFilter()
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

void Ur5eSuVisualServoingClass::combineFilteredImgs()
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

    if((num_comp > 1) && (num_comp <= 15))
    {
      // Mat mask(label_img.size(), CV_8UC1, cv::Scalar(0));
      // Segment largest region as the target mask
      sorted_ind = Mat::zeros(num_comp, 1, CV_16U);
      cv::sortIdx(
        stats.col(4), sorted_ind, CV_SORT_EVERY_COLUMN + CV_SORT_DESCENDING);
      x0 = 0;
      y0 = 0;
      j0 = 0;

      mask.setTo(cv::Scalar(0));
      mask = (label_img == sorted_ind.at<ushort>(1, 0));
      this->segmented_img.setTo(cv::Scalar(0));
      combined_img.copyTo(this->segmented_img, mask);

      // Segment the target image and background image
      target_img = Mat::zeros(combined_img.rows, combined_img.cols, CV_8UC1);
      this->preprocessed_img.copyTo(target_img, mask);

      bg_mask = Mat::zeros(mask.rows, mask.cols, CV_8UC1);
      cv::bitwise_not(mask, bg_mask);
      background_img =
        Mat::zeros(combined_img.rows, combined_img.cols, CV_8UC1);
      this->preprocessed_img.copyTo(background_img, bg_mask);

      if(this->debug_segmentation)
      {
        disp_img = Mat::zeros(
          2 * this->proc_img_needle_u.rows, 4 * this->proc_img_needle_u.cols,
          CV_8U);
        this->preprocessed_img.copyTo(disp_img(cv::Rect(
          x0, y0, this->preprocessed_img.cols, this->preprocessed_img.rows)));
        x0 += this->preprocessed_img.cols;
        ++j0;

        this->segmented_img.copyTo(disp_img(cv::Rect(
          x0, y0, this->segmented_img.cols, this->segmented_img.rows)));
        x0 += this->segmented_img.cols;
        ++j0;

        this->proc_img_needle_u.copyTo(disp_img(cv::Rect(
          x0, y0, this->proc_img_needle_u.cols, this->proc_img_needle_u.rows)));
        x0 += this->proc_img_needle_u.cols;
        ++j0;

        this->proc_img_moment_u.copyTo(disp_img(cv::Rect(
          x0, y0, this->proc_img_moment_u.cols, this->proc_img_moment_u.rows)));
        ++j0;

        target_img.copyTo(disp_img(cv::Rect(
          (j0 % 4) * target_img.cols, (j0 / 4) * target_img.rows,
          target_img.cols, target_img.rows)));
        ++j0;

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
          if(i0 < 2)
          {
            // Show mask of the largest 2 components only
            mask.copyTo(disp_img(cv::Rect(x0, y0, mask.cols, mask.rows)));
          }
          else
          {
            // No operation
          }
          ++j0;
        }
      }
      else
      {
        disp_img = Mat::zeros(
          this->preprocessed_img.rows, this->preprocessed_img.cols, CV_8U);
        this->preprocessed_img.copyTo(disp_img);
      }

      // Compute the mean of the target and background
      cv::meanStdDev(background_img, bg_mean, bg_std);
      ROS_DEBUG_STREAM("background mean = " << bg_mean.at<double>(0, 0));
      ROS_DEBUG_STREAM("background std = " << bg_std.at<double>(0, 0));

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

      Mat color_disp_img;
      cv::cvtColor(disp_img, color_disp_img, cv::COLOR_GRAY2BGR);
      x0 = 0;
      cv::circle(
        color_disp_img, cv::Point(x0 + this->ptip(0), this->ptip(1)), 2,
        cv::Scalar(0, 0, 255), 2);
      cv::circle(
        color_disp_img, cv::Point(x0 + this->pcenter(0), this->pcenter(1)), 2,
        cv::Scalar(255, 0, 0), 2);
      cv::line(
        color_disp_img, cv::Point(x0 + this->pcent_star(0), 0),
        cv::Point(x0 + this->pcent_star(0), this->preprocessed_img.rows),
        cv::Scalar(0, 255, 0), 1);
      x0 += this->preprocessed_img.cols;

      if(this->debug_segmentation)
      {
        cv::circle(
          color_disp_img, cv::Point(x0 + this->ptip(0), this->ptip(1)), 2,
          cv::Scalar(0, 0, 255), 2);
        cv::circle(
          color_disp_img, cv::Point(x0 + this->pcenter(0), this->pcenter(1)), 2,
          cv::Scalar(255, 0, 0), 2);
        cv::line(
          color_disp_img, cv::Point(x0 + this->pcent_star(0), 0),
          cv::Point(x0 + this->pcent_star(0), this->preprocessed_img.rows),
          cv::Scalar(0, 255, 0), 1);
      }
      else
      {
        // Rescale display image to be larger
        cv::resize(color_disp_img, color_disp_img, cv::Size(247 * 2, 156 * 2));
      }
      cv::imshow("Components", color_disp_img);
      cv::waitKey(5);

      // Publish disp img to topic
      cv_bridge::CvImage cv_msg;
      cv_msg.header = this->us_img->header;
      cv_msg.encoding = "bgr8";
      cv_msg.image = color_disp_img;
      disp_img_pub.publish(cv_msg.toImageMsg());
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

void Ur5eSuVisualServoingClass::unknownLocationVelocity()
{
  this->velocity = VectorXd::Zero(6);
  this->unknown_loc_counter = this->unknown_loc_counter % 50;
  if((this->unknown_loc_counter >= 10) && (this->unknown_loc_counter < 35))
  {
    this->velocity(1) = this->unknown_loc_vy;
  }
  else
  {
    this->velocity(1) = -this->unknown_loc_vy;
  }

  ROS_INFO_STREAM(
    "FSM [" << this->fsm_state << "] (unknown location):" << std::endl
            << "v = [" << (format("%.3e") % this->velocity(0)) << ", "
            << (format("%.3e") % this->velocity(1)) << ", "
            << (format("%.3e") % this->velocity(2)) << ", "
            << (format("%.3e") % this->velocity(3)) << ", "
            << (format("%.3e") % this->velocity(4)) << ", "
            << (format("%.3e") % this->velocity(5)) << "]");
}

void Ur5eSuVisualServoingClass::centeringVelocity()
{
  double K = 0.75;

  this->velocity = VectorXd::Zero(6);
  this->velocity(0) = K * this->pcent_xdiff * this->pixel_x;

  ROS_INFO_STREAM(
    "FSM [" << this->fsm_state << "] (centering):" << std::endl
            << "v = [" << (format("%.3e") % this->velocity(0)) << ", "
            << (format("%.3e") % this->velocity(1)) << ", "
            << (format("%.3e") % this->velocity(2)) << ", "
            << (format("%.3e") % this->velocity(3)) << ", "
            << (format("%.3e") % this->velocity(4)) << ", "
            << (format("%.3e") % this->velocity(5)) << "]");
}

void Ur5eSuVisualServoingClass::aligningVelocity()
{
  this->align_counter++;

  double wz_prev = this->velocity(5);

  if(this->align_counter == 1)
  {
    this->aligning_wz = this->max_aligning_wz;  // initialize wz to the max
  }
  else
  {
    // No operation
  }

  if(abs(wz_prev) < 1e-6)
  {
    wz_prev = this->aligning_wz;
  }
  else
  {
    // No operation
  }

  this->velocity = VectorXd::Zero(6);
  int curr_delA = this->delAs(this->buf_counter);
  int prev_delA = this->delAs(
    ((this->buf_counter - 1) + this->buf_length) % this->buf_length);
  // int prev_delA = this->delAs((this->buf_counter + 1) % this->buf_length);
  ROS_DEBUG_STREAM(
    "[aligningVelocity] curr_delA = " << curr_delA
                                      << " prev delA = " << prev_delA);
  if((abs(curr_delA) <= 5) && (abs(prev_delA) <= 5))
  {
    this->aligning_wz = 0.75 * this->aligning_wz;
    this->velocity(5) = sign(wz_prev) * this->aligning_wz;
  }
  else if(
    ((sign(curr_delA) == sign(prev_delA)) && (abs(curr_delA) < abs(prev_delA)))
    || (sign(curr_delA) > 0))
  {
    this->velocity(5) = wz_prev;
  }
  else
  {
    this->velocity(5) = -0.85 * wz_prev;
  }

  ROS_INFO_STREAM(
    "FSM [" << this->fsm_state << "] (aligning):" << std::endl
            << "v = [" << (format("%.3e") % this->velocity(0)) << ", "
            << (format("%.3e") % this->velocity(1)) << ", "
            << (format("%.3e") % this->velocity(2)) << ", "
            << (format("%.3e") % this->velocity(3)) << ", "
            << (format("%.3e") % this->velocity(4)) << ", "
            << (format("%.3e") % this->velocity(5)) << "]");
}

void Ur5eSuVisualServoingClass::findTipVelocity()
{
  this->velocity = VectorXd::Zero(6);
  this->velocity(0) = this->find_tip_vx;

  ROS_INFO_STREAM(
    "FSM [" << this->fsm_state << "] (find tip):" << std::endl
            << "v = [" << (format("%.3e") % this->velocity(0)) << ", "
            << (format("%.3e") % this->velocity(1)) << ", "
            << (format("%.3e") % this->velocity(2)) << ", "
            << (format("%.3e") % this->velocity(3)) << ", "
            << (format("%.3e") % this->velocity(4)) << ", "
            << (format("%.3e") % this->velocity(5)) << "]");
}

void Ur5eSuVisualServoingClass::centerTipVelocity()
{
  double K = 0.5;

  this->velocity = VectorXd::Zero(6);
  this->velocity(0) = K * this->ptip_xdiff * this->pixel_x;

  ROS_INFO_STREAM(
    "FSM [" << this->fsm_state << "] (center tip):" << std::endl
            << "v = [" << (format("%.3e") % this->velocity(0)) << ", "
            << (format("%.3e") % this->velocity(1)) << ", "
            << (format("%.3e") % this->velocity(2)) << ", "
            << (format("%.3e") % this->velocity(3)) << ", "
            << (format("%.3e") % this->velocity(4)) << ", "
            << (format("%.3e") % this->velocity(5)) << "]");
}

int Ur5eSuVisualServoingClass::sign(double val)
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

void Ur5eSuVisualServoingClass::planPath()
{
  Eigen::MatrixXd rotation(6, 6);

  ROS_DEBUG_STREAM("Transforming velocity to spatial frame...");
  const Eigen::Isometry3d& fwd_kin = this->getGlobalLinkTransform();
  rotation << fwd_kin.rotation(), Eigen::MatrixXd::Zero(3, 3),
    Eigen::MatrixXd::Zero(3, 3), fwd_kin.rotation();

  ROS_DEBUG_STREAM("Setting cartesian velocity...");
  this->setSpatialCartesianVelocity(rotation * this->velocity);
  ROS_INFO_STREAM("Sending goal...");
  this->sendGoalAndWait();
  ROS_INFO_STREAM("Trajectory executed.");
}

void Ur5eSuVisualServoingClass::moveToInitialPosition()
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
