#include "pulse_vs_sim/pa_chdat_gen_class.hpp"

/*
 * \brief Construct an object of the type `PaChannelDataGenerator`
 */
vs_sim::PaChannelDataGenerator::PaChannelDataGenerator(ros::NodeHandle& nh)
  : nh(nh), probe_frame_name("probe"), pt_src_frame_name("pt_src")
{
  double min_amp_scaling;
  double max_amp_scaling;
  std::string chdat_h5_file_name;

  // Fetch the name of the directory containing the simulated dataset to be used for photoacoustic channel data frame
  // generation.
  if(this->nh.hasParam("dataset_dir_name"))
  {
    this->nh.getParam("dataset_dir_name", this->dataset_dir_name);
  }
  else
  {
    ROS_WARN("Could not find parameter `dataset_dir_name`, using default value...");
    this->dataset_dir_name =
      "/home/mardava/Datasets/paimdb/sources/p42v_3d_12cm_20230627_162555/images/rt_xy_01s_20240710_154100/chdat/";
  }

  // Fetch the image generation timer period from the ROS parameter server.
  if(this->nh.hasParam("delta_t"))
  {
    this->nh.getParam("delta_t", this->delta_t);
  }
  else
  {
    ROS_WARN("Could not find parameter `delta_t`, using default value...");
    this->delta_t = 0.1;
  }

  // Fetch the desired image dimensions.
  if(this->nh.hasParam("img_height_px"))
  {
    this->nh.getParam("img_height_px", this->img_height_px);
  }
  else
  {
    ROS_WARN("Could not find parameter `img_height_px`, using default value...");
    this->img_height_px = 926;
  }

  if(this->nh.hasParam("img_width_px"))
  {
    this->nh.getParam("img_width_px", this->img_width_px);
  }
  else
  {
    ROS_WARN("Could not find parameter `img_width_px`, using default value...");
    this->img_width_px = 256;
  }

  // Fetch the minimum and maximum amplitude scaling factors.
  if(this->nh.hasParam("max_amp_scaling"))
  {
    this->nh.getParam("max_amp_scaling", max_amp_scaling);
  }
  else
  {
    ROS_WARN("Could not find parameter `max_amp_scaling`, using default value...");
    max_amp_scaling = 1.1;
  }

  if(this->nh.hasParam("min_amp_scaling"))
  {
    this->nh.getParam("min_amp_scaling", min_amp_scaling);
  }
  else
  {
    ROS_WARN("Could not find parameter `min_amp_scaling`, using default value...");
    min_amp_scaling = 0.75;
  }

  // Fetch the speed of sound to be used in the creation of simulated photoacoustic channel data frames.
  if(this->nh.hasParam("sim_speed_of_sound"))
  {
    this->nh.getParam("sim_speed_of_sound", this->sim_speed_of_sound);
  }
  else
  {
    ROS_WARN("Could not find parameter `sim_speed_of_sound`, using default value...");
    this->sim_speed_of_sound = 1540.0;
  }

  // Fetch the sampling frequency of the ultrasound transducer to be simulated.
  if(this->nh.hasParam("trans_samp_freq"))
  {
    this->nh.getParam("trans_samp_freq", this->trans_samp_freq);
    ROS_DEBUG("Transducer sampling frequency set to %.2f Hz.", this->trans_samp_freq);
  }
  else
  {
    ROS_WARN("Could not find parameter `trans_samp_freq`, using default value...");
    this->trans_samp_freq = 4.0 * 2.97e6;
  }

  // Fetch lists of simulated lateral, elevation, and axial positions for binary searches later.
  chdat_h5_file_name = this->dataset_dir_name + "chdat.h5";
  HighFive::File chdat_h5_fh(chdat_h5_file_name, HighFive::File::ReadOnly);
  auto ch_data_x_dataset = chdat_h5_fh.getDataSet("/ch_data/x_vec");
  this->pt_src_x_vec = ch_data_x_dataset.read<std::vector<double>>();
  auto ch_data_y_dataset = chdat_h5_fh.getDataSet("/ch_data/y_vec");
  this->pt_src_y_vec = ch_data_y_dataset.read<std::vector<double>>();
  auto ch_data_z_dataset = chdat_h5_fh.getDataSet("/ch_data/z_vec");
  this->pt_src_z_vec = ch_data_z_dataset.read<std::vector<double>>();

  // Initialize the random number generator for reflection artifact position generation.
  this->ref_pos_distribution = new std::normal_distribution<double>(0.0, 2e-3);

  // Initialize the random number generator for the amplitude scaling factor.
  this->amp_scale_distribution = new std::uniform_real_distribution<double>(min_amp_scaling, max_amp_scaling);

  // Initialize publishers and timers.
  this->tf_listener = new tf2_ros::TransformListener(this->tf_buffer);

  // this->img_nh = new image_transport::ImageTransport(nh);
  // this->pa_chdat_pub = this->img_nh->advertise("/verasonics/channel_data", 1);
  this->pa_chdat_pub = this->nh.advertise<sensor_msgs::Image>("/verasonics/channel_data", 1);
  this->pub_timer = this->nh.createTimer(ros::Duration(this->delta_t), &PaChannelDataGenerator::timerCallback, this);
}

/*
 * \brief Get the channel data frame corresponding to the given source or reflection artifact position.
 */
bool vs_sim::PaChannelDataGenerator::getChannelDataFrame(
  Eigen::Vector3d src_ref_pos, cv::Mat& output_img, int num_downshift_px)
{
  bool point_found;
  cv::Mat input_img_shifted;

  // Character arrays containing sub-directory and file base names
  char x_str[PULSE_SRC_REF_POS_STR_LEN];
  char y_str[PULSE_SRC_REF_POS_STR_LEN];
  char z_str[PULSE_SRC_REF_POS_STR_LEN];

  // Input image corresponding to the given source or reflection artifact position
  cv::Mat input_img;
  cv::Mat input_img_converted;

  // Indices of simulated positions closest to current source or artifact location in probe frame
  int x_id;
  int y_id;
  int z_id;

  // Names of files related to simulated data
  std::string input_h5_file_name;
  std::string input_img_file_name;

  // Vectors containing scaling information related to loaded channel data frames
  std::vector<double> ch_data_min_vec;
  std::vector<double> ch_data_max_vec;
  std::vector<double> ch_data_mean_vec;

  // Determine the corresponding file and directory indices from the lists of simulated lateral, elevation, and axial
  // positions using the current point source location in the probe frame.
  x_id = this->binarySearch(src_ref_pos(0), this->pt_src_x_vec) + 1;
  z_id = this->binarySearch(src_ref_pos(2), this->pt_src_z_vec) + 1;

  // Exploit the elevation symmetry of the relationship between waveform shape and source position.
  y_id = this->binarySearch(abs(src_ref_pos(1)), this->pt_src_y_vec) + 1;

  if((x_id >= 1) && (y_id >= 1) && (z_id >= 1))
  {
    // Load minimum and maximum values with which to rescale the channel data frame from the corresponding HDF5 file.
    snprintf(x_str, PULSE_SRC_REF_POS_STR_LEN, "%03d", x_id);
    snprintf(y_str, PULSE_SRC_REF_POS_STR_LEN, "%03d", y_id);
    snprintf(z_str, PULSE_SRC_REF_POS_STR_LEN, "%03d", z_id);
    input_h5_file_name = this->dataset_dir_name + z_str + "/" + y_str + ".h5";
    HighFive::File input_h5_fh(input_h5_file_name, HighFive::File::ReadOnly);
    auto ch_data_min_dataset = input_h5_fh.getDataSet("/ch_data/min_vec");
    ch_data_min_vec = ch_data_min_dataset.read<std::vector<double>>();
    auto ch_data_max_dataset = input_h5_fh.getDataSet("/ch_data/max_vec");
    ch_data_max_vec = ch_data_max_dataset.read<std::vector<double>>();
    auto ch_data_mean_dataset = input_h5_fh.getDataSet("/ch_data/mean_vec");
    ch_data_mean_vec = ch_data_mean_dataset.read<std::vector<double>>();

    // Generate the name of the image file containing the desired channel data.
    input_img_file_name = this->dataset_dir_name + z_str + "/" + y_str + "/" + x_str + ".jpg";
    ROS_DEBUG_STREAM("Reading file " << input_img_file_name);

    // Load the channel data frame, typecast, and rescale to the original amplitude range.
    input_img = cv::imread(input_img_file_name, cv::IMREAD_GRAYSCALE);
    input_img.convertTo(input_img_converted, CV_32FC1);
    input_img_converted = input_img_converted * ch_data_max_vec[x_id - 1] / 255.0;
    input_img_converted = input_img_converted + ch_data_min_vec[x_id - 1];

    if(num_downshift_px > 0)
    {
      this->downshiftChannelDataFrame(input_img_converted, output_img, num_downshift_px, ch_data_mean_vec[x_id - 1]);
    }
    else
    {
      output_img = input_img_converted;
    }

    point_found = true;
  }
  else
  {
    output_img.create(this->img_height_px, this->img_width_px, CV_32FC1);
    point_found = false;
  }

  return point_found;
}

/*
 * \brief Find the index of the element in the vector `v` closest in value to the desired value `x`
 */
int vs_sim::PaChannelDataGenerator::binarySearch(double x, std::vector<double> v)
{
  int mid;
  int left;
  int right;

  if(x > v.back() || x < v.front())
  {
    // The value `x` is outside the range of the vector `v`, so return -1 indicating that it was not found.
    mid = -1;
  }
  else
  {
    // Initialize the left and right indices to the ends of the vector `v`.
    left = 0;
    right = v.size() - 1;

    // The value `x` might not be found in vector `v`, so we cannot limit ourselves to exact matches.
    while(left + 1 < right)
    {
      mid = (left + right) / 2;

      if(x == v[mid])
      {
        // We found an exact match, so set the left and right indices to this exact match to simplify the processing
        // further below.
        left = mid;
        right = mid;

        break;
      }
      else if(x > v[mid])
      {
        // Value `x` is in the upper half of the sub-array currently considered to contain `x`.
        left = mid;
      }
      else
      {
        // Value `x` is in the lower half of the sub-array currently considered to contain `x`.
        right = mid;
      }
    }

    if(left != right)
    {
      // Indices `left` and `right` are expected to differ by one here, and `x` is expected to lie between `v[left]` and
      // `v[right]`.
      if(x > (v[left] + v[right]) / 2.0)
      {
        // Value `x` is closer to `v[right]` than `v[left]`.
        mid = right;
      }
      else
      {
        // Value `x` is closer to `v[left]` than `v[right]`.
        mid = left;
      }
    }
    else
    {
      // Indices `left` and `right` match, indicating that an exact match was found.
      // No operation
    }
  }

  return mid;
}

/*
 * \brief Axially downshift the channel data frame by the given number of pixels.
 */
void vs_sim::PaChannelDataGenerator::downshiftChannelDataFrame(
  cv::Mat& input_img, cv::Mat& output_img, int num_downshift_px, double mean_amp)
{
  int i0;
  int i1;

  output_img.create(input_img.rows, input_img.cols, CV_32FC1);
  input_img(cv::Range(0, input_img.rows - num_downshift_px - 1), cv::Range::all())
    .copyTo(output_img(cv::Range(num_downshift_px, input_img.rows - 1), cv::Range::all()));
  output_img(cv::Range(0, num_downshift_px - 1), cv::Range::all()) = mean_amp;
}

/*
 * \brief Timer callback function to execute photoacoustic channel data generation and publishing
 */
void vs_sim::PaChannelDataGenerator::timerCallback(const ros::TimerEvent& e)
{
  // Flag indicating whether or not a waveform corresponding to the current source position could be constructed.
  bool src_found;

  // Original and processed images of channel data frames corresponding to source, reflection artifact, and output.
  cv::Mat output_img;
  cv::Mat output_img_rescaled;
  cv::Mat ref_img;
  cv::Mat src_img;

  // Limits of output image before rescaling, required for rescaling, typecasting, and publishing.
  cv::Point output_min_loc;
  cv::Point output_max_loc;
  double output_min_val;
  double output_max_val;

  // Amplitude scaling factors for source and reflection artifact.
  double ref_amp_scaling;
  double src_amp_scaling;

  // Distance between source and reflection artifact.
  double src_ref_distance;

  // Positions of the source and artifact in the probe reference frame
  Eigen::Vector3d ref_pos_probe;
  Eigen::Vector3d src_pos_probe;

  // Number of pixels by which to downshift the reflection artifact.
  int num_downshift_px;

  // Runtime estimation variables
  ros::Time x_start = ros::Time::now();
  ros::Time x_end;

  // Image message to be published to ROS topic.
  sensor_msgs::ImagePtr img_msg;
  std_msgs::Header img_msg_header;

  // Transform from the point source frame to the probe frame
  geometry_msgs::TransformStamped src_pos_probe_msg;

  try
  {
    // Get the current position of the point source in the probe frame.
    // ROS_DEBUG_STREAM(
    //   "PaChannelDataGenerator: "
    //   << "Waiting for transform from frame '" << this->pt_src_frame_name << "' to frame '" << this->probe_frame_name
    //   << "'...");
    src_pos_probe_msg = this->tf_buffer.lookupTransform(this->probe_frame_name, this->pt_src_frame_name, ros::Time(0));

    // Extract the current position of the point source with respect to the probe.
    tf::vectorMsgToEigen(src_pos_probe_msg.transform.translation, src_pos_probe);

    // Generate a random position for a reflection artifact.
    ref_pos_probe(0) = (*(this->ref_pos_distribution))(this->rand_num_gen);
    ref_pos_probe(1) = (*(this->ref_pos_distribution))(this->rand_num_gen);
    ref_pos_probe(2) = (*(this->ref_pos_distribution))(this->rand_num_gen);
    ref_pos_probe = src_pos_probe + ref_pos_probe;
    src_ref_distance = (src_pos_probe - ref_pos_probe).norm();
    num_downshift_px = (int)(src_ref_distance * this->trans_samp_freq / this->sim_speed_of_sound);

    // Attempt to generate the photoacoustic channel data frame corresponding to the given source location.
    ROS_DEBUG("x_S = [%.2f, %.2f, %.2f] mm", 1e3 * src_pos_probe(0), 1e3 * src_pos_probe(1), 1e3 * src_pos_probe(2));
    src_found = this->getChannelDataFrame(src_pos_probe, src_img, 0);

    // Scale the amplitudes by a random factor.
    src_amp_scaling = (*(this->amp_scale_distribution))(this->rand_num_gen);
    output_img = (src_amp_scaling * src_img);

    if(src_found)
    {
      // Attempt to generate the photoacoustic channel data frame corresponding to the given reflection artifact.
      this->getChannelDataFrame(ref_pos_probe, ref_img, num_downshift_px);
      ref_amp_scaling = (*(this->amp_scale_distribution))(this->rand_num_gen);

      // Add the frames corresponding to the source and artifact.
      output_img = output_img + (ref_amp_scaling * ref_img);
    }
    else
    {
      // No operation
    }

    // Rescale, typecast, and publish the resulting MONO-8 image.
    cv::minMaxLoc(output_img, &output_min_val, &output_max_val, &output_min_loc, &output_max_loc);
    output_img = output_img - output_min_val;
    output_img = output_img * (255.0 / (output_max_val - output_min_val));
    output_img.convertTo(output_img_rescaled, CV_8UC1);

    img_msg_header.frame_id = "probe";
    img_msg_header.stamp = ros::Time::now();
    img_msg = cv_bridge::CvImage(img_msg_header, "mono8", output_img_rescaled).toImageMsg();
    this->pa_chdat_pub.publish(img_msg);
  }
  catch(tf2::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
  }

  x_end = ros::Time::now();
  ROS_DEBUG("Published channel data frame in %.2f ms.", 1e3 * (x_end.toSec() - x_start.toSec()));
}
