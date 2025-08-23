#include "pulse_vs_sim/sim_img_publish_class.hpp"

std::vector<std::string> split(std::string s, std::string delimiter)
{
  size_t pos_start = 0, pos_end, delim_len = delimiter.length();
  std::string token;
  std::vector<std::string> res;

  while((pos_end = s.find(delimiter, pos_start)) != std::string::npos)
  {
    token = s.substr(pos_start, pos_end - pos_start);
    pos_start = pos_end + delim_len;
    res.push_back(token);
  }

  res.push_back(s.substr(pos_start));
  return res;
}

PaimdbSimImgPublisher::PaimdbSimImgPublisher(ros::NodeHandle& nh) : nh(nh), sim_img_id(1)
{
  int i0, i1;

  // Fetch parameters from the parameter server.
  if(!this->nh.getParam("/deep_learning/num_sim_img", this->num_sim_img))
  {
    ROS_WARN("Could not find parameter '/deep_learning/num_sim_img', setting "
             "number of simulated images to default value (20000)...");
    this->num_sim_img = 20000;
  }

  if(!this->nh.getParam("/deep_learning/sim_img_dir_name", this->sim_img_dir_name))
  {
    ROS_WARN("Could not find parameter '/deep_learning/sim_img_dir_name', "
             "using default value...");
    this->sim_img_dir_name = "/home/mardava/Datasets/paimdb/images/"
                             "p42v_2d_12cm_1s1r_20230703_926x1130/Images";
  }

  if(!this->nh.getParam("/deep_learning/zero_pad_width_px", this->zero_pad_width_px))
  {
    ROS_WARN("Could not find parameter '/deep_learning/zero_pad_width_px', "
             "using default value...");
    this->zero_pad_width_px = 502;
  }

  // Clean up the path to the simulated images.
  std::vector<std::string> sim_img_dir_vec = split(this->sim_img_dir_name, "/");
  std::vector<bool> include_dir(sim_img_dir_vec.size(), true);

  for(i0 = 0; i0 < sim_img_dir_vec.size(); i0++)
  {
    if(sim_img_dir_vec[i0].compare("..") == 0)
    {
      for(i1 = i0 - 1; i1 >= 0; i1--)
      {
        if(include_dir[i1])
        {
          include_dir[i1] = false;
          include_dir[i0] = false;
          break;
        }
      }
    }
  }

  if((sim_img_dir_vec[0].empty()) || (sim_img_dir_vec[0].compare("..") == 0))
  {
    this->sim_img_dir_name = "";
  }
  else
  {
    this->sim_img_dir_name = "/";
  }

  for(i0 = 0; i0 < sim_img_dir_vec.size(); i0++)
  {
    if(include_dir[i0])
    {
      this->sim_img_dir_name += sim_img_dir_vec[i0];

      if(i0 < sim_img_dir_vec.size() - 1)
      {
        this->sim_img_dir_name += "/";
      }
    }
  }

  // Initialize the simulated image publisher.
  this->it = new image_transport::ImageTransport(this->nh);
  this->sim_img_pub = this->it->advertise("/verasonics/channel_data", 10);

  // Match this timer to the pulse repetition rate of the laser.
  this->sim_img_timer = this->nh.createTimer(ros::Duration(0.1), &PaimdbSimImgPublisher::timerCallback, this);
}

PaimdbSimImgPublisher::~PaimdbSimImgPublisher()
{
}

void PaimdbSimImgPublisher::timerCallback(const ros::TimerEvent& e)
{
  char img_file_name[256];
  snprintf(img_file_name, 255, "%s/%06d.jpg", this->sim_img_dir_name.c_str(), this->sim_img_id);
  std::string img_file_name_str(img_file_name);

  ROS_INFO_STREAM(
    ""
    << "Loading and publishing file" << std::endl
    << "'" << img_file_name_str << "'...");
  cv::Mat img_mat = cv::imread(img_file_name_str, cv::IMREAD_COLOR);
  ROS_INFO_STREAM(
    ""
    << "Read image of dimensions [" << img_mat.rows << ", " << img_mat.cols << "]");
  // img_mat = img_mat(
  //  cv::Range(0, img_mat.rows),
  //  cv::Range(this->zero_pad_width_px, this->zero_pad_width_px + 64 - 1));
  sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_mat).toImageMsg();
  this->sim_img_pub.publish(img_msg);

  // Increment the index of the simulated image to be read and published. The index
  // counts from one, so perform the modulus operation before incrementing.
  this->sim_img_id %= this->num_sim_img;
  this->sim_img_id++;
}
