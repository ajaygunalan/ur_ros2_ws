#include "pulse_ur5e_sweep/ur5e_su_sweep_class.hpp"

// Constructor
Ur5eSuSweepClass::Ur5eSuSweepClass(
  ros::NodeHandle& nh, double scan_dist, int max_x_steps, int max_y_steps)
  : mgi::Ur5eMoveGroupInterfaceClass(nh)
{
  std::time_t time_stamp = std::chrono::system_clock::to_time_t(
    std::chrono::system_clock::now());  // TODO: fix date string format
  auto time_stamp_str =
    std::put_time(std::localtime(&time_stamp), "%Y%m%d-%H%M%S");
  std::stringstream transform_file_name;

  auto date_str = std::put_time(std::localtime(&time_stamp), "%Y%m%d");
  this->data_path << "/home/verasonics/Desktop/Jessica_REU2019/" << date_str;

  transform_file_name << data_path.str() << "_tf_" << time_stamp_str << ".csv";

  this->scan_dist = scan_dist;
  this->grid_position_id[0] = 0;
  this->grid_position_id[1] = 0;
  this->max_grid_position_id[0] = max_x_steps;
  this->max_grid_position_id[1] = max_y_steps;

  // Subscribe to the /verasonics/das_us_img ROS topic.
  this->das_us_img_sub = this->nh.subscribe(
    "/verasonics/das_us_img", 1000, &Ur5eSuSweepClass::dasUsImgAcqCallback,
    this);

  // Set up a timer to call the runCallback function periodically.
  this->run_timer = this->nh.createTimer(
    ros::Duration(5.0), &Ur5eSuSweepClass::runCallback, this);
  this->transform_fh.open(transform_file_name.str(), std::ios_base::app);

  // TODO: add in folder information and initializing parameters to command ur5
}

Ur5eSuSweepClass::~Ur5eSuSweepClass()
{
  this->transform_fh.close();
}

// update the current image
void Ur5eSuSweepClass::dasUsImgAcqCallback(const sensor_msgs::ImageConstPtr& img)
{
  // convert ROS image to CV image and save
  try
  {
    this->das_us_img =
      cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}

// save the image and append the tf data to file
bool Ur5eSuSweepClass::saveImgTfToFile()
{
  std::time_t time_stamp = std::chrono::system_clock::to_time_t(
    std::chrono::system_clock::now());  // TODO: fix date string format
  auto time_stamp_str =
    std::put_time(std::localtime(&time_stamp), "%Y%m%d-%H%M%S");

  std::stringstream sstream;
  sstream << this->data_path.str() << "BeamformedImg" << time_stamp_str
          << ".png";

  ROS_ASSERT(cv::imwrite(sstream.str(), this->das_us_img->image));

  ROS_INFO_STREAM("Image Saved");

  // append entry to existing file
  this->transform_fh
    << time_stamp_str << ","
    << this->ur5e_base_tool0_curr_tf.transform.translation.x << ","
    << this->ur5e_base_tool0_curr_tf.transform.translation.y << ","
    << this->ur5e_base_tool0_curr_tf.transform.translation.z << ","
    << this->ur5e_base_tool0_curr_tf.transform.rotation.w << ","
    << this->ur5e_base_tool0_curr_tf.transform.rotation.x << ","
    << this->ur5e_base_tool0_curr_tf.transform.rotation.y << ","
    << this->ur5e_base_tool0_curr_tf.transform.rotation.z << std::endl;
  this->transform_fh.flush();
  ROS_INFO_STREAM("Tf appended");

  return true;
}

// start scan
void Ur5eSuSweepClass::runCallback(const ros::TimerEvent& e)
{
  tf::StampedTransform st_base_tool0;
  moveit::planning_interface::MoveGroupInterface::Plan motion_plan;
  moveit::core::MoveItErrorCode plan_err;
  moveit::core::MoveItErrorCode move_err;

  // Step the robot.
  if(this->grid_position_id[0] < this->max_grid_position_id[0])
  {
    if(this->grid_position_id[0] == 0 && this->grid_position_id[1] == 0)
    {
      try
      {
        // TODO: Make robot horizontal.

        // Get the current pose of the robot, and save it as the starting pose.
        this->tf_listener.lookupTransform(
          this->mgi_obj->getPoseReferenceFrame(), this->probe_frame_name,
          ros::Time(0), st_base_tool0);
        tf::transformStampedTFToMsg(
          st_base_tool0, this->ur5e_base_tool0_start_tf);

        this->mgi_obj->setStartStateToCurrentState();

        // make robot end effector perpendicular to xy plane
        geometry_msgs::PoseStamped starting_target_pose;
        starting_target_pose.header = this->ur5e_base_tool0_start_tf.header;
        starting_target_pose.pose.position.x =
          this->ur5e_base_tool0_start_tf.transform.translation.x;
        starting_target_pose.pose.position.y =
          this->ur5e_base_tool0_start_tf.transform.translation.y;
        starting_target_pose.pose.position.z =
          this->ur5e_base_tool0_start_tf.transform.translation.z;

        starting_target_pose.pose.orientation.w = 0;
        starting_target_pose.pose.orientation.x = -1 / sqrt(2);
        starting_target_pose.pose.orientation.y = 0;
        starting_target_pose.pose.orientation.z = 1 / sqrt(2);

        this->mgi_obj->setPoseTarget(starting_target_pose);
        plan_err = this->mgi_obj->plan(motion_plan);

        if(plan_err == moveit::core::MoveItErrorCode::SUCCESS)
        {
          ROS_INFO_STREAM("Executing path for starting position...");
          move_err = this->mgi_obj->execute(motion_plan);

          if(move_err == moveit::core::MoveItErrorCode::SUCCESS)
          {
            ROS_DEBUG("Successfully moved robot to horizontal pose.");
          }
          else
          {
            ROS_WARN_STREAM("Could not move robot, error " << move_err);
          }

          // Update the starting pose to the current pose of the robot after the
          // horizontal alignment.
          this->tf_listener.lookupTransform(
            this->mgi_obj->getPoseReferenceFrame(), this->probe_frame_name,
            ros::Time(0), st_base_tool0);
          tf::transformStampedTFToMsg(
            st_base_tool0, this->ur5e_base_tool0_start_tf);
        }
        else
        {
          ROS_WARN_STREAM("Could not plan path, error " << plan_err);
        }

        this->ur5e_base_tool0_curr_tf = this->ur5e_base_tool0_start_tf;
      }
      catch(tf::TransformException& e)
      {
        // Print the error, and reattempt on next timer callback.
        ROS_ERROR("%s", e.what());
      }
    }
    else
    {
      // Save latest image and transformation to file.
      this->saveImgTfToFile();

      // Move the robot to the target pose.
      geometry_msgs::PoseStamped target_pose;
      target_pose.header = this->ur5e_base_tool0_start_tf.header;

      target_pose.pose.position.x =
        this->ur5e_base_tool0_start_tf.transform.translation.x
        + (this->grid_position_id[0] * this->scan_dist);
      target_pose.pose.position.y =
        this->ur5e_base_tool0_start_tf.transform.translation.y
        + (this->grid_position_id[1] * this->scan_dist);
      target_pose.pose.position.z =
        this->ur5e_base_tool0_start_tf.transform.translation.z;

      // TODO: Fix hard-coded target pose orientation.
      target_pose.pose.orientation.w =
        this->ur5e_base_tool0_start_tf.transform.rotation.w;
      target_pose.pose.orientation.x =
        this->ur5e_base_tool0_start_tf.transform.rotation.x;
      target_pose.pose.orientation.y =
        this->ur5e_base_tool0_start_tf.transform.rotation.y;
      target_pose.pose.orientation.z =
        this->ur5e_base_tool0_start_tf.transform.rotation.z;
      ROS_INFO_STREAM(
        "Planning path for grid position (" << this->grid_position_id[0] << ", "
                                            << this->grid_position_id[1]
                                            << ")...");

      this->mgi_obj->setStartStateToCurrentState();
      this->mgi_obj->setPoseTarget(target_pose);
      plan_err = this->mgi_obj->plan(motion_plan);

      // TODO: Put move command over here.
      // NOTE: This assumes that MoveIt will not move the robot if the current
      // and desired poses are identical.
      if(plan_err == moveit::core::MoveItErrorCode::SUCCESS)
      {
        ROS_INFO_STREAM(
          "Executing path for grid position ("
          << this->grid_position_id[0] << ", " << this->grid_position_id[1]
          << ")...");
        move_err = this->mgi_obj->execute(motion_plan);

        if(move_err == moveit::core::MoveItErrorCode::SUCCESS)
        {
          ROS_DEBUG("Successfully moved robot to target pose.");
        }
        else
        {
          ROS_WARN_STREAM("Could not move robot, error " << move_err);
        }
      }
      else
      {
        // TODO: See if splitting the path into smaller segments would help in this case.
        ROS_WARN_STREAM("Could not plan path, error " << plan_err);
      }

      // Acquire latest transformation.
      try
      {
        ROS_INFO("Fetching updated transformation from base_link to tool0...");
        this->tf_listener.lookupTransform(
          this->mgi_obj->getPoseReferenceFrame(), this->probe_frame_name,
          ros::Time(0), st_base_tool0);
        tf::transformStampedTFToMsg(
          st_base_tool0, this->ur5e_base_tool0_curr_tf);
      }
      catch(tf::TransformException& e)
      {
        // Print the error, and reattempt on next timer callback.
        ROS_ERROR("%s", e.what());
      }
    }

    // Increment grid position indices.
    ROS_DEBUG("Incrementing grid indices...");
    this->grid_position_id[1]++;

    if(this->grid_position_id[1] >= this->max_grid_position_id[1])
    {
      this->grid_position_id[1] = 0;
      this->grid_position_id[0]++;
    }
    else
    {
      // No operation
    }
  }
  else
  {
    if(this->grid_position_id[1] == 0)
    {
      // Save the final image.
      this->saveImgTfToFile();
      this->grid_position_id[1]++;
      this->~Ur5eSuSweepClass();
      ROS_INFO_STREAM("Scan completed");
    }
    else
    {
      // No operation
    }
  }
}
