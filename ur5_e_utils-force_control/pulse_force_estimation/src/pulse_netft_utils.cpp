/**
 * \file pulse_netft_utils.cpp
 *
 * \brief Utilities to interface with ATI Gamma NET-FT sensor
 *
 * Replaces netft_utils.cpp in netft_utils package.
 *
 * \author Mardava Gubbi <mgubbi1@jhu.edu>
 */
#include "pulse_force_estimation/pulse_netft_utils.hpp"

PulseNetftUtils::PulseNetftUtils(ros::NodeHandle nh)
  : nh(nh)
  , lpf_is_on(false)
  , lpf_delta_t(0.0)
  , lpf_cutoff_freq(0.0)
  , lpf_new(false)
  , bias_is_on(false)
  , bias_new(false)
  , grav_comp_new(false)
  , grav_comp_is_on(false)
  , grav_comp_payload_weight(0.)
  , grav_comp_payload_lever_arm(0.)
  , cancel_count(CANCEL_COUNT_MAX)
  , cancel_wait(CANCEL_WAIT_MAX)
  , force_max_b(10.0)
  , torque_max_b(0.8)
  , force_max_u(50.0)
  , torque_max_u(5.0)
{
}

PulseNetftUtils::~PulseNetftUtils(void)
{
  delete this->tf_listener;
  delete this->lpf_h;
}

void PulseNetftUtils::initialize(void)
{
  // this->lpf_h = new LPFilter(0.002,200,6);

  // Zero out the zero wrench
  this->zero_wrench.wrench.force.x = 0.0;
  this->zero_wrench.wrench.force.y = 0.0;
  this->zero_wrench.wrench.force.z = 0.0;
  this->zero_wrench.wrench.torque.x = 0.0;
  this->zero_wrench.wrench.torque.y = 0.0;
  this->zero_wrench.wrench.torque.z = 0.0;

  // Initialize cancel message
  this->cancel_msg.toCancel = false;

  // Listen to the transfomation from the ft sensor to world frame.
  this->tf_listener = new tf::TransformListener(ros::Duration(300));

  // Publishers on the /netft topics. Queue up to 100000 data points.
  this->netft_raw_world_data_pub =
    this->nh.advertise<geometry_msgs::WrenchStamped>("/netft/raw_world", 100000);
  this->netft_proc_world_data_pub =
    this->nh.advertise<geometry_msgs::WrenchStamped>(
      "/netft/proc_world", 100000);
  this->netft_proc_tool_data_pub =
    this->nh.advertise<geometry_msgs::WrenchStamped>("/netft/proc_tool", 100000);

  // TODO: Test this link to the appropriate robot motion cancel topic using the
  // subscriber in the action client class (Ur5eBaseClass).
  this->netft_cancel_pub =
    this->nh.advertise<netft_utils::Cancel>("/netft/cancel", 100000);

  // Advertise bias and threshold services
  this->bias_service = this->nh.advertiseService(
    "/netft/set_bias", &PulseNetftUtils::setFixedOrientationBias, this);
  this->grav_comp_service = this->nh.advertiseService(
    "/netft/set_gravity_comp", &PulseNetftUtils::setGravityCompensation, this);
  this->set_max_service = this->nh.advertiseService(
    "/netft/set_max_values", &PulseNetftUtils::setMaxValues, this);
  this->theshold_service = this->nh.advertiseService(
    "/netft/set_threshold", &PulseNetftUtils::setThreshold, this);
  this->weight_bias_service = this->nh.advertiseService(
    "/netft/set_weight_bias", &PulseNetftUtils::setWeightBias, this);
  this->get_weight_service = this->nh.advertiseService(
    "/netft/get_weight", &PulseNetftUtils::getWeight, this);
  this->filter_service = this->nh.advertiseService(
    "/netft/set_filter", &PulseNetftUtils::setFilter, this);

  // this->update_timer = this->nh.createTimer(
  // ros::Duration(0.002), &PulseNetftUtils::updateCallback, this);

  // Subscribe to the topic of raw sensor data. Make sure this is at the end of
  // the constructor to ensure that the publishers and service servers are
  // initialized before the first call to the subscriber callback function.
  this->raw_data_sub = this->nh.subscribe(
    "/netft/raw_tool", 100, &PulseNetftUtils::netftCallback, this);
}

void PulseNetftUtils::updateCallback(const ros::TimerEvent& e)
{
  this->update();
}

void PulseNetftUtils::setUserInput(
  std::string world, std::string ft, double force, double torque)
{
  this->tf_world_frame = world;
  this->tf_tool_frame = ft;

  if(force != 0.0)
  {
    this->force_max_u = force;
  }
  else
  {
    // No operation
  }

  if(torque != 0.0)
  {
    this->torque_max_u = torque;
  }
  else
  {
    // No operation
  }
}

void PulseNetftUtils::update(void)
{
  // Look up transform from ft to world frame
  tf::StampedTransform tempTransform;

  // Check for a filter
  if(this->lpf_new)
  {
    delete this->lpf_h;

    this->lpf_h = new LPFilter(this->lpf_delta_t, this->lpf_cutoff_freq, 6);
    this->lpf_new = false;
  }
  else
  {
    // No operation
  }

  try
  {
    this->tf_listener->waitForTransform(
      this->tf_world_frame, this->tf_tool_frame, ros::Time(0),
      ros::Duration(1.0));
    this->tf_listener->lookupTransform(
      this->tf_world_frame, this->tf_tool_frame, ros::Time(0), tempTransform);
  }
  catch(tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
  }

  // Set translation to zero before updating value
  tempTransform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  this->tf_tool_to_world = tempTransform;

  this->checkForceLimitViolation();

  // Publish transformed dat
  this->netft_raw_world_data_pub.publish(raw_data_world);
  this->netft_proc_world_data_pub.publish(this->proc_data_world);
  this->netft_proc_tool_data_pub.publish(this->proc_data_tool);
  this->netft_cancel_pub.publish(this->cancel_msg);
}

void PulseNetftUtils::copyWrench(
  geometry_msgs::WrenchStamped& in, geometry_msgs::WrenchStamped& out,
  geometry_msgs::WrenchStamped& bias)
{
  out.header.stamp = in.header.stamp;
  out.header.frame_id = in.header.frame_id;
  out.wrench.force.x = in.wrench.force.x - bias.wrench.force.x;
  out.wrench.force.y = in.wrench.force.y - bias.wrench.force.y;
  out.wrench.force.z = in.wrench.force.z - bias.wrench.force.z;
  out.wrench.torque.x = in.wrench.torque.x - bias.wrench.torque.x;
  out.wrench.torque.y = in.wrench.torque.y - bias.wrench.torque.y;
  out.wrench.torque.z = in.wrench.torque.z - bias.wrench.torque.z;
}

void PulseNetftUtils::applyThresholdDouble(double& value, double thresh)
{
  if(value <= thresh && value >= -thresh)
  {
    value = 0.0;
  }
  else
  {
    // No operation
  }
}

void PulseNetftUtils::applyThresholdWrenchStamped(
  geometry_msgs::WrenchStamped& data, geometry_msgs::WrenchStamped& thresh)
{
  applyThresholdDouble(data.wrench.force.x, thresh.wrench.force.x);
  applyThresholdDouble(data.wrench.force.y, thresh.wrench.force.y);
  applyThresholdDouble(data.wrench.force.z, thresh.wrench.force.z);
  applyThresholdDouble(data.wrench.torque.x, thresh.wrench.torque.x);
  applyThresholdDouble(data.wrench.torque.y, thresh.wrench.torque.y);
  applyThresholdDouble(data.wrench.torque.z, thresh.wrench.torque.z);
}

void PulseNetftUtils::transformFrame(
  geometry_msgs::WrenchStamped in_data, geometry_msgs::WrenchStamped& out_data,
  char target_frame)
{
  tf::Vector3 f_tmp;
  tf::Vector3 t_tmp;
  f_tmp.setX(in_data.wrench.force.x);
  f_tmp.setY(in_data.wrench.force.y);
  f_tmp.setZ(in_data.wrench.force.z);
  t_tmp.setX(in_data.wrench.torque.x);
  t_tmp.setY(in_data.wrench.torque.y);
  t_tmp.setZ(in_data.wrench.torque.z);

  if(target_frame == 'w')
  {
    out_data.header.frame_id = this->tf_world_frame;
    f_tmp = this->tf_tool_to_world * f_tmp;
    t_tmp = this->tf_tool_to_world * t_tmp;
  }
  else if(target_frame == 't')
  {
    out_data.header.frame_id = this->tf_tool_frame;
    f_tmp = this->tf_tool_to_world.inverse() * f_tmp;
    t_tmp = this->tf_tool_to_world.inverse() * t_tmp;
  }

  out_data.header.stamp = in_data.header.stamp;
  out_data.wrench.force.x = f_tmp.getX();
  out_data.wrench.force.y = f_tmp.getY();
  out_data.wrench.force.z = f_tmp.getZ();
  out_data.wrench.torque.x = t_tmp.getX();
  out_data.wrench.torque.y = t_tmp.getY();
  out_data.wrench.torque.z = t_tmp.getZ();
}

/**
 *
 * \brief Callback function to run when a new raw data point is received
 *
 * \arg[in] data The message object containing the raw sensor data
 */
void PulseNetftUtils::netftCallback(
  const geometry_msgs::WrenchStamped::ConstPtr& data)
{
  // Filter data
  std::vector<double> data_tmp;
  data_tmp.resize(6);
  data_tmp.at(0) = -data->wrench.force.x;
  data_tmp.at(1) = data->wrench.force.y;
  data_tmp.at(2) = data->wrench.force.z;
  data_tmp.at(3) = -data->wrench.torque.x;
  data_tmp.at(4) = data->wrench.torque.y;
  data_tmp.at(5) = data->wrench.torque.z;

  if(this->lpf_is_on && !this->lpf_new)
  {
    this->lpf_h->update(data_tmp, data_tmp);
  }
  else
  {
    // No operation
  }

  // Copy tool frame data. Apply negative to x data to follow right hand rule
  // convention (raw sensor data does not)
  this->raw_data_tool.header.stamp = data->header.stamp;
  this->raw_data_tool.header.frame_id = this->tf_tool_frame;
  this->raw_data_tool.wrench.force.x = data_tmp.at(0);
  this->raw_data_tool.wrench.force.y = data_tmp.at(1);
  this->raw_data_tool.wrench.force.z = data_tmp.at(2);
  this->raw_data_tool.wrench.torque.x = data_tmp.at(3);
  this->raw_data_tool.wrench.torque.y = data_tmp.at(4);
  this->raw_data_tool.wrench.torque.z = data_tmp.at(5);

  // Transform tool frame data to world frame
  this->transformFrame(this->raw_data_tool, this->raw_data_world, 'w');

  // Apply the bias for a static sensor frame
  if(this->bias_is_on)
  {
    // Transform the bias from the tool frame to the world frame.
    geometry_msgs::WrenchStamped bias_world;
    this->transformFrame(this->bias_tool, bias_world, 'w');

    // Add the tool frame and world frame bias values to the corresponding data.
    copyWrench(this->raw_data_world, this->proc_data_world, bias_world);
    copyWrench(this->raw_data_tool, this->proc_data_tool, this->bias_tool);
  }
  else
  {
    // Just pass the data straight through
    copyWrench(this->raw_data_world, this->proc_data_world, this->zero_wrench);
    copyWrench(this->raw_data_tool, this->proc_data_tool, this->zero_wrench);
  }

  // The raw data variables are no longer helpful, so use the processed data
  // variables for the remainder of this function.
  if(this->grav_comp_is_on)
  {
    /* Compensate for gravity. Assumes world Z-axis is up.
     Gravity moment = (payload lever arm) cross (payload force)  <== all in the
     sensor frame. Need to convert to world later. Since it's assumed that the
     CoM of the payload is on the sensor's central axis, this calculation is
     simplified.
     This assumes that there is no friction between the end effector and contact
     surface. */
    double grav_moment_x =
      -this->grav_comp_payload_lever_arm * this->proc_data_tool.wrench.force.y;
    double grav_moment_y =
      this->grav_comp_payload_lever_arm * this->proc_data_tool.wrench.force.x;

    // Subtract the gravity torques from the previously-calculated wrench in the
    // tool frame.
    this->proc_data_tool.wrench.torque.x =
      this->proc_data_tool.wrench.torque.x - grav_moment_x;
    this->proc_data_tool.wrench.torque.y =
      this->proc_data_tool.wrench.torque.y - grav_moment_y;

    // Convert to world to account for the gravity force. Assumes the z-axis of
    // the world frame is up.
    this->transformFrame(this->proc_data_tool, this->proc_data_world, 'w');
    this->proc_data_world.wrench.force.z =
      this->proc_data_world.wrench.force.z - this->grav_comp_payload_weight;

    // this->proc_data_world now accounts for gravity completely. Convert back
    // to the tool frame to make that data available, too
    this->transformFrame(this->proc_data_world, this->proc_data_tool, 't');
  }
  else
  {
    // No operation
  }

  // Apply thresholds
  applyThresholdWrenchStamped(this->proc_data_world, this->threshold);
  applyThresholdWrenchStamped(this->proc_data_tool, this->threshold);
  this->update();
}

/**
 *
 * \brief Compute and store the sensor bias
 *
 * \details Set the readings from the sensor to zero at this instant and
 * continue to apply the bias on future readings. This doesn't account for
 * gravity i.e. it will not change if the sensor's orientation changes. Run
 * this method when the sensor is stationary to avoid inertial effects.
 *
 * \arg[in] req The request object containing the bias command
 * \arg[out] res The response object containing the status of the executed
 * function
 */
bool PulseNetftUtils::setFixedOrientationBias(
  netft_utils::SetBias::Request& req, netft_utils::SetBias::Response& res)
{
  if(req.toBias)
  {
    // Store the current wrench readings in the 'bias' variable, to be applied
    // hereafter.
    copyWrench(this->raw_data_tool, this->bias_tool, this->zero_wrench);

    // if forceMax was specified and > 0
    if(req.forceMax >= 0.0001)
    {
      this->force_max_b = req.forceMax;
    }
    else
    {
      // No operation
    }

    // if torqueMax was specified and > 0
    if(req.torqueMax >= 0.0001)
    {
      this->torque_max_b = req.torqueMax;
    }
    else
    {
      // No operation
    }

    this->bias_new = true;
    this->bias_is_on = true;
  }
  else
  {
    // Clear the stored bias if the argument was false
    copyWrench(this->zero_wrench, this->bias_tool, this->zero_wrench);
  }

  res.success = true;

  return true;
}

/**
 *
 * \brief Compute and store the payload weight and lever arm length
 *
 * \details Calculate the payload's mass and center of mass so gravity can be
 * compensated for, even as the sensor changes orientation. It is assumed that
 * the center of mass of the payload is located on the z-axis of the sensor
 * and the z-axis of the world frame points upwards. Run this method when the
 * sensor is stationary to avoid inertial effects. Also ensure that the
 * payload is horizontal for accurate weight computation.
 *
 * \arg[in] req The request object containing the gravity compensation command
 * \arg[out] res The response object containing the status of the executed
 * function
 */
bool PulseNetftUtils::setGravityCompensation(
  netft_utils::SetBias::Request& req, netft_utils::SetBias::Response& res)
{
  res.success = false;

  if(req.toBias)
  {
    if(this->bias_is_on)
    {
      ROS_ERROR("Cannot compensate for gravity if the sensor has already been "
                "biased, i.e. useful data was wiped out");
    }
    else
    {
      // Cannot compensate for gravity if the sensor has already been
      // biased, i.e. useful data was wiped out.
      // Get the weight of the payload. Assumes the world Z axis is up.
      this->grav_comp_payload_weight = raw_data_world.wrench.force.z;

      // Calculate the z-coordinate of the payload's center of mass, in the
      // sensor frame. It's assumed that the x- and y-coordinates of the payload
      // center of mass are zero. This is a lever arm.
      this->grav_comp_payload_lever_arm =
        raw_data_tool.wrench.torque.y / raw_data_tool.wrench.force.x;
      /* this->grav_comp_payload_lever_arm =
        sqrt(
          (raw_data_tool.wrench.torque.x * raw_data_tool.wrench.torque.x)
          + (raw_data_tool.wrench.torque.y * raw_data_tool.wrench.torque.y))
        / sqrt(
          (raw_data_tool.wrench.force.x * raw_data_tool.wrench.force.x)
          + (raw_data_tool.wrench.force.y * raw_data_tool.wrench.force.y)); */

      this->grav_comp_new = true;
      this->grav_comp_is_on = true;
      res.success = true;
    }
  }
  else
  {
    ROS_ERROR_STREAM("Service request does not include gravity compensation. "
                     "Set toBias flag to true.");
  }

  return res.success;
}

bool PulseNetftUtils::setFilter(
  netft_utils::SetFilter::Request& req, netft_utils::SetFilter::Response& res)
{
  if(req.toFilter)
  {
    this->lpf_new = true;
    this->lpf_is_on = true;
    this->lpf_delta_t = req.deltaT;
    this->lpf_cutoff_freq = req.cutoffFrequency;
  }
  else
  {
    this->lpf_is_on = false;
  }

  return true;
}

bool PulseNetftUtils::setMaxValues(
  netft_utils::SetMax::Request& req, netft_utils::SetMax::Response& res)
{
  if(req.forceMax >= 0.0001)
  {
    this->force_max_u = req.forceMax;
  }
  else
  {
    // No operation
  }

  if(req.torqueMax >= 0.0001)
  {
    this->torque_max_u = req.torqueMax;
  }
  else
  {
    // No operation
  }

  res.success = true;

  return true;
}

bool PulseNetftUtils::setWeightBias(
  netft_utils::SetBias::Request& req, netft_utils::SetBias::Response& res)
{
  if(req.toBias)
  {
    copyWrench(raw_data_tool, weight_bias, this->zero_wrench);
  }
  else
  {
    copyWrench(this->zero_wrench, weight_bias, this->zero_wrench);
  }

  res.success = true;

  return true;
}

bool PulseNetftUtils::getWeight(
  netft_utils::GetDouble::Request& req, netft_utils::GetDouble::Response& res)
{
  geometry_msgs::WrenchStamped carried_weight;
  copyWrench(raw_data_tool, carried_weight, weight_bias);
  res.weight = pow(
                 (pow(carried_weight.wrench.force.x, 2.0)
                  + pow(carried_weight.wrench.force.y, 2.0)
                  + pow(carried_weight.wrench.force.z, 2.0)),
                 0.5)
    / 9.81 * 1000;

  return true;
}

bool PulseNetftUtils::setThreshold(
  netft_utils::SetThreshold::Request& req,
  netft_utils::SetThreshold::Response& res)
{
  this->threshold.wrench.force.x = req.data.wrench.force.x;
  this->threshold.wrench.force.y = req.data.wrench.force.y;
  this->threshold.wrench.force.z = req.data.wrench.force.z;
  this->threshold.wrench.torque.x = req.data.wrench.torque.x;
  this->threshold.wrench.torque.y = req.data.wrench.torque.y;
  this->threshold.wrench.torque.z = req.data.wrench.torque.z;

  res.success = true;

  return true;
}

void PulseNetftUtils::checkForceLimitViolation(void)
{
  double f_mag = pow(
    (pow(this->proc_data_tool.wrench.force.x, 2.0)
     + pow(this->proc_data_tool.wrench.force.y, 2.0)
     + pow(this->proc_data_tool.wrench.force.z, 2.0)),
    0.5);
  double t_mag = pow(
    (pow(this->proc_data_tool.wrench.torque.x, 2.0)
     + pow(this->proc_data_tool.wrench.torque.y, 2.0)
     + pow(this->proc_data_tool.wrench.torque.z, 2.0)),
    0.5);
  double f_max;
  double t_max;

  if(this->bias_is_on)
  {
    if(this->bias_new)
    {
      f_max = this->force_max_u;
      t_max = this->torque_max_u;
      this->bias_new = false;
    }
    else
    {
      f_max = this->force_max_b;
      t_max = this->torque_max_b;
    }
  }
  else
  {
    f_max = this->force_max_u;
    t_max = this->torque_max_u;
  }

  if((fabs(f_mag) > f_max) || (fabs(t_mag) > t_max))
  {
    // Send a sequence of cancel messages, followed by a period of silence,
    // for as long as the force-torque limits are exceeded.
    if((this->cancel_count > 0) && (this->cancel_count <= CANCEL_COUNT_MAX))
    {
      this->cancel_msg.toCancel = true;
      this->cancel_count--;
      this->printForceLimitViolation(f_mag, f_max, t_mag, t_max);
    }
    else if((this->cancel_wait > 0) && (this->cancel_wait <= CANCEL_WAIT_MAX))
    {
      this->cancel_msg.toCancel = false;
      this->cancel_wait--;
      this->printForceLimitViolation(f_mag, f_max, t_mag, t_max);
    }
    else
    {
      this->cancel_msg.toCancel = false;
      this->cancel_count = CANCEL_COUNT_MAX;
      this->cancel_wait = CANCEL_WAIT_MAX;
    }
  }
  else
  {
    this->cancel_msg.toCancel = false;
    this->cancel_count = CANCEL_COUNT_MAX;
    this->cancel_wait = CANCEL_WAIT_MAX;
  }
}

void PulseNetftUtils::printForceLimitViolation(
  double f_mag, double f_max, double t_mag, double t_max)
{
  if(fabs(f_mag) > f_max)
  {
    ROS_ERROR(
      "|F| violation: %.6f > %.6f\nCancel (count, wait) = (%d, %d)", f_mag,
      f_max, this->cancel_count, this->cancel_wait);
  }
  else
  {
    ROS_ERROR(
      "|T| violation: %.6f > %.6f\nCancel (count, wait) = (%d, %d)", t_mag,
      t_max, this->cancel_count, this->cancel_wait);
  }
}
