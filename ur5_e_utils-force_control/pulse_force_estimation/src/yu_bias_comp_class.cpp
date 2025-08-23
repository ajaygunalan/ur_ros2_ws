/**
 *
 * \file yu_bias_comp_class.cpp
 *
 * \brief Perform sensor bias and gravity compensation for the force-torque
 * sensor based on the method described by Yu et al. [1].
 *
 * \author Mardava Gubbi <mgubbi1@jh.edu>
 */
#include "pulse_force_estimation/yu_bias_comp_class.hpp"

/*
 * \brief Constructor for YuBiasCompClass
 */
YuBiasCompClass::YuBiasCompClass(
  ros::NodeHandle& nh, std::string base_frame_name, std::string ee_frame_name, std::string sensor_frame_name,
  std::string probe_frame_name)
  : nh(nh)
  , base_frame_name(base_frame_name)
  , ee_frame_name(ee_frame_name)
  , sensor_frame_name(sensor_frame_name)
  , probe_frame_name(probe_frame_name)
{
  // Transformations from probe frame `P` to sensor frame `S`.
  tf::StampedTransform st_s_p;
  Eigen::Isometry3d eig_s_p;
  Eigen::MatrixXd mat_s_p;

  // Rotation from sensor frame `S` to probe frame `P`.
  Eigen::MatrixXd rot_p_s;

  std::string bias_values_file_name;
  int i0;

  this->f_grav_b.setZero();
  this->f_bias_s.setZero();
  this->t_bias_s.setZero();
  this->p_grav_s_hat.setZero();
  this->rot_s_e.setIdentity();

  if(this->nh.hasParam("bias_values_file"))
  {
    this->nh.getParam("bias_values_file", bias_values_file_name);

    if(!bias_values_file_name.empty())
    {
      boost::filesystem::path bias_values_file_path(bias_values_file_name);

      if(boost::filesystem::is_regular_file(bias_values_file_path))
      {
        ROS_INFO("Updating bias and gravity parameters from file '%s'...", bias_values_file_path.string().c_str());
        HighFive::File bias_values_fh(bias_values_file_path.string(), HighFive::File::ReadOnly);

        std::vector<double> f_grav_b_vec;
        std::vector<double> f_bias_s_vec;
        std::vector<double> t_bias_s_vec;
        std::vector<double> p_grav_s_vec;
        std::vector<double> q_s_e_vec;
        Eigen::Vector3d p_grav_s;

        bias_values_fh.getDataSet("/f_grav_b").read(f_grav_b_vec);
        bias_values_fh.getDataSet("/f_bias_s").read(f_bias_s_vec);
        bias_values_fh.getDataSet("/t_bias_s").read(t_bias_s_vec);
        bias_values_fh.getDataSet("/p_grav_s").read(p_grav_s_vec);
        bias_values_fh.getDataSet("/q_s_e").read(q_s_e_vec);

        for(i0 = 0; i0 < 3; i0++)
        {
          this->f_grav_b(i0) = f_grav_b_vec[i0];
          this->f_bias_s(i0) = f_bias_s_vec[i0];
          this->t_bias_s(i0) = t_bias_s_vec[i0];
          p_grav_s(i0) = p_grav_s_vec[i0];
        }

        this->p_grav_s_hat = hatOperator(p_grav_s);
        Eigen::Quaterniond q_s_e;
        q_s_e.x() = q_s_e_vec[0];
        q_s_e.y() = q_s_e_vec[1];
        q_s_e.z() = q_s_e_vec[2];
        q_s_e.w() = q_s_e_vec[3];
        this->rot_s_e = q_s_e.normalized().toRotationMatrix();
      }
      else
      {
        ROS_WARN("Could not find bias values file '%s'.", bias_values_file_path.string().c_str());
        throw std::runtime_error("Could not find bias values file.");
      }
    }
    else
    {
      ROS_WARN("Received empty field for bias values file name.");
    }
  }
  else
  {
    ROS_WARN("No existing bias values file name provided.");
  }

  ROS_DEBUG("Initializing TF listener...");
  this->tf_listener = new tf::TransformListener();
  this->ft_adjoint_p_s.setZero();

  // Set the adjoint transformation matrix for force-torque readings from the sensor frame `S` to the probe frame `P`.
  // To do so, you need a transform listener to fetch the (constant) transform from frame `P` to frame `S`,
  try
  {
    ROS_DEBUG_STREAM(
      ""
      << "Waiting for transform from `" << this->probe_frame_name << "` to `" << this->sensor_frame_name << "`...");
    this->tf_listener->waitForTransform(
      this->sensor_frame_name, this->probe_frame_name, ros::Time(0), ros::Duration(10.0));

    ROS_DEBUG_STREAM(
      ""
      << "Looking up transform from `" << this->probe_frame_name << "` to `" << this->sensor_frame_name << "`...");
    this->tf_listener->lookupTransform(this->sensor_frame_name, this->probe_frame_name, ros::Time(0), st_s_p);

    // the rotational component of the transformation from `S` to `P`,
    tf::transformTFToEigen(st_s_p, eig_s_p);
    mat_s_p = eig_s_p.matrix();
    rot_p_s = mat_s_p.block(0, 0, 3, 3).transpose();
    this->ft_adjoint_p_s.block(0, 0, 3, 3) = rot_p_s;
    this->ft_adjoint_p_s.block(3, 3, 3, 3) = rot_p_s;

    // and the translational component of the transformation from `P` to `S`.
    // See Equ. (2.66) in [2] for more information.
    this->ft_adjoint_p_s.block(3, 0, 3, 3) = -rot_p_s * hatOperator(mat_s_p.block(0, 3, 3, 1));

    ROS_DEBUG_STREAM("Computed adjoint matrix:" << std::endl << this->ft_adjoint_p_s);
  }
  catch(tf::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
    throw ex;
  }

  ROS_DEBUG("Initializing publishers...");
  this->ft_proc_sensor_pub = this->nh.advertise<geometry_msgs::WrenchStamped>("/netft/proc_sensor", 1);
  this->ft_proc_probe_pub = this->nh.advertise<geometry_msgs::WrenchStamped>("/netft/proc_probe", 1);
  this->ft_raw_force_norm_pub = this->nh.advertise<std_msgs::Float64>("/netft/raw_sensor_force_norm", 1);
  this->ft_raw_torque_norm_pub = this->nh.advertise<std_msgs::Float64>("/netft/raw_sensor_torque_norm", 1);
  this->ft_cancel_pub = this->nh.advertise<netft_utils::Cancel>("/netft/cancel", 1);
  this->bias_update_server = this->nh.advertiseService("/netft/yu_set_biases", &YuBiasCompClass::setBiases, this);

  ROS_DEBUG("Initialize execution time estimator...");
  this->bias_comp_exe_timer = new et::PulseExecutionTimer(nh);

  ROS_DEBUG("Initializing subscriber...");
  this->ft_raw_sensor_sub =
    this->nh.subscribe("/netft/raw_sensor", 1, &YuBiasCompClass::rawSensorSubscriberCallback, this);
}

/*
 * \brief Destructor for YuBiasCompClass
 */
YuBiasCompClass::~YuBiasCompClass(void)
{
}

/*
 * \brief Set or update the bias and gravity compensation parameters
 */
bool YuBiasCompClass::setBiases(
  pulse_force_estimation::YuSetBiases::Request& req, pulse_force_estimation::YuSetBiases::Response& res)
{
  Eigen::Quaterniond q_s_e;
  Eigen::Quaterniond q_g_b;
  Eigen::Vector3d p_grav_s_vec;

  ROS_INFO("Updating bias and gravity compensation parameters:");
  ROS_INFO("Gravity F_b: [% .3f, % .3f, % .3f]", req.f_grav_b.x, req.f_grav_b.y, req.f_grav_b.z);
  ROS_INFO("Force bias ^{s}F_0: [% .3f, % .3f, % .3f]", req.f_bias_s.x, req.f_bias_s.y, req.f_bias_s.z);
  ROS_INFO("Torque bias ^{s}T_0: [% .3f, % .3f, % .3f]", req.t_bias_s.x, req.t_bias_s.y, req.t_bias_s.z);
  tf::vectorMsgToEigen(req.f_grav_b, this->f_grav_b);
  tf::vectorMsgToEigen(req.f_bias_s, this->f_bias_s);
  tf::vectorMsgToEigen(req.t_bias_s, this->t_bias_s);

  // Store the center of gravity of the tool in the sensor frame `S` as a
  // hat-matrix rather than a vector, as the gravity compensation expression
  // involves a cross product.
  ROS_INFO("Tool grav center ^{s}P_g: [% .3f, % .3f, % .3f]", req.p_grav_s.x, req.p_grav_s.y, req.p_grav_s.z);
  tf::vectorMsgToEigen(req.p_grav_s, p_grav_s_vec);
  this->p_grav_s_hat = hatOperator(p_grav_s_vec);

  // Update constant rotation matrices from robot end effector frame `E` to
  // sensor frame `S` and from gravity frame `G` to robot base frame `B`.
  ROS_INFO("Quaternion ^{s}q_e: [% .3f, % .3f, % .3f, % .3f]", req.q_s_e.x, req.q_s_e.y, req.q_s_e.z, req.q_s_e.w);
  tf::quaternionMsgToEigen(req.q_s_e, q_s_e);
  this->rot_s_e = q_s_e.normalized().toRotationMatrix();

  res.success = true;

  return res.success;
}

/*
 * \brief Callback function to run when a new raw data point is received
 */
void YuBiasCompClass::rawSensorSubscriberCallback(const geometry_msgs::WrenchStamped& raw_msg)
{
  // Store the start time of callback function execution.
  this->bias_comp_exe_timer->start();

  // Extract the raw force and torque vectors in the sensor frame `S` from the input message.
  Eigen::Vector3d f_raw_s;
  tf::vectorMsgToEigen(raw_msg.wrench.force, f_raw_s);

  Eigen::Vector3d t_raw_s;
  tf::vectorMsgToEigen(raw_msg.wrench.torque, t_raw_s);

  // Fetch the instantaneous transform from the robot base frame `B` to the
  // robot end effector frame `E`.
  tf::StampedTransform transform_e_b;

  try
  {
    this->tf_listener->lookupTransform(this->ee_frame_name, this->base_frame_name, ros::Time(0), transform_e_b);
    tf::transformTFToEigen(transform_e_b, this->t_e_b);
  }
  catch(tf::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
  }

  // Compute the gravitational force due to the probe weight in the sensor frame
  // `S` (see Equ. (10) in [1]).
  Eigen::Vector3d f_grav_s = this->rot_s_e * this->t_e_b.matrix().block(0, 0, 3, 3) * this->f_grav_b;

  // Compute the bias and gravity compensated force-torque wrench in the sensor
  // frame `S` (see Equs. (10) and (11), respectively, in [1]).
  Eigen::Matrix<double, 6, 1> ft_proc_s;
  ft_proc_s.block(0, 0, 3, 1) = f_raw_s - f_grav_s - this->f_bias_s;
  ft_proc_s.block(3, 0, 3, 1) = t_raw_s - (this->p_grav_s_hat * f_grav_s) - this->t_bias_s;

  // Construct and publish standard and debug messages on corresponding topics.
  //
  // Message containing bias and gravity compensated force-torque wrench in
  // probe frame `P` to be published to downstream nodes.
  geometry_msgs::WrenchStamped proc_msg;
  proc_msg.header.stamp = ros::Time::now();
  proc_msg.header.frame_id = this->sensor_frame_name;
  tf::wrenchEigenToMsg(ft_proc_s, proc_msg.wrench);
  this->ft_proc_sensor_pub.publish(proc_msg);

  // Debug messages containing norms of force and torque vectors to determine
  // effectiveness of bias compensation in the absence of contact.
  std_msgs::Float64 raw_tool_force_norm_msg;
  raw_tool_force_norm_msg.data = sqrt(
    (raw_msg.wrench.force.x * raw_msg.wrench.force.x) + (raw_msg.wrench.force.y * raw_msg.wrench.force.y)
    + (raw_msg.wrench.force.z * raw_msg.wrench.force.z));
  this->ft_raw_force_norm_pub.publish(raw_tool_force_norm_msg);

  std_msgs::Float64 raw_tool_torque_norm_msg;
  raw_tool_torque_norm_msg.data = sqrt(
    (raw_msg.wrench.torque.x * raw_msg.wrench.torque.x) + (raw_msg.wrench.torque.y * raw_msg.wrench.torque.y)
    + (raw_msg.wrench.torque.z * raw_msg.wrench.torque.z));
  this->ft_raw_torque_norm_pub.publish(raw_tool_torque_norm_msg);

  // Transform the processed F/T readings from the sensor frame `S` to the probe
  // frame `P`.

  Eigen::Matrix<double, 6, 1> ft_proc_p = this->ft_adjoint_p_s * ft_proc_s;
  proc_msg.header.frame_id = this->probe_frame_name;
  tf::wrenchEigenToMsg(ft_proc_p, proc_msg.wrench);
  this->ft_proc_probe_pub.publish(proc_msg);

  // Store the end time and compute the execution duration of the callback function.
  this->bias_comp_exe_timer->end();
}
