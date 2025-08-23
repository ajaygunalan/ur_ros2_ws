/**
 *
 * \file particle_filter_vel_noise_class.cpp
 *
 * \brief Particle filter described by Elfring et al. [1]
 *
 * \author Mardava Gubbi <mgubbi1@jhu.edu>
 *
 * \reference Elfring, et al., "Particle filters: A hands-on tutorial.", Sensors 21.2 (2021): 438.
 *
 */
#include "pulse_estimation_filters/particle_filter_vel_noise_class.hpp"

/*
 * \brief Construct an object of the type `ParticleFilterVelNoise`.
 */
pulse_est::ParticleFilterVelNoise::ParticleFilterVelNoise(ros::NodeHandle& nh) : EstimationFilter(nh)
{
  double particle_std_dev_val;
  int i0;
  std::string detection_topic_name;

  // No real justification for this, might need to play around with it to see what works. Just make sure it is a nice
  // round number like 512, 1024, etc. to simplify algorithms like binary search.
  this->number_of_particles = 2048;

  // Computed using a scan-converted FOV for 12 cm imaging depth.
  this->x_min = -85e-3;
  this->x_max = 85e-3;
  this->z_min = 5e-3;
  this->z_max = 120e-3;

  // Roughly thrice the transducer element height in total.
  this->y_min = -20e-3;
  this->y_max = 20e-3;

  // These values should eventually be sourced from standard deviations of either simulational or experimental
  // localization errors.
  // For now, assume a process noise of (10 / sqrt(3)) mm/s (i.e., the point source moves with a speed of around 1 cm/s).
  this->process_noise_std = 5.77e-3;
  this->measurement_noise_std = 4e-3;

  // Normally distributed random number generators as surrogates for particle velocity.
  this->x_proc_gen = new std::normal_distribution<double>(0.0, this->process_noise_std);
  this->y_proc_gen = new std::normal_distribution<double>(0.0, this->process_noise_std);
  this->z_proc_gen = new std::normal_distribution<double>(0.0, this->process_noise_std);

  // Uniformly distributed random number generator for particle resampling.
  this->cdf_rand_gen = new std::uniform_real_distribution<double>(0.0, 1.0);

  if(this->nh.hasParam("src_class_id"))
  {
    this->nh.getParam("src_class_id", this->src_class_id);
  }
  else
  {
    ROS_WARN("Could not fetch source class index, using default value...");
    this->src_class_id = 0;
  }

  if(this->nh.hasParam("detection_topic_name"))
  {
    this->nh.getParam("detection_topic_name", detection_topic_name);
  }
  else
  {
    ROS_WARN("Could not fetch name of topic containing point source detections, using default value...");
    detection_topic_name = "/pt_src_loc_sys_3d_a/detections";
  }

  for(i0 = 0; i0 < 3; i0++)
  {
    this->particle_cov_mat_inv(i0, i0) = 1.0 / (this->measurement_noise_std * this->measurement_noise_std);
  }

  // Perform the initial sampling of particles.
  this->initializeParticles();

  // Initialize publisher of estimated target position.
  this->tgt_pos_pub = this->nh.advertise<pulse_vs_msgs::KalmanArray>("target_pose", 1);

  // Initialize ROS publishers unique to the particle filter.
  this->weight_update_pub = this->nh.advertise<sensor_msgs::PointCloud>("weight_updated_point_cloud", 1);
  this->resampled_pub = this->nh.advertise<sensor_msgs::PointCloud>("resampled_point_cloud", 1);

  // Initialize ROS subscribers.
  this->detection_sub =
    this->nh.subscribe(detection_topic_name, 1, &ParticleFilterVelNoise::detectionSubscriberCallback, this);
}

/*
 * \brief Perform a binary search on the given array for the given value.
 */
int pulse_est::ParticleFilterVelNoise::binarySearch(std::vector<double> x_vec, double x_val)
{
  int left;
  int right;
  int bin_search_id = (x_vec.size() / 2) - 1;

  if(x_vec[0] < x_val && x_vec.back() > x_val)
  {
    left = 0;
    right = x_vec.size() - 1;
    bin_search_id = left + (right - left) / 2;

    while(left < right)
    {
      if(x_vec[bin_search_id] == x_val)
      {
        break;
      }
      else if(x_vec[bin_search_id] < x_val)
      {
        left = bin_search_id + 1;
      }
      else
      {
        right = bin_search_id;
      }

      bin_search_id = left + (right - left) / 2;
    }
  }
  else if(x_vec[0] >= x_val)
  {
    bin_search_id = 0;
  }
  else
  {
    bin_search_id = x_vec.size() - 1;
  }

  return bin_search_id;
}

/*
 * \brief Compute the effective sample size using Eq. (18) in [1].
 */
void pulse_est::ParticleFilterVelNoise::computeEffectiveSampleSize(void)
{
}

/*
 * \brief Update function for particle filter
 */
void pulse_est::ParticleFilterVelNoise::detectionSubscriberCallback(vision_msgs::Detection2DArray msg)
{
  Eigen::Isometry3d base_pose_probe_eig;
  Eigen::Isometry3d probe_pose_base_eig;
  geometry_msgs::TransformStamped base_pose_probe_msg;
  geometry_msgs::TransformStamped probe_pose_base_msg;

  try
  {
    // Fetch transforms back and forth between probe and base frames.
    base_pose_probe_msg = this->tf_buffer.lookupTransform(this->probe_frame_name, this->base_frame_name, ros::Time(0));
    base_pose_probe_eig = tf2::transformToEigen(base_pose_probe_msg);

    probe_pose_base_msg = this->tf_buffer.lookupTransform(this->base_frame_name, this->probe_frame_name, ros::Time(0));
    probe_pose_base_eig = tf2::transformToEigen(probe_pose_base_msg);

    // Propagate particle positions using model with noise for particle velocity.
    ROS_DEBUG("Propagating particle positions...");
    this->propagateParticlePositions();

    // Validate particle positions to ensure that no particle is outside the pre-defined transducer FoV.
    ROS_DEBUG("Validating particle positions...");
    this->validateParticlePositions(base_pose_probe_eig, probe_pose_base_eig);

    // Compute and publish the effective sample size.
    ROS_DEBUG("Computing effective sample size...");
    this->computeEffectiveSampleSize();

    // Update weight of each particle based on network detections (if available) and current particle positions.
    ROS_DEBUG("Updating particle weights...");
    this->updateParticleWeights(msg, probe_pose_base_eig);

    // TODO: Resample particles using updated particle weights.
    ROS_DEBUG("Resampling particles...");
    this->resampleParticles();
  }
  catch(tf2::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
  }
}

/*
 * \brief Initialize the sampling of particles using a uniform random distribution along each axis.
 */
void pulse_est::ParticleFilterVelNoise::initializeParticles(void)
{
  int i0;
  int num_tf_fetch_tries = 10;
  geometry_msgs::TransformStamped probe_pose_base_msg;
  Eigen::Isometry3d probe_pose_base_eig;

  // Try a fixed number of times to obtain the current transform from the probe frame to the base frame.
  for(i0 = 0; i0 < num_tf_fetch_tries; i0++)
  {
    try
    {
      ROS_DEBUG_STREAM(
        "Fetching transform from frame '" << this->probe_frame_name << "' to frame '" << this->base_frame_name
                                          << "'...");
      probe_pose_base_msg =
        this->tf_buffer.lookupTransform(this->base_frame_name, this->probe_frame_name, ros::Time(0));
      probe_pose_base_eig = tf2::transformToEigen(probe_pose_base_msg);
    }
    catch(tf2::TransformException& ex)
    {
      ROS_ERROR("%s", ex.what());

      // Allow for a limited number of failures to look up the transform from the probe frame to the base frame.
      if(i0 < num_tf_fetch_tries - 1)
      {
        ROS_INFO("Attempting again to fetch transformation from probe frame to base frame after a delay...");
        ros::Duration(1.0).sleep();
      }
      else
      {
        throw ex;
      }
    }
  }

  // Uniformly distributed random number generators for the uniform sampling process.
  std::uniform_real_distribution<double> x_rand_gen(this->x_min, this->x_max);
  std::uniform_real_distribution<double> y_rand_gen(this->y_min, this->y_max);
  std::uniform_real_distribution<double> z_rand_gen(this->z_min, this->z_max);

  // In the absence of any measurements, all particles are equally likely.
  this->particle_weights.resize(this->number_of_particles, 1.0 / (double)this->number_of_particles);

  // Uniformly distributed particle positions within the transducer FoV.
  this->particle_positions_mat = Eigen::MatrixXd::Zero(4, this->number_of_particles);

  for(i0 = 0; i0 < this->number_of_particles; i0++)
  {
    this->particle_positions_mat(0, i0) = x_rand_gen(this->rand_num_gen);
    this->particle_positions_mat(1, i0) = y_rand_gen(this->rand_num_gen);
    this->particle_positions_mat(2, i0) = z_rand_gen(this->rand_num_gen);
    this->particle_positions_mat(3, i0) = 1.0;
  }

  this->particle_positions_mat = probe_pose_base_eig.matrix() * this->particle_positions_mat;
  t_det_prev = ros::Time::now();
}

/*
 * \brief Propagate the motion of particles in the base frame using noise for velocity
 */
void pulse_est::ParticleFilterVelNoise::propagateParticlePositions(void)
{
  double delta_t;
  Eigen::Vector3d particle_velocity_vec;
  int i0;
  int i1;
  ros::Time t_det_curr;

  // Compute the time since the previous execution of the particle filter.
  t_det_curr = ros::Time::now();
  delta_t = t_det_curr.toSec() - this->t_det_prev.toSec();

  // Run the process model, propagating particle motion in the base frame.
  ROS_DEBUG("Propagating particles using particle motion model...");

  for(i0 = 0; i0 < this->number_of_particles; i0++)
  {
    particle_velocity_vec(0) = (*(this->x_proc_gen))(this->rand_num_gen);
    particle_velocity_vec(1) = (*(this->y_proc_gen))(this->rand_num_gen);
    particle_velocity_vec(2) = (*(this->z_proc_gen))(this->rand_num_gen);

    for(i1 = 0; i1 < 3; i1++)
    {
      this->particle_positions_mat(i1, i0) =
        this->particle_positions_mat(i1, i0) + (particle_velocity_vec(i1) * delta_t);
    }
  }

  // Update the variable containing the previous execution instant.
  this->t_det_prev = t_det_curr;
}

/*
 * \brief Resample particles using the previously computed posteriors
 */
void pulse_est::ParticleFilterVelNoise::resampleParticles(void)
{
  double cdf_val;
  Eigen::MatrixXd new_particle_positions = Eigen::MatrixXd::Zero(4, this->number_of_particles);
  Eigen::Vector3d pt_src_mean = Eigen::MatrixXd::Zero(3, 1);
  Eigen::Vector3d pt_src_abs_elev_mean = Eigen::MatrixXd::Zero(3, 1);
  geometry_msgs::Point32 pt_cloud_coordinates;
  pulse_vs_msgs::Kalman pose_probe_msg;
  pulse_vs_msgs::KalmanArray tgt_pose_probe_msg;
  int bin_search_id;
  int i0;
  sensor_msgs::PointCloud pt_cloud_msg;
  std::vector<double> cumulative_particle_weights(this->number_of_particles, 0.0);
  cumulative_particle_weights[0] = this->particle_weights[0];

  for(i0 = 1; i0 < this->number_of_particles; i0++)
  {
    cumulative_particle_weights[i0] = cumulative_particle_weights[i0 - 1] + this->particle_weights[i0];
  }

  // Confirm that the weights were properly normalized.
  // ROS_DEBUG("Cumulative weights range %.2e-%.2e", cumulative_particle_weights[0], cumulative_particle_weights.back());

  pt_cloud_msg.header.stamp = ros::Time::now();
  pt_cloud_msg.header.frame_id = this->base_frame_name;

  for(i0 = 0; i0 < this->number_of_particles; i0++)
  {
    // Generate a random number between zero and one.
    cdf_val = (*(this->cdf_rand_gen))(this->rand_num_gen);

    // Binary search for the closest CDF value to this number.
    bin_search_id = this->binarySearch(cumulative_particle_weights, cdf_val);

    // Select the position from `this->particle_positions_mat` corresponding to the fetched index as the current fresh
    // position sample.
    new_particle_positions.block<4, 1>(0, i0) = this->particle_positions_mat.block<4, 1>(0, bin_search_id);

    pt_cloud_coordinates.x = new_particle_positions(0, i0);
    pt_cloud_coordinates.y = new_particle_positions(1, i0);
    pt_cloud_coordinates.z = new_particle_positions(2, i0);
    pt_cloud_msg.points.push_back(pt_cloud_coordinates);

    pt_src_mean(0) += new_particle_positions(0, i0) / this->number_of_particles;
    pt_src_mean(1) += new_particle_positions(1, i0) / this->number_of_particles;
    pt_src_mean(2) += new_particle_positions(2, i0) / this->number_of_particles;
    pt_src_abs_elev_mean(0) += new_particle_positions(0, i0) / this->number_of_particles;
    pt_src_abs_elev_mean(1) += abs(new_particle_positions(1, i0)) / this->number_of_particles;
    pt_src_abs_elev_mean(2) += new_particle_positions(2, i0) / this->number_of_particles;
  }

  // Update the particle positions with the fresh samples.
  this->particle_positions_mat = new_particle_positions;

  ROS_DEBUG("Publishing resampled point cloud...");
  this->resampled_pub.publish(pt_cloud_msg);

  if((abs(pt_src_mean(1)) <= 1e-3) && pt_src_abs_elev_mean(1) > 1e-3)
  {
    pt_src_mean = pt_src_abs_elev_mean;
  }
  else
  {
    // No operation
  }

  tgt_pose_probe_msg.header.stamp = ros::Time::now();
  tgt_pose_probe_msg.header.frame_id = this->probe_frame_name;

  pose_probe_msg.kalman_state[0] = pt_src_mean(0);
  pose_probe_msg.kalman_state[1] = pt_src_mean(1);
  pose_probe_msg.kalman_state[2] = pt_src_mean(2);
  tgt_pose_probe_msg.filters.push_back(pose_probe_msg);

  ROS_DEBUG("Publishing estimated target pose...");
  this->tgt_pos_pub.publish(tgt_pose_probe_msg);
}

/*
 * \brief Update particle weights from the given measurements using a pre-determined algorithm
 */
void pulse_est::ParticleFilterVelNoise::updateParticleWeights(
  vision_msgs::Detection2DArray msg, Eigen::Isometry3d probe_pose_base_eig)
{
  // Scaling factor for each individual Gaussian in the Gaussian mixture model.
  // NOTE: We assume each individual Gaussian has the same covariance matrix.
  double gaussian_scaling = sqrt(this->particle_cov_mat_inv.determinant() / (8.0 * M_PI * M_PI * M_PI));

  // Sum of weights computed for normalization.
  double weight_sum = 0.0;
  double score_sum = 0.0;
  double exp_val;
  double particle_weight_sum = 0.0;

  // Positions of source detections and negative elevation counterparts in base frame.
  Eigen::MatrixXd det_pos_base_mat;

  // Error between positions of current detection and current particle inside loop.
  Eigen::Vector3d particle_det_error_vec;
  geometry_msgs::Point32 pt_cloud_coordinates;
  int i0;
  int i1;
  int i2;
  int num_src_det = 0;
  sensor_msgs::ChannelFloat32 probability_msg;
  sensor_msgs::PointCloud pt_cloud_msg;
  std::vector<double> particle_likelihoods(this->number_of_particles, 0.0);
  std::vector<double> particle_weights_new(this->number_of_particles, 0.0);
  std::vector<double> score_vec;
  std::vector<int> src_id_vec;

  pt_cloud_msg.header.stamp = ros::Time::now();
  pt_cloud_msg.header.frame_id = this->base_frame_name;
  probability_msg.name = "probabilities";

  // Extract source detections and negative elevation counterparts in the base frame into a matrix, along with the
  // indices of network outputs corresponding to the source class.
  this->extractSourcePositions(msg, probe_pose_base_eig, det_pos_base_mat, src_id_vec, num_src_det);

  // Extract confidence scores from the index vector obtained above.
  for(i0 = 0; i0 < src_id_vec.size(); i0++)
  {
    // Store the score corresponding to the source...
    score_vec.push_back(msg.detections[src_id_vec[i0]].results[0].score);

    // ... and the to negative elevation counterpart.
    score_vec.push_back(msg.detections[src_id_vec[i0]].results[0].score);

    // Add the scores for the source and the negative elevation counterpart to the running sum.
    score_sum += 2.0 * msg.detections[src_id_vec[i0]].results[0].score;
  }

  for(i0 = 0; i0 < num_src_det; i0++)
  {
    score_vec[i0] /= score_sum;
  }

  if(num_src_det > 0)
  {
    // Compute the likelihood of each particle based on the provided detections using a Gaussian mixture model.
    for(i0 = 0; i0 < this->number_of_particles; i0++)
    {
      for(i1 = 0; i1 < num_src_det; i1++)
      {
        particle_det_error_vec = this->particle_positions_mat.block<3, 1>(0, i0) - det_pos_base_mat.block<3, 1>(0, i1);
        exp_val = -0.5 * particle_det_error_vec.transpose() * this->particle_cov_mat_inv * particle_det_error_vec;
        particle_likelihoods[i0] += score_vec[i1] * gaussian_scaling * exp(exp_val);
      }

      particle_weights_new[i0] = this->particle_weights[i0] * particle_likelihoods[i0];
      particle_weight_sum += particle_weights_new[i0];
    }

    if(particle_weight_sum > PARTICLE_WEIGHT_SUM_THRESH)
    {
      for(i0 = 0; i0 < this->number_of_particles; i0++)
      {
        this->particle_weights[i0] = particle_weights_new[i0] / particle_weight_sum;
      }
    }
    else
    {
      ROS_WARN(
        "Sum of particle weights = %.2e < %.2e, resetting weights to %.2e...", particle_weight_sum,
        PARTICLE_WEIGHT_SUM_THRESH, 1.0 / ((double)this->number_of_particles));

      for(i0 = 0; i0 < this->number_of_particles; i0++)
      {
        this->particle_weights[i0] = 1.0 / ((double)this->number_of_particles);
      }
    }
  }
  else
  {
    ROS_WARN("No sources detected, retaining existing particle weights...");
  }

  for(i0 = 0; i0 < this->number_of_particles; i0++)
  {
    pt_cloud_coordinates.x = this->particle_positions_mat(0, i0);
    pt_cloud_coordinates.y = this->particle_positions_mat(1, i0);
    pt_cloud_coordinates.z = this->particle_positions_mat(2, i0);
    pt_cloud_msg.points.push_back(pt_cloud_coordinates);

    probability_msg.values.push_back(this->particle_weights[i0]);
  }

  pt_cloud_msg.channels.push_back(probability_msg);
  this->weight_update_pub.publish(pt_cloud_msg);
}

/*
 * \brief Cycle the particles within the current transducer FoV if required.
 */
void pulse_est::ParticleFilterVelNoise::validateParticlePositions(
  Eigen::Isometry3d base_pose_probe_eig, Eigen::Isometry3d probe_pose_base_eig)
{
  bool fix_required;
  Eigen::MatrixXd particle_positions_probe = base_pose_probe_eig.matrix() * this->particle_positions_mat;
  int i0;

  for(i0 = 0; i0 < this->number_of_particles; i0++)
  {
    fix_required = false;

    while(particle_positions_probe(0, i0) < this->x_min)
    {
      particle_positions_probe(0, i0) += (this->x_max - this->x_min);
      fix_required = true;
    }

    while(particle_positions_probe(0, i0) > this->x_max)
    {
      particle_positions_probe(0, i0) -= (this->x_max - this->x_min);
      fix_required = true;
    }

    while(particle_positions_probe(1, i0) < this->y_min)
    {
      particle_positions_probe(1, i0) += (this->y_max - this->y_min);
      fix_required = true;
    }

    while(particle_positions_probe(1, i0) > this->y_max)
    {
      particle_positions_probe(1, i0) -= (this->y_max - this->y_min);
      fix_required = true;
    }

    while(particle_positions_probe(2, i0) < this->z_min)
    {
      particle_positions_probe(2, i0) += (this->z_max - this->z_min);
      fix_required = true;
    }

    while(particle_positions_probe(2, i0) > this->z_max)
    {
      particle_positions_probe(2, i0) -= (this->z_max - this->z_min);
      fix_required = true;
    }

    if(fix_required)
    {
      this->particle_positions_mat.block<4, 1>(0, i0) =
        probe_pose_base_eig.matrix() * particle_positions_probe.block<4, 1>(0, i0);
    }
  }
}
