/**
 *
 * \file pulse_netft_utils.cpp
 *
 * \brief Test the class PulseNetftUtils.
 *
 * \author Mardava Gubbi <mgubbi1@jhu.edu>
 */
#include "pulse_force_estimation/pulse_netft_utils.hpp"

int main(int argc, char** argv)
{
  // Initialize the ros netft_utils_node
  ros::init(argc, argv, "netft_utils_node");

  // Instantiate utils class
  ros::NodeHandle n;
  PulseNetftUtils utils(n);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Initialize utils
  utils.initialize();

  // Set up user input
  std::string world_frame;
  std::string ft_frame;
  double forceMaxU = 0.0;
  double torqueMaxU = 0.0;

  if(argc < 3)
  {
    ROS_FATAL("You must pass in at least the world and ft frame as command "
              "line arguments. Argument options are [world frame, ft frame, "
              "max force, max torque]");
    return 1;
  }
  else if(argc >= 6)
  {
    ROS_FATAL("Too many arguments for netft_utils");
  }
  else
  {
    world_frame = argv[1];
    ft_frame = argv[2];

    if(argc >= 4)
    {
      forceMaxU = atof(argv[3]);
    }
    else
    {
      // No operation
    }

    if(5 == argc)
    {
      torqueMaxU = atof(argv[4]);
    }
    else
    {
      // No operation
    }
  }

  utils.setUserInput(world_frame, ft_frame, forceMaxU, torqueMaxU);
  ros::ServiceClient bias_client =
    n.serviceClient<netft_utils::SetBias>("/netft/bias");
  netft_utils::SetBias srv;
  srv.request.toBias = true;

  ROS_INFO("Sleeping prior to bias computation...");
  ros::Duration(10.0).sleep();

  if(bias_client.call(srv))
  {
    ROS_INFO("Successfully initiated bias.");
  }
  else
  {
    ROS_ERROR("Bias initiation failed.");
  }

  ros::waitForShutdown();
  return 0;
}
