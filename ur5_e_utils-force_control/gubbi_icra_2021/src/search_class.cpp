/**
 *
 * \file search_class.cpp
 *
 * \brief Base class for search pattern
 *
 * \author Mardava Gubbi <mgubbi1@jhu.edu>
 *
 */
#include "gubbi_icra_2021/search_class.h"

/*
 * \brief Construct an object of the type Ur5eSearchClass.
 */
Ur5eSearchClass::Ur5eSearchClass(
  ros::NodeHandle& nh, std::string action_server_name, std::string search_ns,
  std::string path_planning_ns, std::string end_effector_ns,
  std::string joint_ns)
  : Ur5eActionClientLinClass(
    nh, action_server_name, path_planning_ns, end_effector_ns, joint_ns)
{
  std::string search_path_planning_total_t_param_name = search_ns + "/total_t";

  this->search_iteration = 0;

  if(!this->nh.getParam(
       search_path_planning_total_t_param_name, this->path_planning_total_t))
  {
    ROS_WARN_STREAM(
      ""
      << "Could not find parameter '" << search_path_planning_total_t_param_name
      << "', using default value...");
    this->path_planning_total_t = 0.1;
  }
  else
  {
    // No operation
  }
}

/*
 * \brief Reset the search parameters.
 */
void Ur5eSearchClass::resetSearchParameters(void)
{
  this->search_iteration = 0;
}
