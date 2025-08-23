/**
 *
 * \file search_class.h
 *
 * \brief Base class for search pattern
 *
 * \author Mardava Gubbi <mgubbi1@jhu.edu>
 *
 */
#ifndef PULSE_LAB_UR5_E_SEARCH_CLASS_H
#define PULSE_LAB_UR5_E_SEARCH_CLASS_H

#include "pulse_control_utils/ur5e_base_class.hpp"

class Ur5eSearchClass : public Ur5eBaseClass
{
protected:
  /*
   * \brief Iteration of search path
   */
  int search_iteration;

public:
  /**
   *
   * \brief Construct an object of the type Ur5eSearchClass.
   *
   * \arg[in] nh The node handler
   * \arg[in] action_server_name The name of the action server to connect to
   * \arg[in] search_ns The namespace of the search parameters
   * \arg[in] path_planning_ns The namespace of the path planning parameters
   * \arg[in] end_effector_ns The namespace of the end effector parameters
   * \arg[in] joint_ns The namespace of the joint parameters
   *
   */
  Ur5eSearchClass(
    ros::NodeHandle& nh, std::string action_server_name,
    std::string search_ns = "/visual_servoing/search",
    std::string path_planning_ns = "/path_planning",
    std::string end_effector_ns = "/path_planning/end_effector",
    std::string joint_ns = "/path_planning/joint");

  /**
   *
   * \brief Reset the search parameters.
   *
   */
  void resetSearchParameters(void);

  /**
   *
   * \brief Plan the updated iteration of the search path.
   *
   * \return A flag indicating whether or not the path planning was successful
   *
   */
  virtual bool planSearchPath(void) = 0;
};

#endif /* PULSE_LAB_UR5_E_SEARCH_CLASS_H */
