/**
 *
 * \file search_spiral.h
 *
 * \brief Class for spiral search pattern
 *
 * \author Mardava Gubbi <mgubbi1@jhu.edu>
 *
 */
#ifndef PULSE_LAB_UR5_E_SEARCH_SPIRAL_H
#define PULSE_LAB_UR5_E_SEARCH_SPIRAL_H

#include "gubbi_icra_2021/search_class.h"

class Ur5eSearchSpiralClass : public Ur5eSearchClass
{
public:
  /**
   *
   * \brief Construct an object of the type Ur5eSearchSpiralClass
   *
   * \arg[in] nh The node handler
   * \arg[in] action_server_name The name of the action server to connect to
   * \arg[in] search_ns The namespace of the search parameters
   * \arg[in] path_planning_ns The namespace of the path planning parameters
   * \arg[in] end_effector_ns The namespace of the end effector parameters
   * \arg[in] joint_ns The namespace of the joint parameters
   *
   */
  Ur5eSearchSpiralClass(
    ros::NodeHandle& nh, std::string action_server_name,
    std::string search_ns = "/visual_servoing/search",
    std::string path_planning_ns = "/path_planning",
    std::string end_effector_ns = "/path_planning/end_effector",
    std::string joint_ns = "/path_planning/joint");

  /**
   *
   * \brief Plan the updated iteration of the search path.
   *
   * \return A flag indicating whether or not the path planning was successful
   *
   */
  bool planSearchPath(void);
};

#endif /* PULSE_LAB_UR5_E_SEARCH_SPIRAL_H */
