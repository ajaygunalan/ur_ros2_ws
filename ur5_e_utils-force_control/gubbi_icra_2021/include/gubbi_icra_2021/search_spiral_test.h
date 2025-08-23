/**
 *
 * \file search_spiral_test.h
 *
 * \brief Test class for spiral search pattern
 *
 * \author Mardava Gubbi <mgubbi1@jhu.edu>
 *
 */
#ifndef PULSE_LAB_UR5_E_SEARCH_SPIRAL_TEST_H
#define PULSE_LAB_UR5_E_SEARCH_SPIRAL_TEST_H

#include "gubbi_icra_2021/search_spiral.h"
#include "pulse_lab_ur5_e_control_utils/SpiralSearchDebug.h"

class Ur5eSearchSpiralTestClass : public Ur5eSearchSpiralClass
{
protected:
  ros::Publisher goal_pub;

public:
  /**
   *
   * \brief Construct an object of the type Ur5eSearchSpiralTestClass
   *
   * \arg[in] nh The node handler
   * \arg[in] action_server_name The name of the action server to connect to
   * \arg[in] search_ns The namespace of the search parameters
   * \arg[in] path_planning_ns The namespace of the path planning parameters
   * \arg[in] end_effector_ns The namespace of the end effector parameters
   * \arg[in] joint_ns The namespace of the joint parameters
   *
   */
  Ur5eSearchSpiralTestClass(
    ros::NodeHandle& nh, std::string action_server_name,
    std::string search_ns = "/visual_servoing/search",
    std::string path_planning_ns = "/path_planning",
    std::string end_effector_ns = "/path_planning/end_effector",
    std::string joint_ns = "/path_planning/joint",
    std::string goal_topic_name = "/search_spiral/goal");

  /**
   *
   * \brief Plan the updated iteration of the search path.
   *
   * \return A flag indicating whether or not the path planning was successful
   *
   */
  bool planSearchPath(void);
};

#endif /* PULSE_LAB_UR5_E_SEARCH_SPIRAL_TEST_H */
