/*
 * Copyright (c) 2014, 2015, 2016, Charles River Analytics, Inc.
 * All rights reserved.
 */

#include "cra_robot_localization/ros_filter_types.h"

#include <ros/ros.h>

#include <vector>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ukf_navigation_node");
  ros::NodeHandle nhLocal("~");

  std::vector<double> args(3, 0);

  nhLocal.param("alpha", args[0], 0.001);
  nhLocal.param("kappa", args[1], 0.0);
  nhLocal.param("beta", args[2], 2.0);

  RobotLocalization::RosUkf ukf(args);

  ukf.run();

  return 0;
}
