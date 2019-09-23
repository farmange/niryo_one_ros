/*
  TODO copyright
*/

#ifndef CARTESIAN_CONTROLLER_TRAJECTORY_MANAGER_H
#define CARTESIAN_CONTROLLER_TRAJECTORY_MANAGER_H

#include <ros/ros.h>

namespace cartesian_controller
{
class TrajectoryManager
{
public:
  TrajectoryManager();

protected:
  ros::NodeHandle n_;
};
}

#endif
