/*
  TODO copyright
*/

#ifndef CARTESIAN_CONTROLLER_POSE_MANAGER_H
#define CARTESIAN_CONTROLLER_POSE_MANAGER_H

#include <ros/ros.h>

#include <niryo_one_msgs/ManagePosition.h>
#include "geometry_msgs/Pose.h"

namespace cartesian_controller
{
class PoseManager
{
public:
  PoseManager();
  const std::vector<double> getJoints(const std::string position_name);
  const geometry_msgs::Pose getPose(const std::string position_name);
  void setJoints(const std::string position_name, const std::vector<double> joint_values);

  bool callbackManagePose(niryo_one_msgs::ManagePosition::Request& req, niryo_one_msgs::ManagePosition::Response& res);

protected:
private:
  void getPosition(std::string position_name, double (&result)[6]);

  ros::NodeHandle n_;

  std::map<std::string, std::vector<double>> position_map_;
};
}
#endif
