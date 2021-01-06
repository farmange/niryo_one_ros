/*
 *  pose_manager.h
 *  Copyright (C) 2019 Orthopus
 *  All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef CARTESIAN_CONTROLLER_POSE_MANAGER_H
#define CARTESIAN_CONTROLLER_POSE_MANAGER_H

#include "ros/ros.h"

#include "geometry_msgs/Pose.h"
#include "niryo_one_msgs/ManagePosition.h"

#include "orthopus_space_control/types/joint_position.h"

namespace space_control
{
class PoseManager
{
public:
  PoseManager(const int joint_number);
  const JointPosition getJoints(const std::string position_name);
  void setJoints(const std::string position_name, const JointPosition q_pose_to_record);

  bool callbackManagePose(niryo_one_msgs::ManagePosition::Request& req, niryo_one_msgs::ManagePosition::Response& res);

protected:
private:
  ros::NodeHandle n_;

  int joint_number_;

  std::map<std::string, JointPosition> q_saved_pose_;
};
}
#endif
