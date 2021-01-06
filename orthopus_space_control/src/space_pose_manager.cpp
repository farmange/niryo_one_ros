/*
 *  space_pose_manager.cpp
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
#include "ros/ros.h"

#include "orthopus_space_control/space_pose_manager.h"

// Eigen
#include "eigen_conversions/eigen_msg.h"

namespace space_control
{
SpacePoseManager::SpacePoseManager(const int joint_number) : joint_number_(joint_number)
{
  SpacePosition x_pose;
  ros::param::get("~drink_pose/position/x", x_pose.position.x());
  ros::param::get("~drink_pose/position/y", x_pose.position.y());
  ros::param::get("~drink_pose/position/z", x_pose.position.z());
  ros::param::get("~drink_pose/orientation/w", x_pose.orientation.w());
  ros::param::get("~drink_pose/orientation/x", x_pose.orientation.x());
  ros::param::get("~drink_pose/orientation/y", x_pose.orientation.y());
  ros::param::get("~drink_pose/orientation/z", x_pose.orientation.z());
  q_saved_pose_.emplace("Home", x_pose);

  ros::param::get("~stand_pose/position/x", x_pose.position.x());
  ros::param::get("~stand_pose/position/y", x_pose.position.y());
  ros::param::get("~stand_pose/position/z", x_pose.position.z());
  ros::param::get("~stand_pose/orientation/w", x_pose.orientation.w());
  ros::param::get("~stand_pose/orientation/x", x_pose.orientation.x());
  ros::param::get("~stand_pose/orientation/y", x_pose.orientation.y());
  ros::param::get("~stand_pose/orientation/z", x_pose.orientation.z());
  q_saved_pose_.emplace("Stand", x_pose);
}

const SpacePosition SpacePoseManager::getSpacePose(const std::string position_name)
{
  return q_saved_pose_.at(position_name);
}

void SpacePoseManager::setSpacePose(const std::string position_name, const SpacePosition q_pose_to_record)
{
  /* Remove entry if exists */
  if (q_saved_pose_.find(position_name) != q_saved_pose_.end())
  {
    q_saved_pose_.erase(position_name);
  }
  q_saved_pose_.emplace(position_name, q_pose_to_record);
}

bool SpacePoseManager::callbackManagePose(niryo_one_msgs::ManagePosition::Request& req,
                                          niryo_one_msgs::ManagePosition::Response& res)
{
  if (req.cmd_type == 0)
  {
    /* Set position */
    if (req.position.joints.size() != joint_number_)
    {
      res.message = "Error, could not set joint position (joint vector size mismatch)";
      ROS_ERROR("Error, could not set joint position (size != %d) ", joint_number_);
      return false;
    }
    res.message = "Set position " + req.position_name;

    SpacePosition q;
    for (int i = 0; i < joint_number_; i++)
    {
      q[i] = req.position.joints[i];
    }
    q_saved_pose_.emplace(req.position_name, q);
  }
  else if (req.cmd_type == 1)
  {
    // TODO get the current joint state and save it
  }
  return true;
}
}
