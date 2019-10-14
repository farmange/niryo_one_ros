/*
 *  pose_manager.cpp
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

#include "orthopus_space_control/pose_manager.h"

// MoveIt!
#include "moveit/robot_model/robot_model.h"
#include "moveit/robot_model_loader/robot_model_loader.h"
#include "moveit/robot_state/robot_state.h"

// Eigen
#include "eigen_conversions/eigen_msg.h"

namespace space_control
{
PoseManager::PoseManager(const int joint_number, const bool use_quaternion)
  : joint_number_(joint_number), use_quaternion_(use_quaternion)
{
  ROS_DEBUG_STREAM("PoseManager constructor");
  JointPosition q(joint_number);
  ros::param::get("~home_position", q);
  q_saved_pose_.emplace("Home", q);
  ros::param::get("~rest_position", q);
  q_saved_pose_.emplace("Rest", q);
}

const JointPosition PoseManager::getJoints(const std::string position_name)
{
  return q_saved_pose_.at(position_name);
}

void PoseManager::setJoints(const std::string position_name, const JointPosition q_pose_to_record)
{
  /* Remove entry if exists */
  if (q_saved_pose_.find(position_name) != q_saved_pose_.end())
  {
    q_saved_pose_.erase(position_name);
  }
  q_saved_pose_.emplace(position_name, q_pose_to_record);
}

bool PoseManager::callbackManagePose(niryo_one_msgs::ManagePosition::Request& req,
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

    JointPosition q(joint_number_);
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
