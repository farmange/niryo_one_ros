/*
 *  joint_pose_manager.cpp
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

#include "orthopus_space_control/joint_pose_manager.h"

// Eigen
#include "eigen_conversions/eigen_msg.h"

namespace space_control
{
JointPoseManager::JointPoseManager(const int joint_number) : joint_number_(joint_number), csv_m_("joint_pose_db.csv")
{
  ROS_DEBUG_STREAM("JointPoseManager constructor");
  JointPosition q(joint_number);
  ros::param::get("~home_position", q);
  q_saved_pose_.emplace("Home", q);
  ros::param::get("~rest_position", q);
  q_saved_pose_.emplace("Rest", q);

  loadJointPoseFromCSV();
}

void JointPoseManager::loadJointPoseFromCSV()
{
  std::vector<std::vector<std::string>> records;
  JointPosition temp_pose(6);

  csv_m_.get_records(records);

  for (int i = 0; i < records.size(); i++)
  {
    if (records[i].size() == 7)
    {
      for (int j = 1; j < records[i].size(); j++)
      {
        temp_pose[j - 1] = stod(records[i][j]);
      }
    }
    q_saved_pose_.emplace(records[i][0], temp_pose);
  }
}

void JointPoseManager::saveJointPoseToCSV()
{
  std::map<std::string, JointPosition>::iterator itr;

  csv_m_.clear_records();

  for (itr = q_saved_pose_.begin(); itr != q_saved_pose_.end(); ++itr)
  {
    if (itr->first != "Rest" && itr->first != "Home")
    {
      std::vector<std::string> record;
      record.push_back(itr->first);

      for (int i = 0; i < itr->second.size(); i++)
      {
        std::stringstream s;
        s << itr->second.at(i);
        record.push_back(s.str());
      }
      csv_m_.add_record(record);
    }
  }
}

const JointPosition JointPoseManager::getJointPosition(const std::string position_name)
{
  return q_saved_pose_.at(position_name);
}

void JointPoseManager::getJointPositionList(std::vector<niryo_one_msgs::Position> &position_list)
{
  niryo_one_msgs::Position position_temp;
  std::map<std::string, JointPosition>::iterator itr;
  position_list.clear();
  for (itr = q_saved_pose_.begin(); itr != q_saved_pose_.end(); ++itr)
  {
    position_temp.name = itr->first;
    position_temp.joints = itr->second;
    position_list.push_back(position_temp);
  }
}

bool JointPoseManager::setJointPosition(const std::string position_name, const JointPosition q_pose_to_record)
{
  /* Cannot modify Rest and Home position */
  if (position_name != "Rest" && position_name != "Home")
  {
    /* Remove entry if exists */
    if (q_saved_pose_.find(position_name) == q_saved_pose_.end())
    {
      q_saved_pose_.emplace(position_name, q_pose_to_record);
      saveJointPoseToCSV();
      return true;
    }
  }
  return false;
}

bool JointPoseManager::updateJointPosition(const std::string position_name, const JointPosition q_pose_to_record)
{
  /* Cannot modify Rest and Home position */
  if (position_name != "Rest" && position_name != "Home")
  {
    /* Update entry if exists */
    if (q_saved_pose_.find(position_name) != q_saved_pose_.end())
    {
      q_saved_pose_.erase(position_name);
      q_saved_pose_.emplace(position_name, q_pose_to_record);
      saveJointPoseToCSV();
      return true;
    }
  }
  return false;
}

bool JointPoseManager::removeJointPosition(const std::string position_name)
{
  std::map<std::string, JointPosition>::iterator itr;
  /* Cannot remove Rest and Home position */
  if (position_name != "Rest" && position_name != "Home")
  {
    for (itr = q_saved_pose_.begin(); itr != q_saved_pose_.end(); ++itr)
    {
      if (itr->first == position_name)
      {
        q_saved_pose_.erase(itr);
        saveJointPoseToCSV();
        return true;
      }
    }
  }
  return false;
}

bool JointPoseManager::renameJointPosition(const std::string position_name, const std::string new_position_name)
{
  std::map<std::string, JointPosition>::iterator itr;
  /* Cannot rename Rest and Home position */
  if (position_name != "Rest" && position_name != "Home")
  {
    for (itr = q_saved_pose_.begin(); itr != q_saved_pose_.end(); ++itr)
    {
      if (itr->first == position_name)
      {
        q_saved_pose_.emplace(new_position_name, itr->second);
        q_saved_pose_.erase(itr);
        saveJointPoseToCSV();
        return true;
      }
    }
  }
  return false;
}
}
