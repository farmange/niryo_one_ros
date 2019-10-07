/*
 *  state_joint_trajectory.cpp
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

#include "orthopus_interface/fsm/state_disable.h"
#include "orthopus_interface/fsm/state_joint_trajectory.h"

#include "orthopus_interface/robot_manager.h"

namespace cartesian_controller
{
State* StateJointTrajectory::handleInput(RobotManager& robot, Event event)
{
  ROS_INFO("StateJointTrajectory handleInput");
  
  if (event == Event::ExecuteSpaceControl)
  {
    return new StateSpaceControl();
  }
  else if (event == Event::Disable)
  {
    return new StateDisable();
  }
  else if (event == Event::Idle)
  {
    return new StateIdle();
  }
  
  return NULL;
}

void StateJointTrajectory::update(RobotManager& robot)
{
  ROS_INFO("StateJointTrajectory update");
}

void StateJointTrajectory::enter(RobotManager& robot)
{
  ROS_INFO("StateJointTrajectory entry");
  robot.gotoPosition_(robot.pose_manager_.getJoints("Home"));
}
}
