/*
 *  state_idle.cpp
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

#include "orthopus_interface/fsm/state_idle.h"
#include "orthopus_interface/fsm/state_disable.h"
#include "orthopus_interface/fsm/state_joint_trajectory.h"
#include "orthopus_interface/fsm/state_space_control.h"
#include "orthopus_interface/fsm/state_space_trajectory.h"

namespace cartesian_controller
{
State* StateIdle::handleInput(RobotManager& robot, Event event)
{
  ROS_INFO("StateIdle handleInput");

  if (event == Event::ExecuteJointTraj)
  {
    return new StateJointTrajectory();
  }
  else if (event == Event::ExecuteSpaceTraj)
  {
    return new StateSpaceTrajectory();
  }
  else if (event == Event::Disable)
  {
    return new StateDisable();
  }

  // Stay in this state.
  return NULL;
}

void StateIdle::update(RobotManager& robot)
{
  ROS_INFO("StateIdle update");
}

void StateIdle::enter(RobotManager& robot)
{
  ROS_INFO("StateIdle entry");
}
}
