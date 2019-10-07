/*
 *  state_space_control.cpp
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

#include "orthopus_interface/fsm/state_space_control.h"
#include "orthopus_interface/fsm/state_disable.h"
#include "orthopus_interface/fsm/state_idle.h"

#include "orthopus_interface/robot_manager.h"

namespace cartesian_controller
{
State* StateSpaceControl::handleInput(RobotManager& robot, Event event)
{
  ROS_INFO("StateSpaceControl handleInput");

  if (event == Event::Disable)
  {
    return new StateDisable();
  }
  else if (event == Event::ExecuteJointTraj)
  {
    return new StateJointTrajectory();
  }
  else if (event == Event::ExecuteSpaceTraj)
  {
    return new StateSpaceTrajectory();
  }
  
  // Stay in this state.
  return NULL;
}

void StateSpaceControl::update(RobotManager& robot)
{
  ROS_INFO("StateSpaceControl update");
  ROS_INFO("=== Update joint position (Open loop)...");
  robot.q_current_ = robot.q_command_;
  ROS_INFO("    Done.");
  
  robot.cartesian_controller_.setDxDesired(robot.dx_desired_);
  robot.cartesian_controller_.setInputSelector(CartesianController::INPUT_USER);
  robot.cartesian_controller_.run(robot.q_current_, robot.q_command_);
  
  ROS_INFO("=== Send Niryo One commands...");
  robot.sendJointsCommand_();
  ROS_INFO("    Done.");
}

void StateSpaceControl::enter(RobotManager& robot)
{
  ROS_INFO("StateSpaceControl entry");
  /* Switch to cartesian mode when position is completed */
  robot.cartesian_controller_.reset();
  robot.q_command_ = robot.q_meas_;
}
}
