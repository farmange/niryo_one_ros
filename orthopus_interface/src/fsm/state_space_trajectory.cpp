/*
 *  state_space_trajectory.cpp
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

#include "orthopus_interface/fsm/state_joint_trajectory.h"
#include "orthopus_interface/fsm/state_space_trajectory.h"
#include "orthopus_interface/fsm/state_disable.h"

#include "orthopus_interface/robot_manager.h"

namespace cartesian_controller
{
State* StateSpaceTrajectory::handleInput(RobotManager& robot, Event event)
{
  ROS_INFO("StateSpaceTrajectory handleInput");

  if (event == Event::Disable)
  {
    return new StateDisable();
  }
  else if (event == Event::ExecuteJointTraj)
  {
    return new StateJointTrajectory();
  }

  // Stay in this state.
  return NULL;
}

void StateSpaceTrajectory::update(RobotManager& robot)
{
  ROS_INFO("StateSpaceTrajectory update");

  ROS_INFO("=== Update joint position (Open loop)...");
  // TODO check q_current consistency (many affectation could lead to issue
  robot.q_current_ = robot.q_command_;
  ROS_INFO("    Done.");

  robot.cartesian_controller_.getInverseKinematic()->requestUpdateAxisConstraints(0, 0.1);
  robot.cartesian_controller_.getInverseKinematic()->requestUpdateAxisConstraints(1, 0.1);
  robot.cartesian_controller_.getInverseKinematic()->requestUpdateAxisConstraints(2, 0.1);
  robot.cartesian_controller_.setDxDesired(robot.dx_desired_);
  robot.cartesian_controller_.setInputSelector(CartesianController::INPUT_TRAJECTORY);
  robot.cartesian_controller_.run(robot.q_current_, robot.q_command_);

  ROS_INFO("=== Send Niryo One commands...");
  robot.sendJointsCommand_();
  ROS_INFO("    Done.");
}

void StateSpaceTrajectory::enter(RobotManager& robot)
{
  ROS_INFO("StateSpaceTrajectory entry");
  robot.cartesian_controller_.reset();
  robot.q_command_ = robot.q_meas_;
  robot.cartesian_controller_.getTrajectoryController()->setTrajectoryPose(robot.drink_pose_);
}
}
