/*
 *  forward_kinematic.h
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
#ifndef CARTESIAN_CONTROLLER_FORWARD_KINEMATIC_H
#define CARTESIAN_CONTROLLER_FORWARD_KINEMATIC_H

#include "ros/ros.h"

// MoveIt!
#include "moveit/robot_model/robot_model.h"
#include "moveit/robot_model_loader/robot_model_loader.h"
#include "moveit/robot_state/robot_state.h"

#include "geometry_msgs/Pose.h"
#include "sensor_msgs/JointState.h"

#include "orthopus_space_control/types/joint_position.h"
#include "orthopus_space_control/types/space_position.h"

namespace space_control
{
class ForwardKinematic
{
public:
  ForwardKinematic(const int joint_number);
  void init(const std::string end_effector_link);
  void reset();
  void resolveForwardKinematic();
  void setQCurrent(const JointPosition& q_current);
  void getXCurrent(SpacePosition& x_current);

protected:
private:
  ros::NodeHandle n_;

  bool init_flag_;
  int joint_number_;

  std::string end_effector_link_;
  JointPosition q_current_;
  SpacePosition x_current_;
  robot_model::RobotModelPtr kinematic_model_;
  robot_state::RobotStatePtr kinematic_state_;
};
}
#endif
