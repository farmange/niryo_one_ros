/*
 *  trajectory_controller.h
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
#ifndef CARTESIAN_CONTROLLER_TRAJECTORY_CONTROLLER_H
#define CARTESIAN_CONTROLLER_TRAJECTORY_CONTROLLER_H

#include "ros/ros.h"

#include "geometry_msgs/Pose.h"
#include "sensor_msgs/JointState.h"

#include "orthopus_space_control/types/space_position.h"
#include "orthopus_space_control/types/space_velocity.h"
#include "orthopus_space_control/utils/pi_controller.h"

namespace space_control
{
class TrajectoryController
{
public:
  TrajectoryController(const int joint_number, const bool use_quaternion);
  void init(double sampling_period);
  void reset();
  void computeTrajectory(SpaceVelocity& dx_output);
  bool isTrajectoryCompleted();
  void setXCurrent(const SpacePosition& x_current);
  void setTrajectoryPose(const SpacePosition& x_pose);

protected:
private:
  ros::NodeHandle n_;
  double sampling_period_;
  bool use_quaternion_;
  int joint_number_;

  SpacePosition x_current_;
  SpacePosition x_traj_desired_;
  PiController pi_ctrl_[6];

  bool is_completed_;
  double euler_factor_[6];
  double traj_position_tolerance_;
  double traj_orientation_tolerance_;

  void updateTrajectoryCompletion_();
  void eulerFlipHandling_();
  void processPi_(SpaceVelocity& dx_output);
};
}
#endif
