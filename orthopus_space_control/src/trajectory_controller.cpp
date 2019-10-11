/*
 *  trajectory_controller.cpp
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

#include "orthopus_space_control/trajectory_controller.h"

namespace space_control
{
TrajectoryController::TrajectoryController(const int joint_number, const bool use_quaternion)
  : joint_number_(joint_number)
  , use_quaternion_(use_quaternion)
  , x_traj_desired_(use_quaternion)
  , x_current_(use_quaternion)
  , sampling_period_(0.0)
{
  ROS_DEBUG_STREAM("TrajectoryController constructor");

  // TODO Improvement: put following value in param
  traj_position_tolerance_ = 0.005;     // 5 mm
  traj_orientation_tolerance_ = 0.001;  // 0.01 rad = 0.573 deg
}

void TrajectoryController::init(double sampling_period)
{
  sampling_period_ = sampling_period;
  is_completed_ = true;
  double p_gain, i_gain, cartesian_max_vel;
  ros::param::get("~trajectory_ctrl_p_gain", p_gain);
  ros::param::get("~trajectory_ctrl_i_gain", i_gain);
  ros::param::get("~cartesian_max_vel", cartesian_max_vel);

  for (int i = 0; i < x_traj_desired_.size(); i++)
  {
    pi_ctrl_[i].init(sampling_period_, -cartesian_max_vel, cartesian_max_vel);
    pi_ctrl_[i].setGains(p_gain, i_gain);
    euler_factor_[i] = 1.0;
  }
}

void TrajectoryController::reset()
{
  for (int i = 0; i < x_traj_desired_.size(); i++)
  {
    pi_ctrl_[i].reset();
    euler_factor_[i] = 1.0;
  }
  is_completed_ = true;
}

void TrajectoryController::setXCurrent(const SpacePosition& x_current)
{
  x_current_ = x_current;
  euler_factor_[0] = 1.0;
  euler_factor_[1] = 1.0;
  euler_factor_[2] = 1.0;
  euler_factor_[3] = 1.0;
  euler_factor_[4] = 1.0;
  if (use_quaternion_)
  {
    euler_factor_[5] = 1.0;
  }
  else
  {
    /*
     * HACK : This allows to handle axis inversion. For exemple, when the tool frame orientation
     * is RPY = (0,0,PI) then roll and pitch are in opposite direction from initial orientation
     * RPY = (0,0,0).
     */
    if (std::abs(x_current_[SpacePosition::kRoll]) > M_PI / 2)
    {
      euler_factor_[4] = -1.0;
      euler_factor_[5] = -1.0;
    }
    if (std::abs(x_current_[SpacePosition::kPitch]) > M_PI / 2)
    {
      euler_factor_[3] = -1.0;
      euler_factor_[5] = -1.0;
    }
    if (std::abs(x_current_[SpacePosition::kYaw]) > M_PI / 2)
    {
      euler_factor_[3] = -1.0;
      euler_factor_[4] = -1.0;
    }
  }
}

void TrajectoryController::setTrajectoryPose(const geometry_msgs::Pose& traj_des_pose)
{
  is_completed_ = false;

  /* Here I suppose that user is passing RPY command in pose */
  x_traj_desired_[0] = traj_des_pose.position.x;
  x_traj_desired_[1] = traj_des_pose.position.y;
  x_traj_desired_[2] = traj_des_pose.position.z;
  x_traj_desired_[3] = traj_des_pose.orientation.x;
  x_traj_desired_[4] = traj_des_pose.orientation.y;
  x_traj_desired_[5] = traj_des_pose.orientation.z;
}

void TrajectoryController::computeTrajectory(SpaceVelocity& dx_output)
{
  eulerFlipHandling_();
  updateTrajectoryCompletion_();
  processPi_(dx_output);
}

bool TrajectoryController::isTrajectoryCompleted()
{
  return is_completed_;
}

void TrajectoryController::updateTrajectoryCompletion_()
{
  if (!is_completed_)
  {
    is_completed_ = true;
    for (int i = 0; i < 3; i++)
    {
      if (std::abs(x_traj_desired_[i] - x_current_[i]) > traj_position_tolerance_)
      {
        is_completed_ = false;
        ROS_DEBUG("Trajectory status : delta of %5f on %d axis", x_traj_desired_[i] - x_current_[i], i);
      }
    }
    for (int i = 3; i < 6; i++)
    {
      if (std::abs(x_traj_desired_[i] - x_current_[i]) > traj_orientation_tolerance_)
      {
        is_completed_ = false;
        ROS_DEBUG("Trajectory status : delta of %5f on %d axis", x_traj_desired_[i] - x_current_[i], i);
      }
    }
  }
}

void TrajectoryController::eulerFlipHandling_()
{
  /*
   * HACK : As we only work in the same side (negative yaw) for our use cases
   * (space control in front and back direction of the arm only), this ensure that
   * no flipping happened
   */
  if (x_current_[SpacePosition::kYaw] > M_PI / 2)
  {
    x_current_[SpacePosition::kYaw] -= 2 * M_PI;
  }
}

void TrajectoryController::processPi_(SpaceVelocity& dx_output)
{
  for (int i = 0; i < x_traj_desired_.size(); i++)
  {
    double error = (x_traj_desired_[i] - x_current_[i] * euler_factor_[i]);
    double pi_result;
    pi_ctrl_[i].execute(error, pi_result);
    dx_output[i] = pi_result;
  }
}
}
