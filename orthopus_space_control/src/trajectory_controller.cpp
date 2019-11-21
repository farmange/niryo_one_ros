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
TrajectoryController::TrajectoryController(const int joint_number)
  : pi_number_(7)
  , joint_number_(joint_number)
  , x_goal_()
  , x_current_()
  , x_error_()
  , x_traj_tolerance_()
  , sampling_period_(0.0)
{
  pi_ctrl_.resize(pi_number_);
}

void TrajectoryController::init(double sampling_period)
{
  sampling_period_ = sampling_period;
  is_completed_ = true;
  double pos_p_gain, pos_i_gain, orient_p_gain, orient_i_gain, space_position_max_vel, space_orientation_max_vel,
      goal_position_tolerance, goal_orientation_tolerance;
  ros::param::get("~traj_ctrl_position_p_gain", pos_p_gain);
  ros::param::get("~traj_ctrl_position_i_gain", pos_i_gain);
  ros::param::get("~traj_ctrl_orientation_p_gain", orient_p_gain);
  ros::param::get("~traj_ctrl_orientation_i_gain", orient_i_gain);
  ros::param::get("~space_position_max_vel", space_position_max_vel);
  ros::param::get("~space_orientation_max_vel", space_orientation_max_vel);
  ros::param::get("~goal_position_tolerance", goal_position_tolerance);
  ros::param::get("~goal_orientation_tolerance", goal_orientation_tolerance);

  for (int i = 0; i < pi_number_; i++)
  {
    if (i < 3)
    {
      /* The three first PI are used to control X,Y,Z position */
      pi_ctrl_[i].init(sampling_period_, -space_position_max_vel, space_position_max_vel);
      pi_ctrl_[i].setGains(pos_p_gain, pos_i_gain);
      x_traj_tolerance_[i] = goal_position_tolerance;
    }
    else
    {
      /* The three last PI are used to control X,Y,Z orientation */
      pi_ctrl_[i].init(sampling_period_, -space_orientation_max_vel, space_orientation_max_vel);
      pi_ctrl_[i].setGains(orient_p_gain, orient_i_gain);
      x_traj_tolerance_[i] = goal_orientation_tolerance;
    }
    is_comp_completed_[i] = false;
  }
}

void TrajectoryController::reset()
{
  for (int i = 0; i < pi_number_; i++)
  {
    pi_ctrl_[i].reset();
    is_comp_completed_[i] = false;
  }
  is_completed_ = true;
}

void TrajectoryController::setXCurrent(const SpacePosition& x_current)
{
  x_current_ = x_current;
}

void TrajectoryController::setXGoal(const SpacePosition& x_goal)
{
  is_completed_ = false;
  x_goal_ = x_goal;
}

void TrajectoryController::computeTrajectory(SpaceVelocity& dx_output)
{
  ROS_DEBUG_STREAM_NAMED("TrajectoryController", "compute trajectory for the goal position : ");
  ROS_DEBUG_STREAM("x_goal_      : " << x_goal_);
  computeError_();
  updateTrajectoryCompletion_();
  processPi_(dx_output);
}

bool TrajectoryController::isTrajectoryCompleted()
{
  return is_completed_;
}

/* Check if trajectory is achieved */
void TrajectoryController::updateTrajectoryCompletion_()
{
  if (!is_completed_)
  {
    is_completed_ = true;
    for (int i = 0; i < pi_number_; i++)
    {
      if (i != 3)
      {
        /* Do not take into account scalar part of the quaternion */
        if (std::abs(x_error_[i]) > x_traj_tolerance_[i])
        {
          is_completed_ = false;
          ROS_DEBUG("Trajectory status : delta of %5f on %d axis", x_error_[i], i);
          is_comp_completed_[i] = false;
        }
        else
        {
          is_comp_completed_[i] = true;
        }
      }
    }
  }
}

void TrajectoryController::computeError_()
{
  /* Compute position error */
  for (int i = 0; i < 3; i++)
  {
    x_error_[i] = (x_goal_[i] - x_current_[i]);
  }

  /* Compute orientation error */
  Eigen::Quaterniond quat_error = x_goal_.getOrientation() * x_current_.getOrientation().conjugate();
  x_error_.orientation = quat_error;
}

void TrajectoryController::processPi_(SpaceVelocity& dx_output)
{
  for (int i = 0; i < pi_number_; i++)
  {
    if (!is_comp_completed_[i])
    {
      /* To improve stability, only component that did meet the desired trajectory are controlled */
      double pi_result;
      pi_ctrl_[i].execute(x_error_[i], pi_result);
      dx_output[i] = pi_result;
    }
    else
    {
      pi_ctrl_[i].reset();
      dx_output[i] = 0.0;
    }
  }

  /* The dx_output computed velocity reflects an angular velocity (omega) in rad/s that is why the scalar part of the
   * quaternion is set to zero */
  dx_output.orientation.w() = 0.0;
}
}
