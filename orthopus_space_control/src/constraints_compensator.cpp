/*
 *  constraints_compensator.cpp
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

#include "orthopus_space_control/constraints_compensator.h"

namespace space_control
{
ConstraintsCompensator::ConstraintsCompensator(const int joint_number, const bool use_quaternion)
  : joint_number_(joint_number)
  , use_quaternion_(use_quaternion)
  , x_current_()
  , x_const_()
  , dx_input_()
  , sampling_period_(0.0)
{
  ROS_DEBUG_STREAM("ConstraintsCompensator constructor");

  orientation_dimension_ = 4;

  pi_ctrl_.resize(orientation_dimension_);
  euler_factor_.resize(orientation_dimension_, 0.0);
}

void ConstraintsCompensator::setDxInput(const SpaceVelocity& dx_input)
{
  dx_input_ = dx_input;
}

void ConstraintsCompensator::setOrientationConstraint(const SpacePosition& x_const)
{
  x_const_ = x_const;
}

void ConstraintsCompensator::init(double sampling_period)
{
  sampling_period_ = sampling_period;
  double p_gain, i_gain, space_position_max_vel;
  ros::param::get("~constraint_ctrl_p_gain", p_gain);
  ros::param::get("~constraint_ctrl_i_gain", i_gain);
  ros::param::get("~space_position_max_vel", space_position_max_vel);

  for (int i = 0; i < orientation_dimension_; i++)
  {
    pi_ctrl_[i].init(sampling_period_, -space_position_max_vel, space_position_max_vel);
    pi_ctrl_[i].setGains(p_gain, i_gain);
  }
}

void ConstraintsCompensator::reset()
{
  for (int i = 0; i < orientation_dimension_; i++)
  {
    pi_ctrl_[i].reset();
    euler_factor_[i] = 1.0;
  }
}

// TODO Following code does not work with quaternion
void ConstraintsCompensator::setXCurrent(const SpacePosition& x_current)
{
  x_current_ = x_current;
  euler_factor_[0] = 1.0;
  euler_factor_[1] = 1.0;
  euler_factor_[2] = 1.0;
  euler_factor_[3] = 1.0;
}

void ConstraintsCompensator::activeOrientationConstraint(SpaceVelocity& dx_output)
{
  /* Copy XYZ velocity */
  dx_output = dx_input_;

  if (orientation_side_ != ORIENTATION_UNDEFINED)
  {
    /* PI control to a predefined desired orientation */
    for (int i = 0; i < orientation_dimension_; i++)
    {
      double error = x_const_[3 + i] - (x_current_[3 + i] * euler_factor_[i]);
      double pi_result;
      pi_ctrl_[i].execute(error, pi_result);
      // ROS_ERROR("PI debug (%d) ; x_const_ = %5f ; x_current_ = %5f ; error = %5f ; pi_result = %5f ; "
      //           "euler_factor_ = %5f",
      //           i, x_const_[3 + i], x_current_[3 + i], error, pi_result, euler_factor_[i]);
      // dx_output[3 + i] = pi_result;
    }
  }
}

void ConstraintsCompensator::run(SpaceVelocity& dx_output)
{
  eulerFlipHandling_();
  detectOrientationSide_(x_current_);
  updateOrientationConstraint_();
  activeOrientationConstraint(dx_output);
}

void ConstraintsCompensator::detectOrientationSide_(const SpacePosition& x_input)
{
}

void ConstraintsCompensator::updateOrientationConstraint_()
{
}

void ConstraintsCompensator::eulerFlipHandling_()
{
}
}
