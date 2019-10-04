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

#include "orthopus_interface/constraints_compensator.h"

namespace cartesian_controller
{
ConstraintsCompensator::ConstraintsCompensator(const int joint_number, const bool use_quaternion)
  : joint_number_(joint_number)
  , use_quaternion_(use_quaternion)
  , x_current_(use_quaternion)
  , x_const_(use_quaternion)
  , dx_input_(use_quaternion)
  , sampling_period_(0.0)
{
  ROS_DEBUG_STREAM("ConstraintsCompensator constructor");

  if (use_quaternion_)
  {
    orientation_dimension_ = 4;
  }
  else
  {
    orientation_dimension_ = 3;
  }
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
  double p_gain, i_gain, cartesian_max_vel;
  ros::param::get("~constraint_ctrl_p_gain", p_gain);
  ros::param::get("~constraint_ctrl_i_gain", i_gain);
  ros::param::get("~cartesian_max_vel", cartesian_max_vel);

  for (int i = 0; i < orientation_dimension_; i++)
  {
    pi_ctrl_[i].init(sampling_period_, -cartesian_max_vel, cartesian_max_vel);
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

// TODO add use_quaternion flag and code to handle it
void ConstraintsCompensator::setXCurrent(const SpacePosition& x_current)
{
  x_current_ = x_current;
  euler_factor_[0] = 1.0;
  euler_factor_[1] = 1.0;
  euler_factor_[2] = 1.0;
  if (use_quaternion_)
  {
    euler_factor_[3] = 1.0;
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
      euler_factor_[1] = -1.0;
      euler_factor_[2] = -1.0;
    }
    if (std::abs(x_current_[SpacePosition::kPitch]) > M_PI / 2)
    {
      euler_factor_[0] = -1.0;
      euler_factor_[2] = -1.0;
    }
    if (std::abs(x_current_[SpacePosition::kYaw]) > M_PI / 2)
    {
      euler_factor_[0] = -1.0;
      euler_factor_[1] = -1.0;
    }
  }
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
      dx_output[3 + i] = pi_result;
    }
  }
}

void ConstraintsCompensator::run(SpaceVelocity& dx_output)
{
  eulerFlipHandling();
  detectOrientationSide(x_current_);
  updateOrientationConstraint();
  activeOrientationConstraint(dx_output);
}

void ConstraintsCompensator::detectOrientationSide(const SpacePosition& x_input)
{
  if (use_quaternion_)
  {
    if ((std::abs(x_input[3]) < 0.01) && (std::abs(x_input[4]) < 0.01) && (std::abs(x_input[5]) < 0.01) &&
        (std::abs(x_input[6]) < 1.0) && (std::abs(x_input[6]) > 0.99))
    {
      orientation_side_ = ORIENTATION_BACK;
    }
    else if ((std::abs(x_input[3]) < 0.01) && (std::abs(x_input[4]) < 0.01) && (std::abs(x_input[5]) < 0.01) &&
             (std::abs(x_input[6]) < 0.01))
    {
      orientation_side_ = ORIENTATION_FRONT;
    }
    else
    {
      ROS_ERROR("Orientation undefined : could not properly constrain orientation !");
      orientation_side_ = ORIENTATION_UNDEFINED;
    }
  }
  else
  {
    if ((std::abs(x_input[3]) < 0.2) && (std::abs(x_input[4]) < 0.2) && (std::abs(x_input[5]) < 3.34) &&
        (std::abs(x_input[5]) > 2.94))
    {
      orientation_side_ = ORIENTATION_BACK;
    }
    else if ((std::abs(x_input[3]) < 0.2) && (std::abs(x_input[4]) < 0.2) && (std::abs(x_input[5]) < 0.2))
    {
      orientation_side_ = ORIENTATION_FRONT;
    }
    else
    {
      ROS_ERROR("Orientation undefined : could not properly constrain orientation !");
      orientation_side_ = ORIENTATION_UNDEFINED;
    }
  }
}

void ConstraintsCompensator::updateOrientationConstraint()
{
  if (orientation_side_ == ORIENTATION_FRONT)
  {
    x_const_[3] = 0.0;
    x_const_[4] = 0.0;
    x_const_[5] = 0.0;
    if (use_quaternion_)
    {
      x_const_[6] = 0.0;
    }
  }
  else if (orientation_side_ == ORIENTATION_BACK)
  {
    if (use_quaternion_)
    {
      x_const_[3] = 0.0;
      x_const_[4] = 0.0;
      x_const_[5] = 0.0;
      x_const_[6] = -1.0;
    }
    else
    {
      x_const_[3] = 0.0;
      x_const_[4] = 0.0;
      x_const_[5] = -M_PI;
    }
  }
}

void ConstraintsCompensator::eulerFlipHandling()
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
}
