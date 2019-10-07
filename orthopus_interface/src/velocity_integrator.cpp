/*
 *  velocity_integrator.cpp
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

#include "orthopus_interface/velocity_integrator.h"

namespace cartesian_controller
{
VelocityIntegrator::VelocityIntegrator(const int joint_number, const bool use_quaternion)
  : joint_number_(joint_number), use_quaternion_(use_quaternion), q_current_(joint_number)
{
  ROS_DEBUG_STREAM("VelocityIntegrator constructor");
  sampling_period_ = 0.0;
}

void VelocityIntegrator::init(const double sampling_period)
{
  sampling_period_ = sampling_period;
}

void VelocityIntegrator::integrate(const JointVelocity& dq_input, JointPosition& q_output)
{
  for (int i = 0; i < joint_number_; i++)
  {
    q_output[i] = q_current_[i] + dq_input[i] * sampling_period_;
  }
}

void VelocityIntegrator::setQCurrent(const JointPosition& q_current)
{
  q_current_ = q_current;
}
}
