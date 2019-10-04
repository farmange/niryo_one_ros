/*
 *  velocity_integrator.h
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
#ifndef CARTESIAN_CONTROLLER_INTEGRATOR_H
#define CARTESIAN_CONTROLLER_INTEGRATOR_H

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>

#include <orthopus_interface/types/joint_position.h>
#include <orthopus_interface/types/joint_velocity.h>

namespace cartesian_controller
{
class VelocityIntegrator
{
public:
  VelocityIntegrator(const int joint_number, const bool use_quaternion);
  void init(const double sampling_period);
  void integrate(const JointVelocity& dq_input, JointPosition& q_output);
  void setQCurrent(const JointPosition& q_current);

protected:
private:
  ros::NodeHandle n_;
  bool use_quaternion_;
  int joint_number_;
  JointPosition q_current_;
  double sampling_period_;
};
}
#endif
