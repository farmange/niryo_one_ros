/*
 *  constraints_compensator.h
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
#ifndef CARTESIAN_CONTROLLER_CONSTRAINTS_COMPENSATOR_H
#define CARTESIAN_CONTROLLER_CONSTRAINTS_COMPENSATOR_H

#include "ros/ros.h"

#include "orthopus_space_control/utils/pi_controller.h"

#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose.h"

#include "orthopus_space_control/types/space_position.h"
#include "orthopus_space_control/types/space_velocity.h"

namespace space_control
{
class ConstraintsCompensator
{
public:
  ConstraintsCompensator(const int joint_number, const bool use_quaternion);
  void init(double sampling_period);
  void activeOrientationConstraint(SpaceVelocity& dx_output);
  void run(SpaceVelocity& dx_output);
  void reset();

  void setXCurrent(const SpacePosition& x_current);
  void setDxInput(const SpaceVelocity& dx_input);
  void setOrientationConstraint(const SpacePosition& x_const);

protected:
private:
  ros::NodeHandle n_;

  int joint_number_;
  int orientation_dimension_;
  bool use_quaternion_;
  double sampling_period_;

  SpacePosition x_current_;
  SpacePosition x_const_;
  SpaceVelocity dx_input_;

  std::vector<PiController> pi_ctrl_;
  std::vector<double> euler_factor_;

  enum OrientationSide
  {
    ORIENTATION_FRONT = 0,
    ORIENTATION_BACK,
    ORIENTATION_UNDEFINED
  };
  typedef OrientationSide OrientationSideType;
  OrientationSideType orientation_side_;

  void eulerFlipHandling_();
  void detectOrientationSide_(const SpacePosition& x_input);
  void updateOrientationConstraint_();
};
}
#endif
