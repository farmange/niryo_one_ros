/*
 *  cartesian_controller.h
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
#ifndef CARTESIAN_CONTROLLER_CARTESIAN_CONTROLLER_H
#define CARTESIAN_CONTROLLER_CARTESIAN_CONTROLLER_H

#include "ros/ros.h"

#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "niryo_one_msgs/SetInt.h"

#include "orthopus_space_control/inverse_kinematic.h"
#include "orthopus_space_control/pose_manager.h"
#include "orthopus_space_control/trajectory_controller.h"
#include "orthopus_space_control/constraints_compensator.h"

#include "orthopus_space_control/forward_kinematic.h"
#include "orthopus_space_control/velocity_integrator.h"

#include "orthopus_space_control/types/joint_position.h"
#include "orthopus_space_control/types/joint_velocity.h"
#include "orthopus_space_control/types/space_position.h"
#include "orthopus_space_control/types/space_velocity.h"

namespace space_control
{
class CartesianController
{
public:
  enum InputSelector
  {
    INPUT_TRAJECTORY = 0,
    INPUT_USER
  };
  typedef InputSelector InputSelectorType;

  CartesianController(const int joint_number, const bool use_quaternion);
  void init(double sampling_period, PoseManager& pose_manager);
  void reset();
  void run(const JointPosition& q_current, JointPosition& q_command);

  void setDxDesired(const SpaceVelocity& dx_desired);
  void setInputSelector(const InputSelectorType input_selector);

  // Callbacks
  void callbackMoveGroupState(const std_msgs::Int32Ptr& msg);
  void callbackVelocitiesDesired(const geometry_msgs::TwistStampedPtr& msg);

  ConstraintsCompensator* getConstraintsCompensator();
  TrajectoryController* getTrajectoryController();
  InverseKinematic* getInverseKinematic();

protected:
private:
  ros::NodeHandle n_;

  int joint_number_;
  bool use_quaternion_;
  double sampling_period_;

  TrajectoryController tc_;
  InverseKinematic ik_;
  ForwardKinematic fk_;
  VelocityIntegrator vi_;
  PoseManager pm_;
  ConstraintsCompensator cc_;

  JointPosition q_command_;
  JointPosition q_current_;
  JointVelocity dq_desired_;

  SpacePosition x_current_;
  SpacePosition x_orientation_constraint_;

  SpaceVelocity dx_user_desired_;
  SpaceVelocity dx_desired_;
  SpaceVelocity dx_desired_prev_;
  SpaceVelocity dx_input_constrained_;
  SpaceVelocity dx_desired_selected_;

  InputSelectorType input_selector_;
};
}
#endif
