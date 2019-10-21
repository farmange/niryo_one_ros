/*
 *  cartesian_controller.cpp
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

#include "tf2/LinearMath/Quaternion.h"

#include "niryo_one_msgs/GetInt.h"
#include "niryo_one_msgs/RobotMove.h"

#include "orthopus_space_control/cartesian_controller.h"

#define INIT_TIME 2

namespace space_control
{
CartesianController::CartesianController(const int joint_number, const bool use_quaternion)
  : tc_(joint_number, use_quaternion)
  , ik_(joint_number, use_quaternion)
  , fk_(joint_number, use_quaternion)
  , vi_(joint_number, use_quaternion)
  , pm_(joint_number, use_quaternion)
  , cc_(joint_number, use_quaternion)
  , joint_number_(joint_number)
  , use_quaternion_(use_quaternion)
  , x_current_(use_quaternion)
  , x_orientation_constraint_(use_quaternion)
  , dx_desired_(use_quaternion)
  , dx_user_desired_(use_quaternion)
  , dx_desired_prev_(use_quaternion)
  , dx_input_constrained_(use_quaternion)
  , dx_desired_selected_(use_quaternion)
  , q_command_(joint_number)
  , q_current_(joint_number)
  , dq_desired_(joint_number)
  , sampling_period_(0.0)
{
  ROS_DEBUG_STREAM("CartesianController constructor");
}

void CartesianController::init(double sampling_period, PoseManager& pose_manager)
{
  ROS_DEBUG_STREAM("CartesianController init");
  pm_ = pose_manager;
  sampling_period_ = sampling_period;

  /* This is use to update joint state before running anything */
  ros::spinOnce();

  tc_.init(sampling_period_);
  ik_.init("tool_link", sampling_period_);
  fk_.init("tool_link");
  vi_.init(sampling_period_);
  cc_.init(sampling_period_);
}

void CartesianController::setDebugPublishers(ros::Publisher& q_current_debug_pub, ros::Publisher& x_current_debug_pub,
                                             ros::Publisher& dx_desired_debug_pub)
{
  q_current_debug_pub_ = q_current_debug_pub;
  x_current_debug_pub_ = x_current_debug_pub;
  dx_desired_debug_pub_ = dx_desired_debug_pub;
}

void CartesianController::reset()
{
  tc_.reset();
  ik_.reset();
  cc_.reset();
}

void CartesianController::setDxDesired(const SpaceVelocity& dx_desired)
{
  dx_desired_ = dx_desired;
}

void CartesianController::setInputSelector(const InputSelectorType input_selector)
{
  input_selector_ = input_selector;
}

void CartesianController::run(const JointPosition& q_current, JointPosition& q_command)
{
  ROS_INFO("---- Execute cartesian control loop ----");
  q_current_ = q_current;

  ROS_INFO("=== Start FK computation...");
  ROS_DEBUG_STREAM("Input joint position                             : " << q_current_);
  fk_.setQCurrent(q_current_);
  fk_.resolveForwardKinematic();
  fk_.getXCurrent(x_current_);
  ROS_DEBUG_STREAM("Forward kinematic computes space position        : " << x_current_);

  if (input_selector_ == INPUT_TRAJECTORY)
  {
    /* If input trajectory is selected, user input is ignore */
    ROS_INFO("=== Perform trajectory control...");
    tc_.setXCurrent(x_current_);
    tc_.computeTrajectory(dx_desired_selected_);
    ROS_DEBUG_STREAM("Trajectory controller generates space velocity   : " << dx_desired_selected_);
  }
  else
  {
    ROS_INFO("=== Retrieve user space velocity...");
    ROS_DEBUG_STREAM("User sent space velocity                         : " << dx_desired_);

    ROS_INFO("=== Perform constraints compensation...");
    cc_.setXCurrent(x_current_);
    cc_.setDxInput(dx_desired_);
    cc_.run(dx_desired_selected_);
    // dx_desired_selected_ = dx_desired_;
    ROS_DEBUG_STREAM("Constraints compensator updates space velocity   : " << dx_desired_selected_);
  }

  ROS_INFO("=== Start IK computation...");
  ik_.setQCurrent(q_current_);
  ik_.setXCurrent(x_current_);
  ik_.resolveInverseKinematic(dq_desired_, dx_desired_selected_);
  ROS_DEBUG_STREAM("Inverse kinematic computes joint velocity        : " << dq_desired_);

  ROS_INFO("=== Integrate joint velocity...");
  vi_.setQCurrent(q_current_);
  vi_.integrate(dq_desired_, q_command_);
  ROS_DEBUG_STREAM("Velocity integrator computes joint position      : " << q_command_);

  /* Write joint command output */
  q_command = q_command_;
  ROS_INFO("----------------------------------------");

  publishDebugTopic_();
}

ConstraintsCompensator* CartesianController::getConstraintsCompensator()
{
  return &cc_;
}

TrajectoryController* CartesianController::getTrajectoryController()
{
  return &tc_;
}

InverseKinematic* CartesianController::getInverseKinematic()
{
  return &ik_;
}

void CartesianController::publishDebugTopic_()
{
  /* debug current joint position */
  sensor_msgs::JointState q_current_state;
  q_current_state.position.resize(joint_number_);
  for (int i = 0; i < joint_number_; i++)
  {
    q_current_state.position[i] = q_current_[i];
  }
  q_current_debug_pub_.publish(q_current_state);

  /* debug current space position (result of forward kinematic) */
  geometry_msgs::Pose x_current_pose;
  x_current_pose.position.x = x_current_[SpacePosition::kX];
  x_current_pose.position.y = x_current_[SpacePosition::kY];
  x_current_pose.position.z = x_current_[SpacePosition::kZ];
  if (use_quaternion_)
  {
    x_current_pose.orientation.w = x_current_[SpacePosition::kQw];
    x_current_pose.orientation.x = x_current_[SpacePosition::kQx];
    x_current_pose.orientation.y = x_current_[SpacePosition::kQy];
    x_current_pose.orientation.z = x_current_[SpacePosition::kQz];
  }
  else
  {
    x_current_pose.orientation.x = x_current_[SpacePosition::kRoll];
    x_current_pose.orientation.y = x_current_[SpacePosition::kPitch];
    x_current_pose.orientation.z = x_current_[SpacePosition::kYaw];
  }
  x_current_debug_pub_.publish(x_current_pose);

  /* debug space velocity which is sent to inverse kinematic solver */
  geometry_msgs::Pose dx_desired_pose;
  dx_desired_pose.position.x = dx_desired_selected_[SpacePosition::kX];
  dx_desired_pose.position.y = dx_desired_selected_[SpacePosition::kY];
  dx_desired_pose.position.z = dx_desired_selected_[SpacePosition::kZ];
  if (use_quaternion_)
  {
    dx_desired_pose.orientation.w = dx_desired_selected_[SpacePosition::kQw];
    dx_desired_pose.orientation.x = dx_desired_selected_[SpacePosition::kQx];
    dx_desired_pose.orientation.y = dx_desired_selected_[SpacePosition::kQy];
    dx_desired_pose.orientation.z = dx_desired_selected_[SpacePosition::kQz];
  }
  else
  {
    dx_desired_pose.orientation.x = dx_desired_selected_[SpacePosition::kRoll];
    dx_desired_pose.orientation.y = dx_desired_selected_[SpacePosition::kPitch];
    dx_desired_pose.orientation.z = dx_desired_selected_[SpacePosition::kYaw];
  }
  dx_desired_debug_pub_.publish(dx_desired_pose);
}
}
