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
#include "std_msgs/UInt16.h"

#include "orthopus_space_control/cartesian_controller.h"

#define INIT_TIME 2

namespace space_control
{
CartesianController::CartesianController(const int joint_number)
  : tc_(joint_number)
  , ik_(joint_number)
  , fk_(joint_number)
  , vi_(joint_number)
  , jpm_(joint_number)
  , joint_number_(joint_number)
  , x_current_()
  , x_orientation_constraint_()
  , x_des_quat_int()
  , dx_desired_()
  , dx_user_desired_()
  , dx_desired_quat_()
  , dx_input_constrained_()
  , dx_desired_selected_()
  , q_command_(joint_number)
  , q_current_(joint_number)
  , dq_desired_(joint_number)
  , sampling_period_(0.0)
{
  ROS_DEBUG_STREAM("CartesianController constructor");
}

void CartesianController::init(double sampling_period, JointPoseManager& joint_pose_manager)
{
  ROS_DEBUG_STREAM("CartesianController init");
  jpm_ = joint_pose_manager;
  sampling_period_ = sampling_period;

  /* This is use to update joint state before running anything */
  ros::spinOnce();

  tc_.init(sampling_period_);
  ik_.init("tool_link", sampling_period_);
  fk_.init("tool_link");
  vi_.init(sampling_period_);
}

void CartesianController::setDebugPublishers(ros::Publisher& q_current_debug_pub, ros::Publisher& x_current_debug_pub,
                                             ros::Publisher& dx_desired_debug_pub)
{
  q_current_debug_pub_ = q_current_debug_pub;
  x_current_debug_pub_ = x_current_debug_pub;
  dx_desired_debug_pub_ = dx_desired_debug_pub;
}

void CartesianController::setControlFeedbackPublisher(ros::Publisher& control_feedback_pub)
{
  control_feedback_pub_ = control_feedback_pub;
}

void CartesianController::reset()
{
  tc_.reset();
  fk_.reset();
  ik_.reset();
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
  ROS_DEBUG_STREAM("Input joint position :");
  ROS_DEBUG_STREAM("q_current_           : " << q_current_);
  fk_.setQCurrent(q_current_);
  fk_.resolveForwardKinematic();
  fk_.getXCurrent(x_current_);
  ROS_DEBUG_STREAM("Forward kinematic computes space position : ");
  ROS_DEBUG_STREAM("x_current_           : " << x_current_);

  if (input_selector_ == INPUT_TRAJECTORY)
  {
    /* If input trajectory is selected, user input is ignore */
    ROS_INFO("=== Perform trajectory control...");
    tc_.setXCurrent(x_current_);
    tc_.computeTrajectory(dx_desired_selected_);
    ROS_DEBUG_STREAM("Trajectory controller generates space velocity :");
    ROS_DEBUG_STREAM("dx_desired_selected_ : " << dx_desired_selected_);
  }
  else
  {
    ROS_INFO("=== Retrieve user space velocity...");
    ROS_DEBUG_STREAM("User sent space velocity : ");
    ROS_DEBUG_STREAM("dx_desired_ : " << dx_desired_);
    dx_desired_selected_ = dx_desired_;
  }

  ROS_INFO("=== Start IK computation...");
  ik_.setQCurrent(q_current_);
  ik_.setXCurrent(x_current_);
  ik_.resolveInverseKinematic(dq_desired_, dx_desired_selected_);
  ROS_DEBUG_STREAM("Inverse kinematic computes joint velocity :");
  ROS_DEBUG_STREAM("dq_desired_          : " << dq_desired_);

  ROS_INFO("=== Integrate joint velocity...");
  vi_.setQCurrent(q_current_);
  vi_.integrate(dq_desired_, q_command_);
  ROS_DEBUG_STREAM("Velocity integrator computes joint position :");
  ROS_DEBUG_STREAM("q_command_           : " << q_command_);

  /* Write joint command output */
  q_command = q_command_;
  ROS_INFO("----------------------------------------");
  publishControlFeedbackTopic_();
  publishDebugTopic_();
}

TrajectoryController* CartesianController::getTrajectoryController()
{
  return &tc_;
}

InverseKinematic* CartesianController::getInverseKinematic()
{
  return &ik_;
}

void CartesianController::publishControlFeedbackTopic_()
{
  std_msgs::UInt16 feedback;
  feedback.data = 0;
  if (ik_.getPositionControlFrame() == InverseKinematic::ControlFrame::Tool)
  {
    feedback.data |= 0x01;
  }
  if (ik_.getOrientationControlFrame() == InverseKinematic::ControlFrame::Tool)
  {
    feedback.data |= 0x02;
  }
  control_feedback_pub_.publish(feedback);
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
  x_current_pose.position.x = x_current_.position.x();
  x_current_pose.position.y = x_current_.position.y();
  x_current_pose.position.z = x_current_.position.z();
  x_current_pose.orientation.w = x_current_.orientation.w();
  x_current_pose.orientation.x = x_current_.orientation.x();
  x_current_pose.orientation.y = x_current_.orientation.y();
  x_current_pose.orientation.z = x_current_.orientation.z();
  x_current_debug_pub_.publish(x_current_pose);

  /* debug space velocity which is sent to inverse kinematic solver */
  geometry_msgs::Pose dx_desired_pose;
  dx_desired_pose.position.x = dx_desired_selected_.position.x();
  dx_desired_pose.position.y = dx_desired_selected_.position.y();
  dx_desired_pose.position.z = dx_desired_selected_.position.z();
  dx_desired_pose.orientation.w = dx_desired_selected_.orientation.w();
  dx_desired_pose.orientation.x = dx_desired_selected_.orientation.x();
  dx_desired_pose.orientation.y = dx_desired_selected_.orientation.y();
  dx_desired_pose.orientation.z = dx_desired_selected_.orientation.z();
  dx_desired_debug_pub_.publish(dx_desired_pose);
}
}
