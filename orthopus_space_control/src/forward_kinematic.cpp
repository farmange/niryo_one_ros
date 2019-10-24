/*
 *  forward_kinematic.cpp
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

#include "orthopus_space_control/forward_kinematic.h"

// Eigen
#include "Eigen/Dense"

// TF
#include "tf/tf.h"

namespace space_control
{
ForwardKinematic::ForwardKinematic(const int joint_number, const bool use_quaternion)
  : joint_number_(joint_number), use_quaternion_(use_quaternion), x_current_(), q_current_(joint_number)
{
  ROS_DEBUG_STREAM("ForwardKinematic constructor");
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  kinematic_model_ = robot_model_loader.getModel();
  kinematic_state_ = std::make_shared<robot_state::RobotState>(kinematic_model_);
  kinematic_state_->setToDefaultValues();
  end_effector_link_ = "";
}

void ForwardKinematic::init(const std::string end_effector_link)
{
  end_effector_link_ = end_effector_link;
  init_flag_ = true;
}

void ForwardKinematic::reset()
{
  init_flag_ = true;
}

void ForwardKinematic::resolveForwardKinematic()
{
  /* Set kinemtic state of the robot to the previous joint positions computed */
  kinematic_state_->setVariablePositions(q_current_);
  kinematic_state_->updateLinkTransforms();

  /* Get the cartesian state of the tool_link frame */
  const Eigen::Affine3d& end_effector_state =
      kinematic_state_->getGlobalLinkTransform(kinematic_state_->getLinkModel(end_effector_link_));

  /* Convert rotation matrix in quaternion */
  // TODO QUAT : eigen or tf ?
  Eigen::Quaterniond conv_quat(end_effector_state.linear());
  Eigen::Quaterniond* final_quat;
  if (init_flag_)
  {
    init_flag_ = false;
    final_quat = new Eigen::Quaterniond(conv_quat);
  }
  else
  {
    double diff_norm = sqrt(pow(conv_quat.w() - conv_quat_prev_.w(), 2) + pow(conv_quat.x() - conv_quat_prev_.x(), 2) +
                            pow(conv_quat.y() - conv_quat_prev_.y(), 2) + pow(conv_quat.z() - conv_quat_prev_.z(), 2));
    if (diff_norm > 1)
    {
      final_quat = new Eigen::Quaterniond(-conv_quat.w(), -conv_quat.x(), -conv_quat.y(), -conv_quat.z());
    }
    else
    {
      final_quat = new Eigen::Quaterniond(conv_quat);
    }
  }
  conv_quat_prev_ = *final_quat;

  x_current_.setX(end_effector_state.translation()[0]);
  x_current_.setY(end_effector_state.translation()[1]);
  x_current_.setZ(end_effector_state.translation()[2]);
  x_current_.setQw(final_quat->w());
  x_current_.setQx(final_quat->x());
  x_current_.setQy(final_quat->y());
  x_current_.setQz(final_quat->z());
}

void ForwardKinematic::setQCurrent(const JointPosition& q_current)
{
  q_current_ = q_current;
}

void ForwardKinematic::getXCurrent(SpacePosition& x_current)
{
  x_current = x_current_;
}
}
