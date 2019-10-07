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

#include "orthopus_interface/forward_kinematic.h"

// Eigen
#include "Eigen/Dense"
#include "eigen_conversions/eigen_msg.h"

// TF
#include "tf/tf.h"

namespace cartesian_controller
{
ForwardKinematic::ForwardKinematic(const int joint_number, const bool use_quaternion)
  : joint_number_(joint_number), use_quaternion_(use_quaternion), x_current_(use_quaternion), q_current_(joint_number)
{
  ROS_DEBUG_STREAM("ForwardKinematic constructor");
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  kinematic_model_ = robot_model_loader.getModel();
  kinematic_state_ = std::make_shared<robot_state::RobotState>(kinematic_model_);
  kinematic_state_->setToDefaultValues();
  use_quaternion_ = false;
  end_effector_link_ = "";
}

void ForwardKinematic::init(const std::string end_effector_link, const bool use_quaternion)
{
  use_quaternion_ = use_quaternion;
  end_effector_link_ = end_effector_link;
}

void ForwardKinematic::resolveForwardKinematic()
{
  geometry_msgs::Pose current_pose;

  /* Set kinemtic state of the robot to the previous joint positions computed */
  kinematic_state_->setVariablePositions(q_current_);
  kinematic_state_->updateLinkTransforms();

  /* Get the cartesian state of the tool_link frame */
  const Eigen::Affine3d& end_effector_state =
      kinematic_state_->getGlobalLinkTransform(kinematic_state_->getLinkModel(end_effector_link_));

  /* Convert cartesian state to goemetry_msgs::Pose */
  tf::poseEigenToMsg(end_effector_state, current_pose);

  x_current_[0] = current_pose.position.x;
  x_current_[1] = current_pose.position.y;
  x_current_[2] = current_pose.position.z;

  if (use_quaternion_ == false)
  {
    /* Convert quaternion pose in RPY */
    tf::Quaternion q(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z,
                     current_pose.orientation.w);

    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    x_current_[3] = roll;
    x_current_[4] = pitch;
    x_current_[5] = yaw;
  }
  else
  {
    x_current_[3] = current_pose.orientation.w;
    x_current_[4] = current_pose.orientation.x;
    x_current_[5] = current_pose.orientation.y;
    x_current_[6] = current_pose.orientation.z;
  }
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
