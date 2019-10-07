/*
 *  spacenav_wrapper.cpp
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

#include "niryo_one_msgs/SetInt.h"
#include "niryo_one_msgs/OpenGripper.h"
#include "niryo_one_msgs/CloseGripper.h"

#include "orthopus_interface/spacenav_wrapper.h"

#define JOY_BUTTON_LEFT 0
#define JOY_BUTTON_RIGHT 1

#define JOY_DEBOUNCE_BUTTON_TIME 0.4
#define JOY_VELOCITY_FACTOR_INC 0.02

namespace cartesian_controller
{
SpacenavWrapper::SpacenavWrapper()
{
  device_sub_ = n_.subscribe("spacenav/joy", 1, &SpacenavWrapper::joyCallback_, this);

  debounce_button_left_ = ros::Time::now();
  debounce_button_right_ = ros::Time::now();

  button_left_ = 0;
  button_right_ = 0;

  learning_mode_ = 0;
  velocity_factor_ = 0.0;

  initializeServices_();
  setGripperId_();

  ros::spin();
}

void SpacenavWrapper::initializeServices_()
{
  ros::service::waitForService("/niryo_one/change_tool");
  ros::service::waitForService("/niryo_one/tools/open_gripper");
  ros::service::waitForService("/niryo_one/tools/close_gripper");

  change_tool_srv_ = n_.serviceClient<niryo_one_msgs::SetInt>("niryo_one/change_tool");
  open_gripper_srv_ = n_.serviceClient<niryo_one_msgs::OpenGripper>("niryo_one/tools/open_gripper");
  close_gripper_srv_ = n_.serviceClient<niryo_one_msgs::CloseGripper>("niryo_one/tools/close_gripper");
}

// TODO Put publish action on parent class
// Convert incoming xbox (joy) commands in require topics
void SpacenavWrapper::joyCallback_(const sensor_msgs::Joy::ConstPtr& msg)
{
  processButtons_(msg);
  updateGripperCmd_();

  // Cartesian control with the axes
  geometry_msgs::TwistStamped cartesian_vel;
  cartesian_vel.header.stamp = ros::Time::now();
  velocity_factor_ = 0.8;
  cartesian_vel.twist.linear.x = velocity_factor_ * msg->axes[0];
  cartesian_vel.twist.linear.y = velocity_factor_ * msg->axes[1];
  cartesian_vel.twist.linear.z = velocity_factor_ * msg->axes[2];

  //   cartesian_vel.twist.angular.x = velocity_factor_ * msg->axes[3];
  //   cartesian_vel.twist.angular.y = velocity_factor_ * msg->axes[4];
  //   cartesian_vel.twist.angular.z = velocity_factor_ * msg->axes[5];

  std_msgs::Int8 cartesian_mode;
  cartesian_mode.data = cartesian_mode_;
  cartesian_cmd_pub_.publish(cartesian_vel);
  gripper_cmd_pub_.publish(gripper_cmd_);
  cartesian_mode_pub_.publish(cartesian_mode);
}

void SpacenavWrapper::processButtons_(const sensor_msgs::Joy::ConstPtr& msg)
{
  button_left_ = 0;
  button_right_ = 0;

  debounceButtons_(msg, JOY_BUTTON_LEFT, debounce_button_left_, button_left_);
  debounceButtons_(msg, JOY_BUTTON_RIGHT, debounce_button_right_, button_right_);
}

void SpacenavWrapper::debounceButtons_(const sensor_msgs::Joy::ConstPtr& msg, const int button_id,
                                       ros::Time& debounce_timer_ptr, int& button_value_ptr)
{
  if (msg->buttons[button_id])
  {
    if (ros::Time::now() > debounce_timer_ptr)
    {
      debounce_timer_ptr = ros::Time::now() + ros::Duration(JOY_DEBOUNCE_BUTTON_TIME);
      button_value_ptr = msg->buttons[button_id];
    }
  }
}

void SpacenavWrapper::updateGripperCmd_()
{
  // Use A to toggle gripper state (open/close)
  if (button_left_ == 1 && gripper_cmd_.data == false)
  {
    gripper_cmd_.data = true;
    openGripper_();
  }
  else if (button_left_ == 1 && gripper_cmd_.data == true)
  {
    gripper_cmd_.data = false;
    closeGripper_();
  }
}

void SpacenavWrapper::setGripperId_()
{
  ROS_INFO("SpacenavWrapper::setGripperId_");
  niryo_one_msgs::SetInt gripper_id;
  gripper_id.request.value = 12;
  change_tool_srv_.call(gripper_id);  // gripper 2
}

void SpacenavWrapper::openGripper_()
{
  ROS_INFO("SpacenavWrapper::openGripper_");
  niryo_one_msgs::OpenGripper open_gripper;
  open_gripper.request.id = 12;
  open_gripper.request.open_position = 640;
  open_gripper.request.open_speed = 300;
  open_gripper.request.open_hold_torque = 128;
  open_gripper_srv_.call(open_gripper);
}

void SpacenavWrapper::closeGripper_()
{
  ROS_INFO("SpacenavWrapper::closeGripper_");
  niryo_one_msgs::CloseGripper close_gripper;
  close_gripper.request.id = 12;
  close_gripper.request.close_position = 400;
  close_gripper.request.close_speed = 300;
  close_gripper.request.close_hold_torque = 128;
  close_gripper.request.close_max_torque = 1023;
  close_gripper_srv_.call(close_gripper);
}
}

using namespace cartesian_controller;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "xbox_wrapper");

  SpacenavWrapper xbox_wrapper;

  return 0;
}
