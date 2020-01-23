/*
 *  device_spacenav.cpp
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

#include "orthopus_space_control/input_device/device_spacenav.h"
#include "orthopus_space_control/input_device/spacenav_config.h"

namespace input_device
{
DeviceSpacenav::DeviceSpacenav()
{
  device_sub_ = n_.subscribe("spacenav/joy", 1, &DeviceSpacenav::callbackJoy_, this);

  debounce_button_left_ = ros::Time::now();
  debounce_button_right_ = ros::Time::now();

  button_left_ = 0;
  button_right_ = 0;

  gripper_toggle_ = false;
  control_mode_toggle_ = false;

  ros::param::get("~debounce_button_time", debounce_button_time_);

  setGripperId_();

  ros::spin();
}

void DeviceSpacenav::callbackJoy_(const sensor_msgs::Joy::ConstPtr& msg)
{
  processButtons_(msg);
  updateGripperCmd_();
  updateControlMode_();

  // Cartesian control with the axes
  geometry_msgs::TwistStamped cartesian_vel;
  cartesian_vel.header.stamp = ros::Time::now();
  ROS_ERROR_STREAM("Control mode : " << control_mode_toggle_);
  if (control_mode_toggle_)
  {
    cartesian_vel.twist.linear.x = 0.0;
    cartesian_vel.twist.linear.y = 0.0;
    cartesian_vel.twist.linear.z = 0.0;
    cartesian_vel.twist.angular.x = msg->axes[3];
    cartesian_vel.twist.angular.y = msg->axes[4];
    cartesian_vel.twist.angular.z = msg->axes[5];
  }
  else
  {
    cartesian_vel.twist.linear.x = msg->axes[0];
    cartesian_vel.twist.linear.y = msg->axes[1];
    cartesian_vel.twist.linear.z = msg->axes[2];
    cartesian_vel.twist.angular.x = 0.0;
    cartesian_vel.twist.angular.y = 0.0;
    cartesian_vel.twist.angular.z = 0.0;
  }

  cartesian_cmd_pub_.publish(cartesian_vel);
}

void DeviceSpacenav::processButtons_(const sensor_msgs::Joy::ConstPtr& msg)
{
  button_left_ = 0;
  button_right_ = 0;

  debounceButtons_(msg, SPACENAV_BUTTON_LEFT, debounce_button_left_, button_left_);
  debounceButtons_(msg, SPACENAV_BUTTON_RIGHT, debounce_button_right_, button_right_);
}

void DeviceSpacenav::debounceButtons_(const sensor_msgs::Joy::ConstPtr& msg, const int button_id,
                                      ros::Time& debounce_timer_ptr, int& button_value_ptr)
{
  if (msg->buttons[button_id])
  {
    if (ros::Time::now() > debounce_timer_ptr)
    {
      debounce_timer_ptr = ros::Time::now() + ros::Duration(debounce_button_time_);
      button_value_ptr = msg->buttons[button_id];
    }
  }
}

void DeviceSpacenav::updateGripperCmd_()
{
  // Use A to toggle gripper state (open/close)
  if (button_left_ == 1 && gripper_toggle_ == false)
  {
    gripper_toggle_ = true;
    openGripper_();
  }
  else if (button_left_ == 1 && gripper_toggle_ == true)
  {
    gripper_toggle_ = false;
    closeGripper_();
  }
}

void DeviceSpacenav::updateControlMode_()
{
  // Use A to toggle gripper state (open/close)
  if (button_right_ == 1 && control_mode_toggle_ == false)
  {
    control_mode_toggle_ = true;  // orientation
  }
  else if (button_right_ == 1 && control_mode_toggle_ == true)
  {
    control_mode_toggle_ = false;  // translation
  }
}
}

using namespace input_device;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "device_spacenav");

  DeviceSpacenav device_spacenav;

  return 0;
}
