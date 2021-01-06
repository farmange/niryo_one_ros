/*
 *  device_xbox.cpp
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

#include "orthopus_space_control/input_device/device_xbox.h"
#include <orthopus_space_control/input_device/xbox_config.h>

namespace input_device
{
DeviceXbox::DeviceXbox()
{
  device_sub_ = n_.subscribe("joy", 1, &DeviceXbox::callbackJoy, this);
  
  debounce_button_a_ = ros::Time::now();
  debounce_button_start_ = ros::Time::now();
  debounce_button_lb_ = ros::Time::now();
  debounce_button_rb_ = ros::Time::now();
  debounce_button_b_ = ros::Time::now();

  button_a_ = 0;
  button_start_ = 0;
  button_lb_ = 0;
  button_rb_ = 0;
  button_b_ = 0;

  learning_mode_ = 0;
  velocity_factor_ = 0.0;
  gripper_toggle_ = false;  
  
  ros::param::get("~debounce_button_time", debounce_button_time_);
  ros::param::get("~velocity_factor_inc", velocity_factor_inc_);
  
  ros::spin();
}


void DeviceXbox::callbackJoy(const sensor_msgs::Joy::ConstPtr& msg)
{
  processButtons(msg);
  updateGripperCmd();
  updateVelocityFactor();
  updateLearningMode();

  // Cartesian control with the axes
  geometry_msgs::TwistStamped cartesian_vel;
  cartesian_vel.header.stamp = ros::Time::now();
  // Up/Down Axis of the right stick
  cartesian_vel.twist.linear.x = velocity_factor_ * msg->axes[XBOX_AXIS_VERTICAL_RIGHT];
  // Left/Right Axis of the left stick
  cartesian_vel.twist.linear.y = velocity_factor_ * msg->axes[XBOX_AXIS_HORIZONTAL_LEFT];
  // Up/Down Axis of the left stick
  cartesian_vel.twist.linear.z = velocity_factor_ * msg->axes[XBOX_AXIS_VERTICAL_LEFT];

  // Up/Down Axis of the cross
  cartesian_vel.twist.angular.x = velocity_factor_ * msg->axes[XBOX_CROSS_VERTICAL];
  // Left/Right Axis of the cross
  cartesian_vel.twist.angular.y = velocity_factor_ * msg->axes[XBOX_CROSS_HORIZONTAL];
  // Left/Right Axis of the right stick
  cartesian_vel.twist.angular.z = velocity_factor_ * msg->axes[XBOX_AXIS_HORIZONTAL_RIGHT];

  cartesian_cmd_pub_.publish(cartesian_vel);
}

void DeviceXbox::processButtons(const sensor_msgs::Joy::ConstPtr& msg)
{
  button_a_ = 0;
  button_start_ = 0;
  button_lb_ = 0;
  button_rb_ = 0;
  button_b_ = 0;

  debounceButtons(msg, XBOX_BUTTON_A, debounce_button_a_, button_a_);
  debounceButtons(msg, XBOX_BUTTON_START, debounce_button_start_, button_start_);
  debounceButtons(msg, XBOX_BUTTON_LB, debounce_button_lb_, button_lb_);
  debounceButtons(msg, XBOX_BUTTON_RB, debounce_button_rb_, button_rb_);
  debounceButtons(msg, XBOX_BUTTON_B, debounce_button_b_, button_b_);
}

void DeviceXbox::debounceButtons(const sensor_msgs::Joy::ConstPtr& msg, const int button_id,
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

void DeviceXbox::updateVelocityFactor()
{
  if (button_rb_)
  {
    velocity_factor_ += velocity_factor_inc_;
  }
  if (button_lb_)
  {
    velocity_factor_ -= velocity_factor_inc_;
  }
  velocity_factor_ = std::min(velocity_factor_, 1.0);
  velocity_factor_ = std::max(velocity_factor_, 0.0);
}

void DeviceXbox::updateGripperCmd()
{
  // Use A to toggle gripper state (open/close)
  if (button_a_ == 1 && gripper_toggle_ == false)
  {
    gripper_toggle_ = true;
    openGripper_();
  }
  else if (button_a_ == 1 && gripper_toggle_ == true)
  {
    gripper_toggle_ = false;
    closeGripper_();
  }
}

void DeviceXbox::updateLearningMode()
{
  // Use START to toggle learning mode state (open/close)
  if (button_start_ == 1 && learning_mode_ == 0)
  {
    learning_mode_ = 1;
    requestLearningMode(learning_mode_);
  }
  else if (button_start_ == 1 && learning_mode_ == 1)
  {
    learning_mode_ = 0;
    requestLearningMode(learning_mode_);
  }
}
}

using namespace input_device;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "device_xbox");

  DeviceXbox device_xbox;

  return 0;
}
