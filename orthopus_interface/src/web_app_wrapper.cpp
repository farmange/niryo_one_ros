/*
 *  wab_app_wrapper.cpp
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

#include "orthopus_interface/web_app_wrapper.h"

#define JOY_BUTTON_A 0
#define JOY_BUTTON_B 1
#define JOY_BUTTON_X 2
#define JOY_BUTTON_Y 3
#define JOY_BUTTON_LB 4
#define JOY_BUTTON_RB 5
#define JOY_BUTTON_BACK 6
#define JOY_BUTTON_START 7
#define JOY_BUTTON_POWER 8
#define JOY_BUTTON_STICK_LEFT 9
#define JOY_BUTTON_STICK_RIGHT 10

#define JOY_AXIS_HORIZONTAL_LEFT 0
#define JOY_AXIS_VERTICAL_LEFT 1
#define JOY_LT 2
#define JOY_AXIS_HORIZONTAL_RIGHT 3
#define JOY_AXIS_VERTICAL_RIGHT 4
#define JOY_RT 5
#define JOY_CROSS_HORIZONTAL 6
#define JOY_CROSS_VERTICAL 7

#define JOY_DEBOUNCE_BUTTON_TIME 0.4
#define JOY_VELOCITY_FACTOR_INC 0.02

namespace cartesian_controller
{
WebAppWrapper::WebAppWrapper()
{
  device_sub_ = n_.subscribe("sim_joy", 1, &WebAppWrapper::joyCallback, this);

  debounce_button_a_ = ros::Time::now();
  debounce_button_start_ = ros::Time::now();
  debounce_button_lb_ = ros::Time::now();

  button_a_ = 0;
  button_start_ = 0;
  button_lb_ = 0;
  button_rb_ = 0;
  button_b_ = 0;

  learning_mode_ = 0;
  cartesian_mode_ = YZ;
  velocity_factor_ = 1.0;
  ros::spin();
}
// TODO Put publish action on parent class
// Convert incoming xbox (joy) commands in require topics
void WebAppWrapper::joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  processButtons(msg);
  updateGripperCmd();
  updateVelocityFactor();
  updateLearningMode();
  updateCartesianMode();

  // Cartesian control with the axes
  geometry_msgs::TwistStamped cartesian_vel;
  cartesian_vel.header.stamp = ros::Time::now();
  // Up/Down Axis of the right stick
  cartesian_vel.twist.linear.x = velocity_factor_ * msg->axes[JOY_AXIS_VERTICAL_RIGHT];
  // Left/Right Axis of the left stick
  cartesian_vel.twist.linear.y = velocity_factor_ * msg->axes[JOY_AXIS_HORIZONTAL_LEFT];
  // Up/Down Axis of the left stick
  cartesian_vel.twist.linear.z = velocity_factor_ * msg->axes[JOY_AXIS_VERTICAL_LEFT];

  // Up/Down Axis of the cross
  cartesian_vel.twist.angular.x = velocity_factor_ * msg->axes[JOY_CROSS_VERTICAL];
  // Left/Right Axis of the cross
  cartesian_vel.twist.angular.y = velocity_factor_ * msg->axes[JOY_CROSS_HORIZONTAL];
  // Left/Right Axis of the right stick
  cartesian_vel.twist.angular.z = velocity_factor_ * msg->axes[JOY_AXIS_HORIZONTAL_RIGHT];

  std_msgs::Int8 cartesian_mode;
  cartesian_mode.data = cartesian_mode_;
  cartesian_cmd_pub_.publish(cartesian_vel);
  gripper_cmd_pub_.publish(gripper_cmd_);
  cartesian_mode_pub_.publish(cartesian_mode);
}

void WebAppWrapper::processButtons(const sensor_msgs::Joy::ConstPtr& msg)
{
  button_a_ = 0;
  button_start_ = 0;
  button_b_ = 0;

  button_lb_ = msg->buttons[JOY_BUTTON_LB];
  button_rb_ = msg->buttons[JOY_BUTTON_RB];
  debounceButtons(msg, JOY_BUTTON_A, debounce_button_a_, button_a_);
  debounceButtons(msg, JOY_BUTTON_START, debounce_button_start_, button_start_);
  debounceButtons(msg, JOY_BUTTON_B, debounce_button_b_, button_b_);
}

void WebAppWrapper::debounceButtons(const sensor_msgs::Joy::ConstPtr& msg, const int button_id,
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

void WebAppWrapper::updateCartesianMode()
{
  // Use B to change cartesian mode
  if (button_b_ == 1 && cartesian_mode_ == XY)
  {
    cartesian_mode_ = YZ;
  }
  else if (button_b_ == 1 && cartesian_mode_ == YZ)
  {
    cartesian_mode_ = XZ;
  }
  else if (button_b_ == 1 && cartesian_mode_ == XZ)
  {
    cartesian_mode_ = XY;
  }
}

void WebAppWrapper::updateVelocityFactor()
{
  if (button_rb_)
  {
    velocity_factor_ += JOY_VELOCITY_FACTOR_INC;
  }
  if (button_lb_)
  {
    velocity_factor_ -= JOY_VELOCITY_FACTOR_INC;
  }
  velocity_factor_ = std::min(velocity_factor_, 1.0);
  velocity_factor_ = std::max(velocity_factor_, 0.0);
}

void WebAppWrapper::updateGripperCmd()
{
  // Use A to toggle gripper state (open/close)
  if (button_a_ == 1 && gripper_cmd_.data == false)
  {
    gripper_cmd_.data = true;
  }
  else if (button_a_ == 1 && gripper_cmd_.data == true)
  {
    gripper_cmd_.data = false;
  }
}

void WebAppWrapper::updateLearningMode()
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

using namespace cartesian_controller;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "xbox_wrapper");

  WebAppWrapper xbox_wrapper;

  return 0;
}
