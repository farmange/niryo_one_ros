/*
 *  device_web_app.cpp
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

#include "orthopus_space_control/input_device/device_web_app.h"

namespace input_device
{
DeviceWebApp::DeviceWebApp()
{
  device_sub_ = n_.subscribe("sim_joy", 1, &DeviceWebApp::callbackJoy_, this);
  ros::spin();
}

void DeviceWebApp::callbackJoy_(const sensor_msgs::Joy::ConstPtr& msg)
{
  geometry_msgs::TwistStamped cartesian_vel;
  cartesian_vel.header.stamp = ros::Time::now();
  // Up/Down Axis of the right stick
  cartesian_vel.twist.linear.x = msg->axes[XBOX_AXIS_VERTICAL_RIGHT];
  // Left/Right Axis of the left stick
  cartesian_vel.twist.linear.y = msg->axes[XBOX_AXIS_HORIZONTAL_LEFT];
  // Up/Down Axis of the left stick
  cartesian_vel.twist.linear.z = msg->axes[XBOX_AXIS_VERTICAL_LEFT];

  // Up/Down Axis of the cross
  cartesian_vel.twist.angular.x = msg->axes[XBOX_CROSS_VERTICAL];
  // Left/Right Axis of the cross
  cartesian_vel.twist.angular.y = msg->axes[XBOX_CROSS_HORIZONTAL];
  // Left/Right Axis of the right stick
  cartesian_vel.twist.angular.z = msg->axes[XBOX_AXIS_HORIZONTAL_RIGHT];

  cartesian_cmd_pub_.publish(cartesian_vel);
}
}

using namespace input_device;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "device_web_app");

  DeviceWebApp device_web_app;

  return 0;
}
