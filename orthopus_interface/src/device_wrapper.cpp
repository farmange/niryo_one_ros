/*
 *  device_wrapper.cpp
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

#include "geometry_msgs/TwistStamped.h"
#include "niryo_one_msgs/SetInt.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"
#include "orthopus_interface/device_wrapper.h"

namespace cartesian_controller
{
DeviceWrapper::DeviceWrapper()
{
  ROS_DEBUG("DeviceWrapper constructor");
  // to subscribe in daughter class
  //     device_sub_;
  cartesian_cmd_pub_ = n_.advertise<geometry_msgs::TwistStamped>("/orthopus_interface/input_device_velocity", 1);
  gripper_cmd_pub_ = n_.advertise<std_msgs::Bool>("gripper_des", 1);
  cartesian_mode_pub_ = n_.advertise<std_msgs::Int8>("cartesian_mode", 1);

  ros::service::waitForService("/niryo_one/activate_learning_mode");
  learning_mode_client_ = n_.serviceClient<niryo_one_msgs::SetInt>("/niryo_one/activate_learning_mode");
}

void DeviceWrapper::requestLearningMode(int state)
{
  niryo_one_msgs::SetInt learning_mode;
  learning_mode.request.value = state;
  while (!learning_mode_client_.call(learning_mode))
  {
    ROS_WARN("Could not set learning mode");
    ros::Duration(1.0).sleep();
  }
  ROS_INFO_STREAM("Learning mode successfuly turn " << (state == 1) ? "ON" : "OFF");
}
}
