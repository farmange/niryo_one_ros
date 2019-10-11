/*
 *  device_spacenav.h
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
#ifndef CARTESIAN_CONTROLLER_DEVICE_SPACENAV_H
#define CARTESIAN_CONTROLLER_DEVICE_SPACENAV_H

#include <ros/ros.h>

#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/Joy.h"

#include <orthopus_space_control/input_device/device.h>

namespace input_device
{
/**
 * \brief Handle spacenav input device to control the robot
 *
 * Process spacenav_node topic (spacenav/joy) to control robot. Gripper control
 * is performed using services provided by niryo ros stack */
class DeviceSpacenav : public Device
{
public:
  DeviceSpacenav();

private:
  ros::NodeHandle n_;

  bool gripper_toggle_;
  double debounce_button_time_;
  ros::Time debounce_button_left_;
  ros::Time debounce_button_right_;
  int button_left_;
  int button_right_;
  
  void callbackJoy_(const sensor_msgs::Joy::ConstPtr& msg);
  void processButtons_(const sensor_msgs::Joy::ConstPtr& msg);
  void debounceButtons_(const sensor_msgs::Joy::ConstPtr& msg, const int button_id, ros::Time& debounce_timer_ptr,
                        int& button_value_ptr);
  void updateGripperCmd_();
};
}
#endif
