/*
 *  device_web_app.h
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
#ifndef CARTESIAN_CONTROLLER_DEVICE_WEB_APP_H
#define CARTESIAN_CONTROLLER_DEVICE_WEB_APP_H

#include <ros/ros.h>

#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"

#include <orthopus_space_control/input_device/device.h>
#include <orthopus_space_control/input_device/xbox_config.h>

namespace input_device
{
/**
 * \brief Handle web app interface to control the robot
 *
 * This is in fact just a binding of Xbox input device. Indeed, the web app 
 * interface publish in a sensor_msgs/Joy topic exactly as Xbox controller 
 * did (same axis index). However, the web app directly handle gripper control.
 * The web interface is based on rosbridge package through roslib.js script.
 */
class DeviceWebApp : public Device
{
public:
  DeviceWebApp();

private:
  ros::NodeHandle n_;

  void callbackJoy_(const sensor_msgs::Joy::ConstPtr& msg);
};
}
#endif
