/*
 *  device.h
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
#ifndef CARTESIAN_CONTROLLER_DEVICE_H
#define CARTESIAN_CONTROLLER_DEVICE_H

#include <ros/ros.h>

/* TODO Improve multiple device handling. Currently, all device publish at the
same time in the same topic device_sub_ which is nor safe nor expected behavior */
namespace input_device
{
/**
  * \brief Abstract interface for devices implementation
  *
  * This class prepares the geometry_msgs/TwistStamped topic expected by
  * the robot manager for cartesian control
  */
class Device
{
public:
  Device();
  virtual ~Device() = 0;

protected:
  ros::NodeHandle n_;
  ros::Subscriber device_sub_;
  ros::Publisher cartesian_cmd_pub_;
  ros::ServiceClient learning_mode_client_;
  ros::ServiceClient change_tool_srv_;
  ros::ServiceClient open_gripper_srv_;
  ros::ServiceClient close_gripper_srv_;

  void initializeServices_();
  void requestLearningMode(int state);
  void setGripperId_();
  void openGripper_();
  void closeGripper_();
};
}

#endif
