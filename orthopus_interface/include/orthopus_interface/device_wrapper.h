/*
 *  device_wrapper.h
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
#ifndef CARTESIAN_CONTROLLER_DEVICE_WRAPPER_H
#define CARTESIAN_CONTROLLER_DEVICE_WRAPPER_H

#include <ros/ros.h>

// #include "geometry_msgs/TwistStamped.h"
// #include "sensor_msgs/Joy.h"
// #include "std_msgs/Bool.h"
namespace cartesian_controller
{
class DeviceWrapper
{
public:
  DeviceWrapper();

  /** Summarises possible plan to navigate. */
  typedef enum {
    XY, /**< Navigate in the XY plan */
    YZ, /**< Navigate in the YZ plan */
    XZ  /**< Navigate in the XZ plan */
  } CartesianMode_t;
  //    CartesianMode CartesianMode_t;

protected:
  void requestLearningMode(int state);

  ros::NodeHandle n_;
  // Subscribed topic from where device inputs are read
  ros::Subscriber device_sub_;
  // Publish
  ros::Publisher cartesian_cmd_pub_;
  ros::Publisher gripper_cmd_pub_;
  ros::Publisher cartesian_mode_pub_;

  ros::ServiceClient learning_mode_client_;

  CartesianMode_t cartesian_mode_;
};
}

#endif
