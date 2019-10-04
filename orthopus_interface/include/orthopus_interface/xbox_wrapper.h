/*
 *  xbox_wrapper.h
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
#ifndef CARTESIAN_CONTROLLER_XBOX_WRAPPER_H
#define CARTESIAN_CONTROLLER_XBOX_WRAPPER_H

#include <ros/ros.h>

#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"

#include <orthopus_interface/device_wrapper.h>

namespace cartesian_controller
{
class XboxWrapper : public DeviceWrapper
{
public:
  XboxWrapper();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
  void processButtons(const sensor_msgs::Joy::ConstPtr& msg);
  void debounceButtons(const sensor_msgs::Joy::ConstPtr& msg, const int button_id, ros::Time& debounce_timer_ptr,
                       int& button_value_ptr);
  void updateCartesianMode();

  void updateVelocityFactor();
  void updateGripperCmd();
  void updateLearningMode();

  std_msgs::Bool gripper_cmd_;
  ros::Time debounce_button_a_;
  ros::Time debounce_button_start_;
  ros::Time debounce_button_lb_;
  ros::Time debounce_button_rb_;
  ros::Time debounce_button_b_;
  int button_a_;
  int button_start_;
  int button_lb_;
  int button_rb_;
  int button_b_;
  int learning_mode_;
  double velocity_factor_;
};
}
#endif
