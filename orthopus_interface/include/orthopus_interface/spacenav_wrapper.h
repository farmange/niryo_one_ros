/*
 *  spacenav_wrapper.h
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
#ifndef CARTESIAN_CONTROLLER_SPACENAV_WRAPPER_H
#define CARTESIAN_CONTROLLER_SPACENAV_WRAPPER_H

#include <ros/ros.h>

#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"

#include <orthopus_interface/device_wrapper.h>

namespace cartesian_controller
{
class SpacenavWrapper : public DeviceWrapper
{
public:
  SpacenavWrapper();

private:
  ros::NodeHandle n_;

  ros::ServiceClient change_tool_srv_;
  ros::ServiceClient open_gripper_srv_;
  ros::ServiceClient close_gripper_srv_;

  void joyCallback_(const sensor_msgs::Joy::ConstPtr& msg);
  void processButtons_(const sensor_msgs::Joy::ConstPtr& msg);
  void debounceButtons_(const sensor_msgs::Joy::ConstPtr& msg, const int button_id, ros::Time& debounce_timer_ptr,
                        int& button_value_ptr);
  void initializeServices_();

  void updateVelocityFactor_();
  void updateGripperCmd_();
  void updateLearningMode_();

  void setGripperId_();
  void openGripper_();
  void closeGripper_();

  std_msgs::Bool gripper_cmd_;
  ros::Time debounce_button_left_;
  ros::Time debounce_button_right_;
  int button_left_;
  int button_right_;
  int learning_mode_;
  double velocity_factor_;
};
}
#endif
