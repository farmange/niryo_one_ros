/*
  TODO copyright
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
  void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
  void processButtons(const sensor_msgs::Joy::ConstPtr& msg);
  void debounceButtons(const sensor_msgs::Joy::ConstPtr& msg, const int button_id, ros::Time& debounce_timer_ptr,
                       int& button_value_ptr);
  void updateCartesianMode();

  void updateVelocityFactor();
  void updateGripperCmd();
  void updateLearningMode();

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
