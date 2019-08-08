/*
  TODO copyright
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

protected:
  void requestLearningMode(int state);

  ros::NodeHandle n_;
  // Subscribed topic from where device inputs are read
  ros::Subscriber device_sub_;
  // Publish
  ros::Publisher cartesian_cmd_pub_;
  ros::Publisher gripper_cmd_pub_;
  ros::ServiceClient learning_mode_client_;
};
}

#endif
