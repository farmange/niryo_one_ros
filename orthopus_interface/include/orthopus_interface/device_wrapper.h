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
