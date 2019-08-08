/*
  TODO copyright
*/

#ifndef CARTESIAN_CONTROLLER_TOOL_CONTROLLER_H
#define CARTESIAN_CONTROLLER_TOOL_CONTROLLER_H

#include <ros/ros.h>

// #include "geometry_msgs/TwistStamped.h"
// #include "sensor_msgs/Joy.h"
#include <actionlib/client/simple_action_client.h>
#include "niryo_one_msgs/RobotMoveAction.h"

namespace cartesian_controller
{
typedef actionlib::SimpleActionClient<niryo_one_msgs::RobotMoveAction> NiryoClient;

class ToolController
{
public:
  ToolController();

  void setToolId(int id_);
  // Send gripper command to action server, only if the state is update
  void sendGripperCommand(bool new_state);
protected:
  
private:
  void sendOpenGripperCommand();
  void sendCloseGripperCommand();

  
  ros::NodeHandle n_;
  NiryoClient ac_;
  
  int tool_id_;
  // Close = true / Open = false
  bool tool_state_;

};
}
#endif
