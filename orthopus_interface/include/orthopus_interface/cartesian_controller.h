/*
  TODO copyright
*/

#ifndef CARTESIAN_CONTROLLER_CARTESIAN_CONTROLLER_H
#define CARTESIAN_CONTROLLER_CARTESIAN_CONTROLLER_H

#include <ros/ros.h>

#include "geometry_msgs/TwistStamped.h"

#include "orthopus_interface/inverse_kinematic.h"
#include "orthopus_interface/tool_controller.h"

namespace cartesian_controller
{
class CartesianController
{
public:
  CartesianController();

protected:
private:
  void sendInitCommand();
  void send6DofCommand();

  // Callbacks
  void jointStatesCB(const sensor_msgs::JointStateConstPtr& msg);
  void dxDesCB(const geometry_msgs::TwistStampedPtr& msg);
  void gripperCB(const std_msgs::BoolPtr& msg);

  Vector6d scaleCartesianCommand(const geometry_msgs::TwistStamped& command) const;

  ros::NodeHandle n_;
  ros::Publisher command_pub_;
  ros::Publisher debug_pub_;
  ros::Publisher debug_des_pub_;
  ros::Subscriber joints_sub_;
  ros::Subscriber dx_des_sub_;
  ros::Subscriber gripper_sub_;

  InverseKinematic ik_;
  ToolController tool_controller_;

  double joint_position_cmd[6];
  //   sensor_msgs::JointState q_meas_forced, q_meas_;
  geometry_msgs::TwistStamped dx_des_;

  sensor_msgs::JointState current_joint_state;
  double cartesian_velocity_desired[6];
  bool gripper_state_;
};
}
#endif
