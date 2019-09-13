/*
 *  TODO copyright
 */
#ifndef CARTESIAN_CONTROLLER_MOVE_GROUP_H
#define CARTESIAN_CONTROLLER_MOVE_GROUP_H

#include "ros/ros.h"

#include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/robot_model_loader/robot_model_loader.h>
// #include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <niryo_one_msgs/RobotMove.h>
#include <niryo_one_msgs/GetInt.h>
#include <std_msgs/Int32.h>

#include <boost/thread/mutex.hpp>

namespace move_group_node
{
class MoveGroupNode
{
public:
  MoveGroupNode();

protected:
private:
  void initializeSubscribers();
  void initializePublishers();
  void initializeServices();
  void initializeActions();
  bool callbackMove(niryo_one_msgs::RobotMove::Request& req, niryo_one_msgs::RobotMove::Response& res);
  bool callbackGetState(niryo_one_msgs::GetInt::Request& req, niryo_one_msgs::GetInt::Response& res);

  ros::NodeHandle n_;
  ros::AsyncSpinner spinner_;

  ros::Publisher state_pub_;
  ros::ServiceServer move_service_;
  ros::ServiceServer get_state_service_;

  moveit::planning_interface::MoveGroupInterface move_group_;
  const robot_state::JointModelGroup* joint_model_group_;

  int state_;
  boost::mutex state_mutex_;
};
}
#endif
