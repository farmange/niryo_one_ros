/*
 *  robot_manager.h
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
#ifndef CARTESIAN_CONTROLLER_ROBOT_MANAGER_H
#define CARTESIAN_CONTROLLER_ROBOT_MANAGER_H

#include "ros/ros.h"

// #include "sensor_msgs/JointState.h"
#include "orthopus_interface/pose_manager.h"
#include "orthopus_interface/cartesian_controller.h"
#include "orthopus_interface/robot_manager_fsm.h"

#include "orthopus_interface/types/joint_position.h"
#include "orthopus_interface/types/space_velocity.h"

namespace cartesian_controller
{
class RobotManager
{
public:
  RobotManager(const int joint_number = 6, const bool use_quaternion = false);
  void init();

protected:
private:
  void initializeSubscribers();
  void initializePublishers();
  void initializeServices();
  void retrieveParameters();

  // Callbacks
  bool callbackAction(niryo_one_msgs::SetInt::Request& req, niryo_one_msgs::SetInt::Response& res);
  void callbackLearningMode(const std_msgs::BoolPtr& msg);
  void callbackJointState(const sensor_msgs::JointStateConstPtr& msg);
  void callbackVelocitiesDesired(const geometry_msgs::TwistStampedPtr& msg);

  void updateFsm();
  void printFsm();
  void runFsm();

  ros::NodeHandle n_;

  ros::Publisher command_pub_;
  ros::Publisher joystick_enabled_pub_;
  ros::Publisher debug_pose_current_;
  ros::Publisher debug_pose_desired_;
  ros::Publisher debug_pose_meas_;
  ros::Publisher debug_joint_desired_;
  ros::Publisher debug_joint_min_limit_;
  ros::Publisher debug_joint_max_limit_;

  ros::Subscriber joints_sub_;
  ros::Subscriber dx_des_sub_;
  ros::Subscriber learning_mode_sub_;

  ros::ServiceServer action_service_;
  ros::ServiceServer cartesian_enable_service_;
  ros::ServiceServer manage_pose_service_;

  PoseManager pose_manager_;
  CartesianController cartesian_controller_;

  int sampling_freq_;
  double sampling_period_;
  int learning_mode_;

  JointPosition q_command_;
  JointPosition q_current_;
  JointPosition q_meas_;

  SpaceVelocity dx_desired_;
  SpaceVelocity dx_desired_prev_;

  RobotManagerFsmAction action_requested_;
  RobotManagerFsmState fsm_state_;
  RobotManagerFsmState fsm_prev_state_;

  geometry_msgs::Pose drink_pose_;
  geometry_msgs::Pose stand_pose_;

  bool use_quaternion_;
  int joint_number_;

  double pose_goal_joints_tolerance_;

  void cartesian_State();
  void cartesian_Entry();
  void cartesian_Exit();

  void gotoHome_State();
  void gotoHome_Entry();
  void gotoHome_Exit();

  void gotoRest_State();
  void gotoRest_Entry();
  void gotoRest_Exit();

  void gotoDrink_State();
  void gotoDrink_Entry();
  void gotoDrink_Exit();

  void gotoStandGlass_State();
  void gotoStandGlass_Entry();
  void gotoStandGlass_Exit();

  void flipPinch_State();
  void flipPinch_Entry();
  void flipPinch_Exit();

  void sendJointsCommand();
  bool cartesianIsEnable();

  void gotoPosition(const JointPosition position);
  double computeDuration(const JointPosition position);
  bool isPositionCompleted(const JointPosition position);
};
}
#endif
