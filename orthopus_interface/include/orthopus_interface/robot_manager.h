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

#include "sensor_msgs/JointState.h"

#include "orthopus_interface/pose_manager.h"
#include "orthopus_interface/cartesian_controller.h"
#include "orthopus_interface/robot_manager_fsm.h"

#include "orthopus_interface/types/joint_position.h"
#include "orthopus_interface/types/space_velocity.h"

#include "orthopus_interface/fsm/state.h"
#include "orthopus_interface/fsm/event.h"
#include "orthopus_interface/fsm/state_disable.h"
#include "orthopus_interface/fsm/state_idle.h"
#include "orthopus_interface/fsm/state_joint_trajectory.h"
#include "orthopus_interface/fsm/state_space_control.h"
#include "orthopus_interface/fsm/state_space_trajectory.h"

namespace cartesian_controller
{
class RobotManager
{
public:
  RobotManager(const int joint_number = 6, const bool use_quaternion = false);
  void init();

protected:
private:
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
  ros::Subscriber dx_input_device_sub_;
  ros::Subscriber learning_mode_sub_;

  ros::ServiceServer action_service_;
  ros::ServiceServer manage_pose_service_;

  PoseManager pose_manager_;
  CartesianController cartesian_controller_;

  int sampling_freq_;
  int learning_mode_;
  int joint_number_;
  bool debug_;
  bool use_quaternion_;
  double sampling_period_;
  double cartesian_max_vel_;
  double pose_goal_joints_tolerance_;

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

  void handleInput(Event event);
  void update();

  friend class StateDisable;
  friend class StateIdle;
  friend class StateJointTrajectory;
  friend class StateSpaceControl;
  friend class StateSpaceTrajectory;
  State* state_;
  Event event_;

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

  void initializeSubscribers_();
  void initializePublishers_();
  void initializeServices_();
  void retrieveParameters_();

  // Callbacks
  bool callbackAction_(niryo_one_msgs::SetInt::Request& req, niryo_one_msgs::SetInt::Response& res);
  void callbackLearningMode_(const std_msgs::BoolPtr& msg);
  void callbackJointState_(const sensor_msgs::JointStateConstPtr& msg);
  void callbackInputDeviceVelocity_(const geometry_msgs::TwistStampedPtr& msg);

  void updateFsm_();
  void printFsm_();
  void runFsm_();

  void sendJointsCommand_();
  bool cartesianIsEnable_();

  void gotoPosition_(const JointPosition position);
  double computeDuration_(const JointPosition position);
  bool isPositionCompleted_(const JointPosition position);
};
}
#endif
