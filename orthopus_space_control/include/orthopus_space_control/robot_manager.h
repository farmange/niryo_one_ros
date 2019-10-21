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

#include "orthopus_space_control/cartesian_controller.h"
#include "orthopus_space_control/pose_manager.h"
#include "orthopus_space_control/robot_manager_fsm.h"

#include "orthopus_space_control/types/joint_position.h"
#include "orthopus_space_control/types/space_velocity.h"

#include "orthopus_space_control/fsm/engine.h"
#include "orthopus_space_control/fsm/state.h"
#include "orthopus_space_control/fsm/transition.h"

namespace space_control
{
class RobotManager
{
public:
  RobotManager(const int joint_number = 6, const bool use_quaternion = false, const bool debug = false);
  void init();

protected:
private:
  ros::NodeHandle n_;
  ros::Publisher command_pub_;
  ros::Publisher joystick_enabled_pub_;
  ros::Publisher q_current_debug_pub_;
  ros::Publisher x_current_debug_pub_;
  ros::Publisher dx_desired_debug_pub_;
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

  SpacePosition x_drink_pose_;
  SpacePosition x_stand_pose_;

  SpaceVelocity dx_desired_;
  SpaceVelocity dx_desired_prev_;

  /* FSM engine */
  Engine<RobotManager>* engine_;

  /* FSM state */
  State<RobotManager>* state_idle_;
  State<RobotManager>* state_disable_;
  State<RobotManager>* state_joint_home_;
  State<RobotManager>* state_joint_rest_;
  State<RobotManager>* state_traj_drink_;
  State<RobotManager>* state_traj_stand_;
  State<RobotManager>* state_flip_pinch_;
  State<RobotManager>* state_space_control_;

  void spaceControlUpdate_();
  void spaceControlEnter_();
  void jointHomeEnter_();
  void jointRestEnter_();
  void trajDrinkUpdate_();
  void trajDrinkEnter_();
  void trajStandUpdate_();
  void trajStandEnter_();
  void flipPinchEnter_();

  /* FSM Transition */
  Transition<RobotManager>* tr_all_to_disable_;
  Transition<RobotManager>* tr_disable_to_idle_;
  Transition<RobotManager>* tr_rest_to_idle_;
  Transition<RobotManager>* tr_to_joint_home_;
  Transition<RobotManager>* tr_to_joint_rest_;
  Transition<RobotManager>* tr_to_traj_drink_;
  Transition<RobotManager>* tr_to_traj_stand_;
  Transition<RobotManager>* tr_to_flip_pinch_;
  Transition<RobotManager>* tr_flip_pinch_to_space_control_;
  Transition<RobotManager>* tr_joint_home_to_space_control_;

  bool trDisableToIdle_();
  bool trAllToDisable_();
  bool trToJointHome_();
  bool trToJointRest_();
  bool trToTrajDrink_();
  bool trToTrajStand_();
  bool trToFlipPinch_();
  bool trJointHomeToSpaceControl_();
  bool trFlipPinchToSpaceControl_();
  bool trRestToIdle_();

  /* Init methods */
  void initializeSubscribers_();
  void initializePublishers_();
  void initializeServices_();
  void retrieveParameters_();
  void initializeStateMachine_();

  /* FSM input event (what is allowed to do from user point of view) */
  FsmInputEvent input_event_requested_;

  // Callbacks
  bool callbackAction_(niryo_one_msgs::SetInt::Request& req, niryo_one_msgs::SetInt::Response& res);
  void callbackLearningMode_(const std_msgs::BoolPtr& msg);
  void callbackJointState_(const sensor_msgs::JointStateConstPtr& msg);
  void callbackInputDeviceVelocity_(const geometry_msgs::TwistStampedPtr& msg);

  void gotoPosition_(const JointPosition position) const;
  bool isPositionCompleted_(const JointPosition position) const;
  double computeDuration_(const JointPosition position) const;
  void sendJointsCommand_() const;
};
}
#endif
