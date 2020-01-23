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

#include "orthopus_space_control/SetRobotAction.h"
#include "orthopus_space_control/SetUInt16.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/UInt16.h"

#include "orthopus_space_control/cartesian_controller.h"
#include "orthopus_space_control/joint_pose_manager.h"
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
  RobotManager(const int joint_number = 6, const bool debug = false);
  void init();

protected:
private:
  ros::NodeHandle n_;
  ros::Publisher command_pub_;
  ros::Publisher joystick_enabled_pub_;
  ros::Publisher q_current_debug_pub_;
  ros::Publisher x_current_debug_pub_;
  ros::Publisher dx_desired_debug_pub_;
  ros::Publisher control_feedback_pub_;
  ros::Subscriber joints_sub_;
  ros::Subscriber dx_input_device_sub_;
  ros::Subscriber learning_mode_sub_;
  ros::ServiceServer set_robot_action_service_;
  ros::ServiceServer manage_position_service_;
  ros::ServiceServer get_position_list_service_;
  ros::ServiceServer set_control_frame_service_;

  JointPoseManager joint_pose_manager_;
  CartesianController cartesian_controller_;

  int sampling_freq_;
  int learning_mode_;
  int joint_number_;
  bool debug_;
  double sampling_period_;
  double joint_max_vel_;
  double space_position_max_vel_;
  double space_orientation_max_vel_;
  double goal_joint_tolerance_;
  uint16_t set_control_frame_requested_;
  ros::Time joint_position_timer_;

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
  State<RobotManager>* state_joint_position_;
  State<RobotManager>* state_space_position_;
  State<RobotManager>* state_space_control_;

  /* FSM Transitions */
  Transition<RobotManager>* tr_all_to_disable_;
  Transition<RobotManager>* tr_disable_to_idle_;
  Transition<RobotManager>* tr_to_joint_position_;
  Transition<RobotManager>* tr_to_space_position_;
  Transition<RobotManager>* tr_joint_position_to_space_control_;
  Transition<RobotManager>* tr_space_position_to_space_control_;

  bool trDisableToIdle_();
  bool trAllToDisable_();
  bool trToJointPosition_();
  bool trToSpacePosition_();
  bool trJointPositionToSpaceControl_();
  bool trSpacePositionToSpaceControl_();

  /* FSM functions */
  void disableUpdate_();
  void idleUpdate_();
  void spaceControlUpdate_();
  void spaceControlEnter_();
  void jointPositionExit_();
  void jointPositionEnter_();
  void spacePositionUpdate_();
  void spacePositionEnter_();

  /* Init methods */
  void initializeSubscribers_();
  void initializePublishers_();
  void initializeServices_();
  void retrieveParameters_();
  void initializeStateMachine_();

  /* FSM input event (what is allowed to do from user point of view) */
  FsmInputEvent input_event_requested_;
  std::string position_requested_;
  // Callbacks
  bool callbackRobotAction_(orthopus_space_control::SetRobotAction::Request& req,
                            orthopus_space_control::SetRobotAction::Response& res);
  bool callbackSetControlFrame_(orthopus_space_control::SetUInt16::Request& req,
                                orthopus_space_control::SetUInt16::Response& res);
  bool callbackManagePosition_(niryo_one_msgs::ManagePosition::Request& req,
                               niryo_one_msgs::ManagePosition::Response& res);
  bool callbackGetPositionList_(niryo_one_msgs::GetPositionList::Request& req,
                                niryo_one_msgs::GetPositionList::Response& res);

  void callbackLearningMode_(const std_msgs::BoolPtr& msg);
  void callbackJointState_(const sensor_msgs::JointStateConstPtr& msg);
  void callbackInputDeviceVelocity_(const geometry_msgs::TwistStampedPtr& msg);

  void gotoPosition_(const JointPosition position);
  bool isPositionCompleted_(const JointPosition position) const;
  double computeDuration_(const JointPosition position) const;
  void sendJointsCommand_() const;
  bool isUserVelocityReceive() const;
};
}
#endif
