/*
 *  robot_manager.cpp
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
#include "ros/ros.h"

#include "orthopus_interface/robot_manager.h"
#include "niryo_one_msgs/CloseGripper.h"

namespace cartesian_controller
{
void RobotManager::update()
{
  ROS_DEBUG_STREAM("RobotManager::update");
  ROS_DEBUG_STREAM(state_);
  
  state_->update(*this);
}

void RobotManager::handleInput(Event event)
{
  ROS_DEBUG_STREAM("RobotManager::handleInput");
  
  State* state = state_->handleInput(*this, event);
  if (state != NULL)
  {
    // Call the enter action on the new state.
    state_->exit(*this);

    delete state_;
    state_ = state;
    
    // Call the enter action on the new state.
    state_->enter(*this);
  }
}

RobotManager::RobotManager(const int joint_number, const bool use_quaternion)
  : cartesian_controller_(joint_number, use_quaternion)
  , pose_manager_(joint_number, use_quaternion)
  , joint_number_(joint_number)
  , use_quaternion_(use_quaternion)
  , q_command_(joint_number)
  , q_current_(joint_number)
  , q_meas_(joint_number)
  , dx_desired_(use_quaternion)
  , dx_desired_prev_(use_quaternion)
{
  ROS_DEBUG_STREAM("***** FSM TEST *****");
  state_ = new StateDisable();
  event_ = Event::None;
 
  ROS_DEBUG_STREAM("RobotManager constructor");
  retrieveParameters_();
  initializeSubscribers_();
  initializePublishers_();
  initializeServices_();
  ros::Rate loop_rate = ros::Rate(sampling_freq_);
  fsm_state_ = RobotManagerFsmState::Disable;

  // Wait for initial messages
  ROS_INFO("Waiting for first joint msg.");
  ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states");
  ROS_INFO("Received first joint msg.");

  if (sampling_freq_ > 0)
  {
    sampling_period_ = 1.0 / sampling_freq_;
  }
  else
  {
    ROS_ERROR("Sampling frequency could not be lower or equal to zero : %d", sampling_freq_);
    ros::shutdown();
  }

  init();

  while (ros::ok())
  {
    ros::spinOnce();

    updateFsm_();
    runFsm_();

//     handleInput(event_);    
//     update();
    
    /* This topic publication prevents niryo "RobotCommander" to execute action */
    std_msgs::Bool joystick_enable_msg;
    joystick_enable_msg.data = true;
    joystick_enabled_pub_.publish(joystick_enable_msg);

    loop_rate.sleep();
  }
}

void RobotManager::init()
{
  cartesian_controller_.init(sampling_period_, pose_manager_);
}

void RobotManager::callbackInputDeviceVelocity_(const geometry_msgs::TwistStampedPtr& msg)
{
  //   ROS_DEBUG_STREAM("callbackInputDeviceVelocity");
  for (int i = 0; i < dx_desired_.size(); i++)
  {
    dx_desired_[i] = 0.0;
  }

  dx_desired_[0] = msg->twist.linear.x;
  dx_desired_[1] = msg->twist.linear.y;
  dx_desired_[2] = msg->twist.linear.z;

  dx_desired_[3] = msg->twist.angular.x;
  dx_desired_[4] = msg->twist.angular.y;
  dx_desired_[5] = msg->twist.angular.z;

  for (int i = 0; i < 3; i++)
  {
    if (dx_desired_[i] != 0)
    {
      cartesian_controller_.getInverseKinematic()->requestUpdateAxisConstraints(i, 1.0);
    }
    else if (dx_desired_[i] == 0 && dx_desired_prev_[i] != 0)
    {
      cartesian_controller_.getInverseKinematic()->requestUpdateAxisConstraints(i, 0.001);
    }
    dx_desired_prev_[i] = dx_desired_[i];
  }
  for (int i = 3; i < 6; i++)
  {
    if (dx_desired_[i] != 0)
    {
      cartesian_controller_.getInverseKinematic()->requestUpdateAxisConstraints(i, 1.0);
    }
    else if (dx_desired_[i] == 0 && dx_desired_prev_[i] != 0)
    {
      cartesian_controller_.getInverseKinematic()->requestUpdateAxisConstraints(i, 0.001);
    }
    dx_desired_prev_[i] = dx_desired_[i];
  }
}

void RobotManager::callbackLearningMode_(const std_msgs::BoolPtr& msg)
{
  //   ROS_DEBUG_STREAM("callbackLearningMode");
  learning_mode_ = msg->data;
}

void RobotManager::callbackJointState_(const sensor_msgs::JointStateConstPtr& msg)
{
  //   ROS_DEBUG_STREAM("callbackJointState");
  for (int i = 0; i < joint_number_; i++)
  {
    // TODO check that data exists for all joint
    q_meas_[i] = msg->position[i];
  }
}

bool RobotManager::callbackAction_(niryo_one_msgs::SetInt::Request& req, niryo_one_msgs::SetInt::Response& res)
{
  if (req.value == RobotManagerFsmAction::Cartesian)
  {
//     event_ = Event::ExecuteSpaceControl;
    action_requested_ = RobotManagerFsmAction::Cartesian;
  }
  else if (req.value == RobotManagerFsmAction::GotoHome)
  {
//     event_ = Event::ExecuteJointTraj;
    action_requested_ = RobotManagerFsmAction::GotoHome;
  }
  else if (req.value == RobotManagerFsmAction::GotoRest)
  {
//     event_ = Event::ExecuteJointTraj;
    action_requested_ = RobotManagerFsmAction::GotoRest;
  }
  else if (req.value == RobotManagerFsmAction::GotoDrink)
  {
//     event_ = Event::ExecuteSpaceTraj;
    action_requested_ = RobotManagerFsmAction::GotoDrink;
  }
  else if (req.value == RobotManagerFsmAction::GotoStandGlass)
  {
//     event_ = Event::ExecuteSpaceTraj;
    action_requested_ = RobotManagerFsmAction::GotoStandGlass;
  }
  else if (req.value == RobotManagerFsmAction::FlipPinch)
  {
//     event_ = Event::ExecuteJointTraj;
    action_requested_ = RobotManagerFsmAction::FlipPinch;
  }
  else
  {
    /* Keep previous requested action */
    ROS_ERROR("Action requested unknown : %d", req.value);
  }
  return true;
}

void RobotManager::initializeSubscribers_()
{
  ROS_DEBUG_STREAM("RobotManager initializeSubscribers");
  joints_sub_ = n_.subscribe("joint_states", 1, &RobotManager::callbackJointState_, this);
  dx_input_device_sub_ =
      n_.subscribe("/orthopus_interface/input_device_velocity", 1, &RobotManager::callbackInputDeviceVelocity_, this);
  learning_mode_sub_ = n_.subscribe("/niryo_one/learning_mode", 1, &RobotManager::callbackLearningMode_, this);
}

void RobotManager::initializePublishers_()
{
  ROS_DEBUG_STREAM("RobotManager initializePublishers");

  command_pub_ =
      n_.advertise<trajectory_msgs::JointTrajectory>("/niryo_one_follow_joint_trajectory_controller/command", 1);
  joystick_enabled_pub_ = n_.advertise<std_msgs::Bool>("/niryo_one/joystick_interface/is_enabled", 1);

  if (debug_)
  {
    debug_pose_current_ = n_.advertise<geometry_msgs::Pose>("/debug_pose_current", 1);
    debug_pose_desired_ = n_.advertise<geometry_msgs::Pose>("/debug_pose_desired", 1);
    debug_pose_meas_ = n_.advertise<geometry_msgs::Pose>("/debug_pose_meas", 1);

    debug_joint_desired_ = n_.advertise<sensor_msgs::JointState>("/debug_joint_desired", 1);
    debug_joint_min_limit_ = n_.advertise<sensor_msgs::JointState>("/debug_joint_min_limit", 1);
    debug_joint_max_limit_ = n_.advertise<sensor_msgs::JointState>("/debug_joint_max_limit", 1);
  }
}

void RobotManager::initializeServices_()
{
  ROS_DEBUG_STREAM("RobotManager initializeServices");
  // TODO : refactor :better name for this service (not action)
  action_service_ = n_.advertiseService("/niryo_one/orthopus_interface/action", &RobotManager::callbackAction_, this);
  manage_pose_service_ = n_.advertiseService("/niryo_one/orthopus_interface/manage_pose",
                                             &PoseManager::callbackManagePose, &pose_manager_);
}

void RobotManager::retrieveParameters_()
{
  ros::param::get("~drink_pose/position/x", drink_pose_.position.x);
  ros::param::get("~drink_pose/position/y", drink_pose_.position.y);
  ros::param::get("~drink_pose/position/z", drink_pose_.position.z);
  ros::param::get("~drink_pose/orientation/x", drink_pose_.orientation.x);
  ros::param::get("~drink_pose/orientation/y", drink_pose_.orientation.y);
  ros::param::get("~drink_pose/orientation/z", drink_pose_.orientation.z);

  ros::param::get("~stand_pose/position/x", stand_pose_.position.x);
  ros::param::get("~stand_pose/position/y", stand_pose_.position.y);
  ros::param::get("~stand_pose/position/z", stand_pose_.position.z);
  ros::param::get("~stand_pose/orientation/x", stand_pose_.orientation.x);
  ros::param::get("~stand_pose/orientation/y", stand_pose_.orientation.y);
  ros::param::get("~stand_pose/orientation/z", stand_pose_.orientation.z);

  ros::param::get("~pose_goal_joints_tolerance", pose_goal_joints_tolerance_);
  ros::param::get("~sampling_frequency", sampling_freq_);
  ros::param::get("~cartesian_max_vel", cartesian_max_vel_);
  ros::param::get("~debug", debug_);
}

void RobotManager::printFsm_()
{
  ROS_DEBUG_STREAM("FSM state : " << fsm_state_.ToString() << " - Action requested : " << action_requested_.ToString());
}

void RobotManager::updateFsm_()
{
  RobotManagerFsmState fsm_local_prev_state = fsm_state_;

  /* This condition is valid for all state */
  if (learning_mode_ == 1)
  {
    fsm_state_ = RobotManagerFsmState::Disable;
  }
  else
  {
    if (fsm_state_ == RobotManagerFsmState::Disable)
    {
      if (learning_mode_ == 0)
      {
        fsm_state_ = RobotManagerFsmState::Idle;
      }
    }
    else if (fsm_state_ == RobotManagerFsmState::Idle)
    {
      if (action_requested_ == RobotManagerFsmAction::GotoHome)
      {
        fsm_state_ = RobotManagerFsmState::GotoHome;
        gotoHome_Entry();
      }
      else if (action_requested_ == RobotManagerFsmAction::GotoRest)
      {
        fsm_state_ = RobotManagerFsmState::GotoRest;
        gotoRest_Entry();
      }
    }
    else if (fsm_state_ == RobotManagerFsmState::CartesianMode)
    {
      if (action_requested_ == RobotManagerFsmAction::GotoDrink)
      {
        fsm_state_ = RobotManagerFsmState::GotoDrink;
        gotoDrink_Entry();
      }
      else if (action_requested_ == RobotManagerFsmAction::GotoStandGlass)
      {
        fsm_state_ = RobotManagerFsmState::GotoStandGlass;
        gotoStandGlass_Entry();
      }
      else if (action_requested_ == RobotManagerFsmAction::GotoHome)
      {
        fsm_state_ = RobotManagerFsmState::GotoHome;
        gotoHome_Entry();
      }
      else if (action_requested_ == RobotManagerFsmAction::GotoRest)
      {
        fsm_state_ = RobotManagerFsmState::GotoRest;
        gotoRest_Entry();
      }
    }
    else if (fsm_state_ == RobotManagerFsmState::GotoDrink)
    {
      if (action_requested_ == RobotManagerFsmAction::GotoHome)
      {
        fsm_state_ = RobotManagerFsmState::GotoHome;
        gotoHome_Entry();
      }
      else if (action_requested_ == RobotManagerFsmAction::GotoRest)
      {
        fsm_state_ = RobotManagerFsmState::GotoRest;
        gotoRest_Entry();
      }
      else if (cartesian_controller_.getTrajectoryController()->isTrajectoryCompleted())
      {
        fsm_state_ = RobotManagerFsmState::FlipPinch;
        flipPinch_Entry();
      }
    }
    else if (fsm_state_ == RobotManagerFsmState::GotoStandGlass)
    {
      if (action_requested_ == RobotManagerFsmAction::GotoHome)
      {
        fsm_state_ = RobotManagerFsmState::GotoHome;
        gotoHome_Entry();
      }
      else if (action_requested_ == RobotManagerFsmAction::GotoRest)
      {
        fsm_state_ = RobotManagerFsmState::GotoRest;
        gotoRest_Entry();
      }
      else if (cartesian_controller_.getTrajectoryController()->isTrajectoryCompleted())
      {
        fsm_state_ = RobotManagerFsmState::FlipPinch;
        flipPinch_Entry();
      }
    }
    else if (fsm_state_ == RobotManagerFsmState::GotoHome)
    {
      if (isPositionCompleted_(pose_manager_.getJoints("Home")))
      {
        fsm_state_ = RobotManagerFsmState::CartesianMode;
        cartesian_Entry();
      }
      else if (action_requested_ == RobotManagerFsmAction::GotoHome)
      {
        fsm_state_ = RobotManagerFsmState::GotoHome;
        gotoHome_Entry();
      }
      else if (action_requested_ == RobotManagerFsmAction::GotoRest)
      {
        fsm_state_ = RobotManagerFsmState::GotoRest;
        gotoRest_Entry();
      }
    }
    else if (fsm_state_ == RobotManagerFsmState::GotoRest)
    {
      if (isPositionCompleted_(pose_manager_.getJoints("Rest")))
      {
        fsm_state_ = RobotManagerFsmState::Idle;
      }
      else if (action_requested_ == RobotManagerFsmAction::GotoHome)
      {
        fsm_state_ = RobotManagerFsmState::GotoHome;
        gotoHome_Entry();
      }
      else if (action_requested_ == RobotManagerFsmAction::GotoRest)
      {
        fsm_state_ = RobotManagerFsmState::GotoRest;
        gotoRest_Entry();
      }
    }
    else if (fsm_state_ == RobotManagerFsmState::FlipPinch)
    {
      if (action_requested_ == RobotManagerFsmAction::GotoHome)
      {
        fsm_state_ = RobotManagerFsmState::GotoHome;
        gotoHome_Entry();
      }
      else if (action_requested_ == RobotManagerFsmAction::GotoRest)
      {
        fsm_state_ = RobotManagerFsmState::GotoRest;
        gotoRest_Entry();
      }
      else if (isPositionCompleted_(pose_manager_.getJoints("Flip")))
      {
        fsm_state_ = RobotManagerFsmState::CartesianMode;
        cartesian_Entry();
      }
    }
    else
    {
      ROS_ERROR_STREAM("Unkown fsm state : " << fsm_state_.ToString());
      exit(0);
    }
  }

  if (fsm_local_prev_state != fsm_state_)
  {
    fsm_prev_state_ = fsm_state_;
  }

  action_requested_ = RobotManagerFsmAction::None;
}

void RobotManager::runFsm_()
{
  printFsm_();
  // TODO could be generic function
  if (fsm_state_ == RobotManagerFsmState::CartesianMode)
  {
    cartesian_State();
  }
  else if (fsm_state_ == RobotManagerFsmState::GotoDrink)
  {
    gotoDrink_State();
  }
  else if (fsm_state_ == RobotManagerFsmState::GotoStandGlass)
  {
    gotoStandGlass_State();
  }
  else if (fsm_state_ == RobotManagerFsmState::FlipPinch || fsm_state_ == RobotManagerFsmState::GotoRest ||
           fsm_state_ == RobotManagerFsmState::GotoHome)
  {
    /* Do nothing */
  }
  else if (fsm_state_ == RobotManagerFsmState::Disable)
  {
    /* Do nothing */
  }
  else if (fsm_state_ == RobotManagerFsmState::Idle)
  {
    /* Do nothing */
  }
  else
  {
    ROS_ERROR_STREAM("Unkown fsm state : " << fsm_state_.ToString());
    exit(0);
  }
}

/*******************************************/
void RobotManager::cartesian_State()
{
  ROS_INFO("=== Update joint position (Open loop)...");
  q_current_ = q_command_;
  ROS_INFO("    Done.");

  cartesian_controller_.setDxDesired(dx_desired_);
  cartesian_controller_.setInputSelector(CartesianController::INPUT_USER);
  cartesian_controller_.run(q_current_, q_command_);

  ROS_INFO("=== Send Niryo One commands...");
  sendJointsCommand_();
  ROS_INFO("    Done.");
}
void RobotManager::cartesian_Entry()
{
  /* Switch to cartesian mode when position is completed */
  cartesian_controller_.reset();
  q_command_ = q_meas_;
}
void RobotManager::cartesian_Exit()
{
}
/*******************************************/
void RobotManager::gotoHome_State()
{
}
void RobotManager::gotoHome_Entry()
{
  gotoPosition_(pose_manager_.getJoints("Home"));
}
void RobotManager::gotoHome_Exit()
{
}
/*******************************************/
void RobotManager::gotoRest_State()
{
}
void RobotManager::gotoRest_Entry()
{
  gotoPosition_(pose_manager_.getJoints("Rest"));
}
void RobotManager::gotoRest_Exit()
{
}
/*******************************************/
void RobotManager::gotoDrink_State()
{
  ROS_INFO("=== Update joint position (Open loop)...");
  // TODO check q_current consistency (many affectation could lead to issue
  q_current_ = q_command_;
  ROS_INFO("    Done.");

  cartesian_controller_.getInverseKinematic()->requestUpdateAxisConstraints(0, 0.1);
  cartesian_controller_.getInverseKinematic()->requestUpdateAxisConstraints(1, 0.1);
  cartesian_controller_.getInverseKinematic()->requestUpdateAxisConstraints(2, 0.1);
  cartesian_controller_.setDxDesired(dx_desired_);
  cartesian_controller_.setInputSelector(CartesianController::INPUT_TRAJECTORY);
  cartesian_controller_.run(q_current_, q_command_);

  ROS_INFO("=== Send Niryo One commands...");
  sendJointsCommand_();
  ROS_INFO("    Done.");
}

void RobotManager::gotoDrink_Entry()
{
  cartesian_controller_.reset();
  q_command_ = q_meas_;
  cartesian_controller_.getTrajectoryController()->setTrajectoryPose(drink_pose_);
}

void RobotManager::gotoDrink_Exit()
{
}
/*******************************************/
void RobotManager::gotoStandGlass_State()
{
  ROS_INFO("=== Update joint position (Open loop)...");
  q_current_ = q_command_;
  ROS_INFO("    Done.");

  cartesian_controller_.getInverseKinematic()->requestUpdateAxisConstraints(0, 0.1);
  cartesian_controller_.getInverseKinematic()->requestUpdateAxisConstraints(1, 0.1);
  cartesian_controller_.getInverseKinematic()->requestUpdateAxisConstraints(2, 0.1);
  cartesian_controller_.setDxDesired(dx_desired_);
  cartesian_controller_.setInputSelector(CartesianController::INPUT_TRAJECTORY);
  cartesian_controller_.run(q_current_, q_command_);

  ROS_INFO("=== Send Niryo One commands...");
  sendJointsCommand_();
  ROS_INFO("    Done.");
}

void RobotManager::gotoStandGlass_Entry()
{
  cartesian_controller_.reset();
  q_command_ = q_meas_;
  cartesian_controller_.getTrajectoryController()->setTrajectoryPose(stand_pose_);
}

void RobotManager::gotoStandGlass_Exit()
{
}
/*******************************************/
void RobotManager::flipPinch_State()
{
}

void RobotManager::flipPinch_Entry()
{
  JointPosition flip_position(joint_number_);
  flip_position = q_meas_;
  if (flip_position[4] < 0)
  {
    flip_position[4] = flip_position[4] + M_PI;
  }
  else
  {
    flip_position[4] = flip_position[4] - M_PI;
  }
  pose_manager_.setJoints("Flip", flip_position);
  gotoPosition_(flip_position);
}

void RobotManager::flipPinch_Exit()
{
}
/*******************************************/

void RobotManager::gotoPosition_(const JointPosition q_pose)
{
  trajectory_msgs::JointTrajectory new_jt_traj;

  new_jt_traj.header.stamp = ros::Time::now();
  new_jt_traj.joint_names.resize(joint_number_);
  trajectory_msgs::JointTrajectoryPoint point;
  point.positions = q_pose;
  /* Set velocity ensure a cubic interpolation */
  point.velocities.resize(joint_number_);
  for (int i = 0; i < joint_number_; i++)
  {
    new_jt_traj.joint_names[i] = "joint_" + std::to_string(i + 1);
    point.velocities[i] = 0.0;
  }

  point.time_from_start = ros::Duration(computeDuration_(q_pose));

  new_jt_traj.points.push_back(point);
  command_pub_.publish(new_jt_traj);
}

bool RobotManager::isPositionCompleted_(const JointPosition q_pose)
{
  bool is_completed = true;
  for (int i = 0; i < joint_number_; i++)
  {
    if (std::abs(q_meas_[i] - q_pose[i]) > pose_goal_joints_tolerance_)
    {
      ROS_ERROR("The current target position %5f of axis %d is not equal to target goal %5f", q_meas_[i], i, q_pose[i]);
      is_completed = false;
    }
  }
  return is_completed;
}

double RobotManager::computeDuration_(const JointPosition q_pose)
{
  double delta_tmp = 0.0, delta_max = 0.0, duration = 0.0;
  for (int i = 0; i < joint_number_; i++)
  {
    double delta_tmp = std::abs(q_pose[i] - q_meas_[i]);
    if (delta_tmp > delta_max)
    {
      delta_max = delta_tmp;
    }
  }
  duration = delta_max / cartesian_max_vel_;
  return duration;
}

void RobotManager::sendJointsCommand_()
{
  trajectory_msgs::JointTrajectory new_jt_traj;
  new_jt_traj.header.stamp = ros::Time::now();
  new_jt_traj.joint_names.resize(joint_number_);
  trajectory_msgs::JointTrajectoryPoint point;
  point.time_from_start = ros::Duration(1.0 / sampling_freq_);
  for (int i = 0; i < joint_number_; i++)
  {
    new_jt_traj.joint_names[i] = "joint_" + std::to_string(i + 1);
  }
  point.positions = q_command_;

  // Important : do not send velocity else cubic interpolation is done !
  new_jt_traj.points.push_back(point);
  command_pub_.publish(new_jt_traj);
};

bool RobotManager::cartesianIsEnable_()
{
  return (fsm_state_ == RobotManagerFsmState::CartesianMode);
}
}

using namespace cartesian_controller;
int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_manager");
  int joint_number = 6;
  bool use_quaternion = false;
  ros::param::get("~joint_number", joint_number);
  ros::param::get("~use_quaternion", use_quaternion);

  RobotManager robot_manager(joint_number, use_quaternion);

  return 0;
}
