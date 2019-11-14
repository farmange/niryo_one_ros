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

#include "niryo_one_msgs/SetInt.h"
#include "orthopus_space_control/robot_manager.h"

// Eigen
#include "eigen_conversions/eigen_msg.h"

// TF
#include "tf/tf.h"

namespace space_control
{
RobotManager::RobotManager(const int joint_number, const bool use_quaternion, const bool debug)
  : cartesian_controller_(joint_number, use_quaternion)
  , pose_manager_(joint_number, use_quaternion)
  , joint_number_(joint_number)
  , use_quaternion_(use_quaternion)
  , debug_(debug)
  , q_command_(joint_number)
  , q_current_(joint_number)
  , q_meas_(joint_number)
  , x_drink_pose_()
  , x_stand_pose_()
  , dx_desired_()
  , dx_desired_prev_()
{
  ROS_DEBUG_STREAM("RobotManager constructor");
  retrieveParameters_();
  initializeSubscribers_();
  initializePublishers_();
  initializeServices_();
  initializeStateMachine_();
  ros::Rate loop_rate = ros::Rate(sampling_freq_);

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
    ROS_DEBUG("Current state is '%s' and input event is '%s'", engine_->getCurrentState()->getName().c_str(),
              input_event_requested_.toString().c_str());

    engine_->process();
    input_event_requested_ = FsmInputEvent::None;
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
  cartesian_controller_.setDebugPublishers(q_current_debug_pub_, x_current_debug_pub_, dx_desired_debug_pub_);
}

void RobotManager::callbackInputDeviceVelocity_(const geometry_msgs::TwistStampedPtr& msg)
{
  //   ROS_DEBUG_STREAM("callbackInputDeviceVelocity");

  dx_desired_.setX(msg->twist.linear.x * space_position_max_vel_);
  dx_desired_.setY(msg->twist.linear.y * space_position_max_vel_);
  dx_desired_.setZ(msg->twist.linear.z * space_position_max_vel_);

  dx_desired_.setQw(0.0);
  dx_desired_.setQx(msg->twist.angular.x * space_orientation_max_vel_);
  dx_desired_.setQy(msg->twist.angular.y * space_orientation_max_vel_);
  dx_desired_.setQz(msg->twist.angular.z * space_orientation_max_vel_);
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
  if (req.value == int(FsmInputEvent::JointHome))
  {
    input_event_requested_ = FsmInputEvent::JointHome;
  }
  else if (req.value == int(FsmInputEvent::JointRest))
  {
    input_event_requested_ = FsmInputEvent::JointRest;
  }
  else if (req.value == int(FsmInputEvent::TrajectoryDrink))
  {
    input_event_requested_ = FsmInputEvent::TrajectoryDrink;
  }
  else if (req.value == int(FsmInputEvent::TrajectoryStand))
  {
    input_event_requested_ = FsmInputEvent::TrajectoryStand;
  }
  else
  {
    /* Keep previous requested action */
    ROS_ERROR("Input event not allowed : %d", req.value);
  }
  return true;
}

void RobotManager::initializeSubscribers_()
{
  ROS_DEBUG_STREAM("RobotManager initializeSubscribers");
  joints_sub_ = n_.subscribe("joint_states", 1, &RobotManager::callbackJointState_, this);
  dx_input_device_sub_ = n_.subscribe("/orthopus_space_control/input_device_velocity", 1,
                                      &RobotManager::callbackInputDeviceVelocity_, this);
  learning_mode_sub_ = n_.subscribe("/niryo_one/learning_mode", 1, &RobotManager::callbackLearningMode_, this);
}

void RobotManager::initializePublishers_()
{
  ROS_DEBUG_STREAM("RobotManager initializePublishers");
  command_pub_ =
      n_.advertise<trajectory_msgs::JointTrajectory>("/niryo_one_follow_joint_trajectory_controller/command", 1);
  joystick_enabled_pub_ = n_.advertise<std_msgs::Bool>("/niryo_one/joystick_interface/is_enabled", 1);
  q_current_debug_pub_ = n_.advertise<sensor_msgs::JointState>("/orthopus_space_control/q_current", 1);
  x_current_debug_pub_ = n_.advertise<geometry_msgs::Pose>("/orthopus_space_control/x_current", 1);
  dx_desired_debug_pub_ = n_.advertise<geometry_msgs::Pose>("/orthopus_space_control/dx_desired", 1);
}

void RobotManager::initializeServices_()
{
  ROS_DEBUG_STREAM("RobotManager initializeServices");
  // TODO : refactor :better name for this service (not action)
  action_service_ =
      n_.advertiseService("/niryo_one/orthopus_space_control/action", &RobotManager::callbackAction_, this);
  manage_pose_service_ = n_.advertiseService("/niryo_one/orthopus_space_control/manage_pose",
                                             &PoseManager::callbackManagePose, &pose_manager_);
}

void RobotManager::retrieveParameters_()
{
  geometry_msgs::Pose drink_pose;
  geometry_msgs::Pose stand_pose;

  ros::param::get("~drink_pose/position/x", drink_pose.position.x);
  ros::param::get("~drink_pose/position/y", drink_pose.position.y);
  ros::param::get("~drink_pose/position/z", drink_pose.position.z);
  ros::param::get("~drink_pose/orientation/w", drink_pose.orientation.w);
  ros::param::get("~drink_pose/orientation/x", drink_pose.orientation.x);
  ros::param::get("~drink_pose/orientation/y", drink_pose.orientation.y);
  ros::param::get("~drink_pose/orientation/z", drink_pose.orientation.z);

  ros::param::get("~stand_pose/position/x", stand_pose.position.x);
  ros::param::get("~stand_pose/position/y", stand_pose.position.y);
  ros::param::get("~stand_pose/position/z", stand_pose.position.z);
  ros::param::get("~stand_pose/orientation/w", stand_pose.orientation.w);
  ros::param::get("~stand_pose/orientation/x", stand_pose.orientation.x);
  ros::param::get("~stand_pose/orientation/y", stand_pose.orientation.y);
  ros::param::get("~stand_pose/orientation/z", stand_pose.orientation.z);

  x_drink_pose_.setX(drink_pose.position.x);
  x_drink_pose_.setY(drink_pose.position.y);
  x_drink_pose_.setZ(drink_pose.position.z);
  x_drink_pose_.setQw(drink_pose.orientation.w);
  x_drink_pose_.setQx(drink_pose.orientation.x);
  x_drink_pose_.setQy(drink_pose.orientation.y);
  x_drink_pose_.setQz(drink_pose.orientation.z);

  x_stand_pose_.setX(stand_pose.position.x);
  x_stand_pose_.setY(stand_pose.position.y);
  x_stand_pose_.setZ(stand_pose.position.z);
  x_stand_pose_.setQw(stand_pose.orientation.w);
  x_stand_pose_.setQx(stand_pose.orientation.x);
  x_stand_pose_.setQy(stand_pose.orientation.y);
  x_stand_pose_.setQz(stand_pose.orientation.z);

  // x_stand_pose_[SpacePosition::kX] = stand_pose.position.x;
  // x_stand_pose_[SpacePosition::kY] = stand_pose.position.y;
  // x_stand_pose_[SpacePosition::kZ] = stand_pose.position.z;

  // if (use_quaternion_)
  // {
  //   x_drink_pose_[SpacePosition::kQw] = drink_pose.orientation.w;
  //   x_drink_pose_[SpacePosition::kQx] = drink_pose.orientation.x;
  //   x_drink_pose_[SpacePosition::kQy] = drink_pose.orientation.y;
  //   x_drink_pose_[SpacePosition::kQz] = drink_pose.orientation.z;

  //   x_stand_pose_[SpacePosition::kQw] = stand_pose.orientation.w;
  //   x_stand_pose_[SpacePosition::kQx] = stand_pose.orientation.x;
  //   x_stand_pose_[SpacePosition::kQy] = stand_pose.orientation.y;
  //   x_stand_pose_[SpacePosition::kQz] = stand_pose.orientation.z;
  // }
  // else
  // {
  //   double roll, pitch, yaw;
  //   tf::Quaternion q;
  //   /* Convert quaternion pose in RPY */
  //   q.setValue(drink_pose.orientation.x, drink_pose.orientation.y, drink_pose.orientation.z,
  //   drink_pose.orientation.w);

  //   tf::Matrix3x3 m1(q);
  //   m1.getRPY(roll, pitch, yaw);

  //   x_drink_pose_[SpacePosition::kRoll] = roll;
  //   x_drink_pose_[SpacePosition::kPitch] = pitch;
  //   x_drink_pose_[SpacePosition::kYaw] = yaw;

  //   /* Convert quaternion pose in RPY */
  //   q.setValue(stand_pose.orientation.x, stand_pose.orientation.y, stand_pose.orientation.z,
  //   stand_pose.orientation.w);

  //   tf::Matrix3x3 m2(q);
  //   m2.getRPY(roll, pitch, yaw);

  //   x_stand_pose_[SpacePosition::kRoll] = roll;
  //   x_stand_pose_[SpacePosition::kPitch] = pitch;
  //   x_stand_pose_[SpacePosition::kYaw] = -yaw;  // HACK force negative side of rotation (conversion cannot handle
  //   that)
  // }

  ros::param::get("~pose_goal_joints_tolerance", pose_goal_joints_tolerance_);
  ros::param::get("~sampling_frequency", sampling_freq_);
  ros::param::get("~space_position_max_vel", space_position_max_vel_);
  ros::param::get("~space_orientation_max_vel", space_orientation_max_vel_);
  ros::param::get("~debug", debug_);
}
void RobotManager::initializeStateMachine_()
{
  /* State definition */
  state_disable_ = new State<RobotManager>(this, "DISABLE");

  state_idle_ = new State<RobotManager>(this, "IDLE");

  state_joint_home_ = new State<RobotManager>(this, "JOINT HOME");
  state_joint_home_->registerEnterFcn(&RobotManager::jointHomeEnter_);

  state_joint_rest_ = new State<RobotManager>(this, "JOINT REST");
  state_joint_rest_->registerEnterFcn(&RobotManager::jointRestEnter_);

  state_traj_drink_ = new State<RobotManager>(this, "TRAJ DRINK");
  state_traj_drink_->registerEnterFcn(&RobotManager::trajDrinkEnter_);
  state_traj_drink_->registerUpdateFcn(&RobotManager::trajDrinkUpdate_);

  state_traj_stand_ = new State<RobotManager>(this, "TRAJ STAND");
  state_traj_stand_->registerEnterFcn(&RobotManager::trajStandEnter_);
  state_traj_stand_->registerUpdateFcn(&RobotManager::trajStandUpdate_);

  state_flip_pinch_ = new State<RobotManager>(this, "FLIP PINCH");
  state_flip_pinch_->registerEnterFcn(&RobotManager::flipPinchEnter_);

  state_space_control_ = new State<RobotManager>(this, "SPACE CONTROL");
  state_space_control_->registerEnterFcn(&RobotManager::spaceControlEnter_);
  state_space_control_->registerUpdateFcn(&RobotManager::spaceControlUpdate_);

  /* Transitions definition */
  tr_all_to_disable_ = new Transition<RobotManager>(this, state_disable_);
  tr_all_to_disable_->registerConditionFcn(&RobotManager::trAllToDisable_);
  tr_all_to_disable_->addInitialState(state_idle_);
  tr_all_to_disable_->addInitialState(state_joint_home_);
  tr_all_to_disable_->addInitialState(state_joint_rest_);
  tr_all_to_disable_->addInitialState(state_traj_drink_);
  tr_all_to_disable_->addInitialState(state_traj_stand_);
  tr_all_to_disable_->addInitialState(state_flip_pinch_);
  tr_all_to_disable_->addInitialState(state_space_control_);

  tr_disable_to_idle_ = new Transition<RobotManager>(this, state_idle_);
  tr_disable_to_idle_->registerConditionFcn(&RobotManager::trDisableToIdle_);
  tr_disable_to_idle_->addInitialState(state_disable_);

  tr_rest_to_idle_ = new Transition<RobotManager>(this, state_idle_);
  tr_rest_to_idle_->registerConditionFcn(&RobotManager::trRestToIdle_);
  tr_rest_to_idle_->addInitialState(state_joint_rest_);

  tr_to_joint_home_ = new Transition<RobotManager>(this, state_joint_home_);
  tr_to_joint_home_->registerConditionFcn(&RobotManager::trToJointHome_);
  tr_to_joint_home_->addInitialState(state_idle_);
  tr_to_joint_home_->addInitialState(state_joint_rest_);
  tr_to_joint_home_->addInitialState(state_traj_drink_);
  tr_to_joint_home_->addInitialState(state_traj_stand_);
  tr_to_joint_home_->addInitialState(state_joint_home_);
  tr_to_joint_home_->addInitialState(state_flip_pinch_);
  tr_to_joint_home_->addInitialState(state_space_control_);

  tr_to_joint_rest_ = new Transition<RobotManager>(this, state_joint_rest_);
  tr_to_joint_rest_->registerConditionFcn(&RobotManager::trToJointRest_);
  tr_to_joint_rest_->addInitialState(state_idle_);
  tr_to_joint_rest_->addInitialState(state_joint_rest_);
  tr_to_joint_rest_->addInitialState(state_traj_drink_);
  tr_to_joint_rest_->addInitialState(state_traj_stand_);
  tr_to_joint_rest_->addInitialState(state_joint_home_);
  tr_to_joint_rest_->addInitialState(state_flip_pinch_);
  tr_to_joint_rest_->addInitialState(state_space_control_);

  tr_to_traj_drink_ = new Transition<RobotManager>(this, state_traj_drink_);
  tr_to_traj_drink_->registerConditionFcn(&RobotManager::trToTrajDrink_);
  tr_to_traj_drink_->addInitialState(state_space_control_);

  tr_to_traj_stand_ = new Transition<RobotManager>(this, state_traj_stand_);
  tr_to_traj_stand_->registerConditionFcn(&RobotManager::trToTrajStand_);
  tr_to_traj_stand_->addInitialState(state_space_control_);

  tr_to_flip_pinch_ = new Transition<RobotManager>(this, state_flip_pinch_);
  tr_to_flip_pinch_->registerConditionFcn(&RobotManager::trToFlipPinch_);
  tr_to_flip_pinch_->addInitialState(state_traj_drink_);
  tr_to_flip_pinch_->addInitialState(state_traj_stand_);

  tr_flip_pinch_to_space_control_ = new Transition<RobotManager>(this, state_space_control_);
  tr_flip_pinch_to_space_control_->registerConditionFcn(&RobotManager::trFlipPinchToSpaceControl_);
  tr_flip_pinch_to_space_control_->addInitialState(state_flip_pinch_);

  tr_joint_home_to_space_control_ = new Transition<RobotManager>(this, state_space_control_);
  tr_joint_home_to_space_control_->registerConditionFcn(&RobotManager::trJointHomeToSpaceControl_);
  tr_joint_home_to_space_control_->addInitialState(state_joint_home_);

  /* Engine definition */
  engine_ = new Engine<RobotManager>(this);
  engine_->registerState(state_idle_);
  engine_->registerState(state_disable_);
  engine_->registerState(state_joint_home_);
  engine_->registerState(state_joint_rest_);
  engine_->registerState(state_traj_drink_);
  engine_->registerState(state_traj_stand_);
  engine_->registerState(state_flip_pinch_);
  engine_->registerState(state_space_control_);
  engine_->registerState(state_disable_);

  engine_->registerTransition(tr_all_to_disable_);
  engine_->registerTransition(tr_disable_to_idle_);
  engine_->registerTransition(tr_rest_to_idle_);
  engine_->registerTransition(tr_to_joint_home_);
  engine_->registerTransition(tr_to_joint_rest_);
  engine_->registerTransition(tr_to_traj_drink_);
  engine_->registerTransition(tr_to_traj_stand_);
  engine_->registerTransition(tr_to_flip_pinch_);
  engine_->registerTransition(tr_flip_pinch_to_space_control_);
  engine_->registerTransition(tr_joint_home_to_space_control_);

  engine_->setCurrentState(state_disable_);
}

void RobotManager::spaceControlUpdate_()
{
  ROS_INFO("=== Update joint position (Open loop)...");
  q_current_ = q_command_;

  cartesian_controller_.setDxDesired(dx_desired_);
  cartesian_controller_.setInputSelector(CartesianController::INPUT_USER);
  cartesian_controller_.run(q_current_, q_command_);

  ROS_INFO("=== Send Niryo One joints command...");
  sendJointsCommand_();
}

void RobotManager::spaceControlEnter_()
{
  /* Switch to cartesian mode when position is completed */
  cartesian_controller_.reset();
  q_command_ = q_meas_;
}

void RobotManager::jointHomeEnter_()
{
  gotoPosition_(pose_manager_.getJoints("Home"));
}

void RobotManager::jointRestEnter_()
{
  gotoPosition_(pose_manager_.getJoints("Rest"));
}

void RobotManager::trajDrinkUpdate_()
{
  ROS_INFO("=== Update joint position (Open loop)...");
  // TODO check q_current consistency (many affectation could lead to issue
  q_current_ = q_command_;
  ROS_INFO("    Done.");

  cartesian_controller_.setDxDesired(dx_desired_);
  cartesian_controller_.setInputSelector(CartesianController::INPUT_TRAJECTORY);
  cartesian_controller_.run(q_current_, q_command_);

  ROS_INFO("=== Send Niryo One commands...");
  sendJointsCommand_();
  ROS_INFO("    Done.");
}

void RobotManager::trajDrinkEnter_()
{
  cartesian_controller_.reset();
  q_command_ = q_meas_;
  cartesian_controller_.getTrajectoryController()->setTrajectoryPose(x_drink_pose_);
}

void RobotManager::trajStandUpdate_()
{
  ROS_INFO("=== Update joint position (Open loop)...");
  q_current_ = q_command_;
  ROS_INFO("    Done.");

  cartesian_controller_.setDxDesired(dx_desired_);
  cartesian_controller_.setInputSelector(CartesianController::INPUT_TRAJECTORY);
  cartesian_controller_.run(q_current_, q_command_);

  ROS_INFO("=== Send Niryo One commands...");
  sendJointsCommand_();
  ROS_INFO("    Done.");
}

void RobotManager::trajStandEnter_()
{
  cartesian_controller_.reset();
  q_command_ = q_meas_;
  cartesian_controller_.getTrajectoryController()->setTrajectoryPose(x_stand_pose_);
}

void RobotManager::flipPinchEnter_()
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

bool RobotManager::trDisableToIdle_()
{
  return (learning_mode_ == 0);
}

bool RobotManager::trAllToDisable_()
{
  return (learning_mode_ == 1);
}

bool RobotManager::trToJointHome_()
{
  return (input_event_requested_ == FsmInputEvent::JointHome);
}

bool RobotManager::trToJointRest_()
{
  return (input_event_requested_ == FsmInputEvent::JointRest);
}

bool RobotManager::trToTrajDrink_()
{
  return (input_event_requested_ == FsmInputEvent::TrajectoryDrink);
}

bool RobotManager::trToTrajStand_()
{
  return (input_event_requested_ == FsmInputEvent::TrajectoryStand);
}

bool RobotManager::trToFlipPinch_()
{
  return (cartesian_controller_.getTrajectoryController()->isTrajectoryCompleted());
}

bool RobotManager::trJointHomeToSpaceControl_()
{
  return (isPositionCompleted_(pose_manager_.getJoints("Home")));
}

bool RobotManager::trFlipPinchToSpaceControl_()
{
  return (isPositionCompleted_(pose_manager_.getJoints("Flip")));
}

bool RobotManager::trRestToIdle_()
{
  return (isPositionCompleted_(pose_manager_.getJoints("Rest")));
}

void RobotManager::gotoPosition_(const JointPosition q_pose) const
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

bool RobotManager::isPositionCompleted_(const JointPosition q_pose) const
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

double RobotManager::computeDuration_(const JointPosition q_pose) const
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
  duration = delta_max / space_position_max_vel_;
  return duration;
}

void RobotManager::sendJointsCommand_() const
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
}

using namespace space_control;
int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_manager");
  int joint_number = 6;
  bool use_quaternion = true;
  bool debug = false;

  ros::param::get("~joint_number", joint_number);
  ros::param::get("~use_quaternion", use_quaternion);
  ros::param::get("~debug", debug);

  RobotManager robot_manager(joint_number, use_quaternion, debug);

  return 0;
}
