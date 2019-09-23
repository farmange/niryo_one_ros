// TODO copyright
#include "ros/ros.h"

#include "orthopus_interface/robot_manager.h"
#include <niryo_one_msgs/CloseGripper.h>

#define RATE 10
#define TOOL_ID_GRIPPER_2 12

namespace cartesian_controller
{
RobotManager::RobotManager()
{
  ROS_DEBUG_STREAM("RobotManager constructor");
  initializeSubscribers();
  initializePublishers();
  initializeServices();
  initializeActions();
  sampling_freq_ = RATE;  // TODO put this param in the config file
  ros::Rate loop_rate = ros::Rate(sampling_freq_);

  // Wait for initial messages
  ROS_INFO("Waiting for first joint msg.");
  ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states");
  ROS_INFO("Received first joint msg.");
  cartesian_controller_.init(sampling_freq_, pose_manager_, command_pub_, debug_pose_current_, debug_pose_desired_,
                             debug_pose_meas_, debug_joint_desired_, debug_joint_min_limit_, debug_joint_max_limit_);

  while (ros::ok())
  {
    ros::spinOnce();
    cartesian_controller_.run();

    // TODO : why not let joystick_enable_pub_ in cartesian_controller_ ?
    std_msgs::Bool joystick_enable_msg;
    joystick_enable_msg.data = cartesian_controller_.cartesianIsEnable();
    joystick_enabled_pub_.publish(joystick_enable_msg);

    loop_rate.sleep();
  }
}

void RobotManager::init(sensor_msgs::JointState& current_joint_state)
{
}

void RobotManager::initializeSubscribers()
{
  ROS_DEBUG_STREAM("RobotManager initializeSubscribers");
  joints_sub_ = n_.subscribe("joint_states", 1, &CartesianController::callbackJointState, &cartesian_controller_);
  move_group_state_ = n_.subscribe("/orthopus_interface/move_groupe_node/state", 1,
                                   &CartesianController::callbackMoveGroupState, &cartesian_controller_);
  dx_des_sub_ = n_.subscribe("dx_des", 1, &CartesianController::callbackVelocitiesDesired, &cartesian_controller_);
  learning_mode_sub_ =
      n_.subscribe("/niryo_one/learning_mode", 1, &CartesianController::callbackLearningMode, &cartesian_controller_);
}
void RobotManager::initializePublishers()
{
  ROS_DEBUG_STREAM("RobotManager initializePublishers");

  command_pub_ =
      n_.advertise<trajectory_msgs::JointTrajectory>("/niryo_one_follow_joint_trajectory_controller/command", 1);
  joystick_enabled_pub_ = n_.advertise<std_msgs::Bool>("/niryo_one/joystick_interface/is_enabled", 1);

  debug_pose_current_ = n_.advertise<geometry_msgs::Pose>("/debug_pose_current", 1);
  debug_pose_desired_ = n_.advertise<geometry_msgs::Pose>("/debug_pose_desired", 1);
  debug_pose_meas_ = n_.advertise<geometry_msgs::Pose>("/debug_pose_meas", 1);

  debug_joint_desired_ = n_.advertise<sensor_msgs::JointState>("/debug_joint_desired", 1);
  debug_joint_min_limit_ = n_.advertise<sensor_msgs::JointState>("/debug_joint_min_limit", 1);
  debug_joint_max_limit_ = n_.advertise<sensor_msgs::JointState>("/debug_joint_max_limit", 1);
}
void RobotManager::initializeServices()
{
  ROS_DEBUG_STREAM("RobotManager initializeServices");

  action_service_ = n_.advertiseService("/niryo_one/orthopus_interface/action", &CartesianController::callbackAction,
                                        &cartesian_controller_);

  cartesian_enable_service_ = n_.advertiseService(
      "/niryo_one/joystick_interface/enable", &CartesianController::callbackCartesianEnable, &cartesian_controller_);

  manage_pose_service_ = n_.advertiseService("/niryo_one/orthopus_interface/manage_pose",
                                             &PoseManager::callbackManagePose, &pose_manager_);
}
void RobotManager::initializeActions()
{
  //   ROS_DEBUG_STREAM("RobotManager initializeActions");
  //
  //   // Connecting to the robot ===========================================
  //   ROS_INFO("Connecting to robot  ========================");
  //   ac_ = new NiryoClient("/niryo_one/commander/robot_action/", true);
  //   ROS_INFO("Waiting for action server to start.");
  //   ac_->waitForServer();
  //   ROS_INFO("Action server started, sending goal.");
}
}

using namespace cartesian_controller;
int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_manager");

  RobotManager robot_manager;

  return 0;
}
