// TODO copyright
#include <ros/ros.h>

#include <tf2/LinearMath/Quaternion.h>

#include <niryo_one_msgs/GetInt.h>
#include <niryo_one_msgs/RobotMove.h>

#include "orthopus_interface/cartesian_controller.h"

#define INIT_TIME 2
#define MAX_VELOCITY 0.9

namespace cartesian_controller
{
CartesianController::CartesianController()
{
  ROS_DEBUG_STREAM("CartesianController constructor");
  move_group_state_ = 0;
  planning_pending_ = false;
  enable_joy_ = true;
  fsm_state = FsmState::Disable;
  // Initialize attributes // TODO check all attributes
  for (int i = 0; i < 6; i++)
  {
    joint_position_cmd[i] = 0.0;
    cartesian_velocity_desired[i] = 0.0;
  }
}

void CartesianController::init(int sampling_freq,
                               PoseManager& pose_manager,
                               ros::Publisher& command_pub, 
                               ros::Publisher& debug_pose_current,
                               ros::Publisher& debug_pose_desired,
                               ros::Publisher& debug_pose_meas,
                               ros::Publisher& debug_joint_desired,
                               ros::Publisher& debug_joint_min_limit,
                               ros::Publisher& debug_joint_max_limit)
{
  ROS_DEBUG_STREAM("CartesianController init");
  pose_manager_ = pose_manager;
  command_pub_ = command_pub;
  debug_pose_current_ = debug_pose_current;
  debug_pose_desired_ = debug_pose_desired;  
  debug_pose_meas_ = debug_pose_meas;  
  debug_joint_desired_ = debug_joint_desired;
  debug_joint_min_limit_ = debug_joint_min_limit;
  debug_joint_max_limit_ = debug_joint_max_limit;
  sampling_freq_ = sampling_freq;
  
  /* This is use to update joint state before running anything */
  ros::spinOnce();
  ik_.Init(sampling_freq_,
           debug_pose_current_,
           debug_pose_desired_,
           debug_pose_meas_,
           debug_joint_desired_,
           debug_joint_min_limit_,
           debug_joint_max_limit_);
  ik_.Reset(current_joint_state);
  
  position_compensator_.init(sampling_freq_,
                             debug_pose_current_,
                             debug_pose_desired_,
                             debug_pose_meas_);
}

void CartesianController::run()
{
  updateFsm();
  runFsm();
}

bool CartesianController::cartesianIsEnable()
{
  return (fsm_state == FsmState::CartesianMode);
}

bool CartesianController::callbackAction(niryo_one_msgs::SetInt::Request& req, niryo_one_msgs::SetInt::Response& res)
{
  if (req.value == FsmAction::Cartesian)
  {
    action_requested = FsmAction::Cartesian;
  }
  else if (req.value == FsmAction::GotoHome)
  {
    action_requested = FsmAction::GotoHome;
  }
  else if (req.value == FsmAction::GotoRest)
  {
    action_requested = FsmAction::GotoRest;
  }
  else if (req.value == FsmAction::GotoDrink)
  {
    action_requested = FsmAction::GotoDrink;
  }
  else if (req.value == FsmAction::GotoStandGlass)
  {
    action_requested = FsmAction::GotoStandGlass;
  }
  else if (req.value == FsmAction::FlipPinch)
  {
    action_requested = FsmAction::FlipPinch;
  }
  else
  {
    /* Keep previous requested action */
    ROS_ERROR("Action requested unknown : %d", req.value);
  }
  return true;
}

void CartesianController::printFsm()
{
  ROS_DEBUG_STREAM("FSM state : " << fsm_state.ToString() << " - Action requested : " << action_requested.ToString());
}

void CartesianController::updateFsm()
{
  FsmState fsm_local_prev_state = fsm_state;

  /* This condition is valid for all state */
  if (learning_mode == true)
  {
    fsm_state = FsmState::Disable;
  }
  else
  {
    if (fsm_state == FsmState::Disable)
    {
      if (learning_mode == false)
      {
        fsm_state = FsmState::Idle;
      }
    }
    else if (fsm_state == FsmState::Idle)
    {
      if (action_requested == FsmAction::GotoHome)
      {
        fsm_state = FsmState::GotoHome;
        gotoHomeState();
      }
      else if (action_requested == FsmAction::GotoRest)
      {
        fsm_state = FsmState::GotoRest;
        gotoRestState();
      }
    }
    else if (fsm_state == FsmState::CartesianMode)
    {
      if (action_requested == FsmAction::GotoDrink)
      {
        ik_.Reset(current_joint_state);
        for (int i = 0; i < 6; i++)
        {
          joint_position_cmd[i] = current_joint_state.position[i];
        }
        position_compensator_.reset();
        geometry_msgs::Pose drink_pose;
        drink_pose.position.x = 0.0237014013867;
        drink_pose.position.y = -0.226999999994;
        drink_pose.position.z = 0.423000149945;
        drink_pose.orientation.x = 0;
        drink_pose.orientation.y = 0;
        drink_pose.orientation.z = 0;
        position_compensator_.setTrajectoryPose(drink_pose);
        
        fsm_state = FsmState::GotoDrink;
      }
      else if (action_requested == FsmAction::GotoStandGlass)
      {
        ik_.Reset(current_joint_state);
        for (int i = 0; i < 6; i++)
        {
          joint_position_cmd[i] = current_joint_state.position[i];
        }
        position_compensator_.reset();
        geometry_msgs::Pose stand_pose;
        stand_pose.position.x = -0.0183778;
        stand_pose.position.y = -0.233611;
        stand_pose.position.z = 0.419733;
        stand_pose.orientation.x = 0;
        stand_pose.orientation.y = 0;
        stand_pose.orientation.z = -3.14;
        position_compensator_.setTrajectoryPose(stand_pose);
        
        fsm_state = FsmState::GotoStandGlass;
      }
      else if (action_requested == FsmAction::GotoHome)
      {
        fsm_state = FsmState::GotoHome;
        gotoHomeState();
      }
      else if (action_requested == FsmAction::GotoRest)
      {
        fsm_state = FsmState::GotoRest;
        gotoRestState();
      }
        
    }
    else if (fsm_state == FsmState::GotoDrink)
    {
      if (action_requested == FsmAction::GotoHome)
      {
        fsm_state = FsmState::GotoHome;
        gotoHomeState();
      }
      else if (action_requested == FsmAction::GotoRest)
      {
        fsm_state = FsmState::GotoRest;
        gotoRestState();
      }
      else if(position_compensator_.isTrajectoryCompleted())
      {
        fsm_state = FsmState::FlipPinch;
        flipPinchState();
      }
    }
    else if (fsm_state == FsmState::GotoStandGlass)
    {
      if (action_requested == FsmAction::GotoHome)
      {
        fsm_state = FsmState::GotoHome;
        gotoHomeState();
      }
      else if (action_requested == FsmAction::GotoRest)
      {
        fsm_state = FsmState::GotoRest;
        gotoRestState();
      }
      else if(position_compensator_.isTrajectoryCompleted())
      {
        fsm_state = FsmState::FlipPinch;
        flipPinchState();
      }
    }
    else if (fsm_state == FsmState::GotoHome)
    {
      if(isPositionCompleted(pose_manager_.getJoints("Home")))
      {
        /* Switch to cartesian mode when position is completed */
        ik_.Reset(current_joint_state);
        for (int i = 0; i < 6; i++)
        {
          joint_position_cmd[i] = current_joint_state.position[i];
        }
        position_compensator_.reset();
        fsm_state = FsmState::CartesianMode;
      }
      else if (action_requested == FsmAction::GotoHome)
      {
        fsm_state = FsmState::GotoHome;
        gotoHomeState();
      }
      else if (action_requested == FsmAction::GotoRest)
      {
        fsm_state = FsmState::GotoRest;
        gotoRestState();
      }
    }
    else if (fsm_state == FsmState::GotoRest)
    {
      if (action_requested == FsmAction::GotoHome)
      {
        fsm_state = FsmState::GotoHome;
        gotoHomeState();
      }
      else if(isPositionCompleted(pose_manager_.getJoints("Rest")))
      {
        /* Switch to idle when position is completed */
        fsm_state = FsmState::Idle;
      }
    }
    else if (fsm_state == FsmState::FlipPinch)
    {
      if(isPositionCompleted(pose_manager_.getJoints("Flip")))
      {
        /* Switch to cartesian mode when position is completed */
        ik_.Reset(current_joint_state);
        for (int i = 0; i < 6; i++)
        {
          joint_position_cmd[i] = current_joint_state.position[i];
        }
        position_compensator_.reset();
        fsm_state = FsmState::CartesianMode;
      }
    }
    else
    {
      ROS_ERROR_STREAM("Unkown fsm state : " << fsm_state.ToString());
      exit(0);
    }
  }

  if (fsm_local_prev_state != fsm_state)
  {
    fsm_prev_state = fsm_state;
  }

  action_requested = FsmAction::None;
}

void CartesianController::runFsm()
{
  printFsm();
  ROS_DEBUG_STREAM("runFsm planning_pending_ :" << planning_pending_);

  if (fsm_state == FsmState::CartesianMode)
  {
    cartesianState();
  }
  else if (fsm_state == FsmState::GotoDrink)
  {
    gotoDrinkState();
  }
  else if (fsm_state == FsmState::GotoStandGlass)
  {
    gotoStandGlassState();
  }
  else if (fsm_state == FsmState::FlipPinch || fsm_state == FsmState::GotoRest || fsm_state == FsmState::GotoHome)
  {
    /* Do nothing */
  }
  else if (fsm_state == FsmState::Disable)
  {
    /* Do nothing */
  }
  else if (fsm_state == FsmState::Idle)
  {
    /* Do nothing */
  }
  else
  {
    ROS_ERROR_STREAM("Unkown fsm state : " << fsm_state.ToString());
    exit(0);
  }
}

void CartesianController::cartesianState()
{
//     ROS_INFO("=== Perform position compensation...");   
//     position_compensator_.setJointState(current_joint_state);
//     position_compensator_.setVelocitiesCommand(cartesian_velocity_desired);
//     position_compensator_.run(cartesian_velocity_compensated);
//     ROS_INFO("    Done.");
    
    ROS_INFO("=== Start IK computation...");   
    ik_.ResolveInverseKinematic(joint_position_cmd, current_joint_state, cartesian_velocity_desired);
    ROS_INFO("    Done.");
    
    ROS_INFO("=== Send Niryo One commands...");
    sendJointsCommand();
    ROS_INFO("    Done.");
}

void CartesianController::gotoHomeState()
{
  gotoPosition(pose_manager_.getJoints("Home"));
}

void CartesianController::gotoRestState()
{
  gotoPosition(pose_manager_.getJoints("Rest"));
}

void CartesianController::gotoDrinkState()
{   
  ROS_INFO("=== Perform position compensation...");   
  position_compensator_.setJointState(current_joint_state);
  position_compensator_.setVelocitiesCommand(cartesian_velocity_desired);
  position_compensator_.run(cartesian_velocity_compensated);  
  ROS_INFO("    Done.");
  
  ROS_INFO("=== Start IK computation...");   
  ik_.RequestUpdateAxisConstraints(0, 0.1);
  ik_.RequestUpdateAxisConstraints(1, 0.1);
  ik_.RequestUpdateAxisConstraints(2, 0.1);
  ik_.ResolveInverseKinematic(joint_position_cmd, current_joint_state, cartesian_velocity_compensated);
  ROS_INFO("    Done.");
  
  ROS_INFO("=== Send Niryo One commands...");
  sendJointsCommand();
  ROS_INFO("    Done.");
}

void CartesianController::gotoStandGlassState()
{   
  ROS_INFO("=== Perform position compensation...");   
  position_compensator_.setJointState(current_joint_state);
  position_compensator_.setVelocitiesCommand(cartesian_velocity_desired);
  position_compensator_.run(cartesian_velocity_compensated);  
  ROS_INFO("    Done.");
  
  ROS_INFO("=== Start IK computation...");   
  ik_.RequestUpdateAxisConstraints(0, 0.1);
  ik_.RequestUpdateAxisConstraints(1, 0.1);
  ik_.RequestUpdateAxisConstraints(2, 0.1);
  ik_.ResolveInverseKinematic(joint_position_cmd, current_joint_state, cartesian_velocity_compensated);
  ROS_INFO("    Done.");
  
  ROS_INFO("=== Send Niryo One commands...");
  sendJointsCommand();
  ROS_INFO("    Done.");
}

void CartesianController::flipPinchState()
{  
  std::vector<double> flip_position;
  flip_position = current_joint_state.position;
  if(flip_position[4] < 0)
  {
    flip_position[4] = flip_position[4] + M_PI;
  }
  else
  {
    flip_position[4] = flip_position[4] - M_PI;
  }
  pose_manager_.setJoints("Flip", flip_position);
  gotoPosition(flip_position);  
}

void CartesianController::gotoPosition(const std::vector<double> position)
{
  trajectory_msgs::JointTrajectory new_jt_traj;

  new_jt_traj.header.stamp = ros::Time::now();
  new_jt_traj.joint_names = current_joint_state.name;
  trajectory_msgs::JointTrajectoryPoint point;

  point.positions = position;

  point.velocities.resize(6);
  point.velocities[0] = 0.0;
  point.velocities[1] = 0.0;
  point.velocities[2] = 0.0;
  point.velocities[3] = 0.0;
  point.velocities[4] = 0.0;
  point.velocities[5] = 0.0;

  point.time_from_start = ros::Duration(computeDuration(position));

  new_jt_traj.points.push_back(point);
  command_pub_.publish(new_jt_traj);
}

bool CartesianController::isPositionCompleted(const std::vector<double> position)
{
  bool is_completed = true;
  for(int i = 0; i<6; i++)
  {
    if (std::abs(current_joint_state.position[i] - position[i]) > 0.025)
    {
      ROS_ERROR("The current target position %5f of axis %d is not equal to target goal %5f",current_joint_state.position[i], i, position[i]);
      is_completed = false;
    }
  }
  return is_completed;
}

double CartesianController::computeDuration(const std::vector<double> position)
{
  double delta_tmp = 0.0, delta_max = 0.0, duration = 0.0;
  for (int i = 0; i < 6; i++)
  {
    double delta_tmp = std::abs(position[i] - current_joint_state.position[i]);
    if (delta_tmp > delta_max)
    {
      delta_max = delta_tmp;
    }
  }
  duration = delta_max / MAX_VELOCITY;
  return duration;
}

void CartesianController::sendJointsCommand()
{

    trajectory_msgs::JointTrajectory new_jt_traj;
    new_jt_traj.header.stamp = ros::Time::now();
    new_jt_traj.joint_names = current_joint_state.name;

    trajectory_msgs::JointTrajectoryPoint point;
    point.time_from_start = ros::Duration(1.0 / sampling_freq_);
    ROS_ERROR("sampling_freq_ = %5f", sampling_freq_);
    for (int i = 0; i < 6; i++)
    {
      point.positions.push_back(joint_position_cmd[i]);
    }

    // Important : do not send velocity as cubic interpolation is done !
    // It should be better to handle that by proper velocity command (force velocity ramp on xbox controller)
    // point.velocities = current_joint_state.velocity;
    new_jt_traj.points.push_back(point);
    command_pub_.publish(new_jt_traj);
};

// Scale the incoming desired velocity
Vector6d CartesianController::scaleCartesianCommand(const geometry_msgs::TwistStamped& command) const
{
  Vector6d result;

  result(0) = command.twist.linear.x;
  result(1) = command.twist.linear.y;
  result(2) = command.twist.linear.z;
  result(3) = command.twist.angular.x;
  result(4) = command.twist.angular.y;
  result(5) = command.twist.angular.z;

  return result;
}

// Convert incoming joy commands to TwistStamped commands for jogging.
// The TwistStamped component goes to jogging, while buttons 0 & 1 control
// joints directly.
void CartesianController::callbackJointState(const sensor_msgs::JointStateConstPtr& msg)
{
  ROS_DEBUG_STREAM("callbackJointState");
  current_joint_state = *msg;
//   ROS_DEBUG_STREAM(current_joint_state);
}

void CartesianController::callbackMoveGroupState(const std_msgs::Int32Ptr& msg)
{
  //   ROS_DEBUG_STREAM("callbackMoveGroupState : " << (msg->data==1)?"true":"false");
  //   move_group_state_ = msg->data;
  //   planning_pending_ = (msg->data==1)?true:false;
  //   ROS_DEBUG_STREAM("callbackMoveGroupState planning_pending_ :" << planning_pending_);
}

void CartesianController::callbackVelocitiesDesired(const geometry_msgs::TwistStampedPtr& msg)
{
  ROS_DEBUG_STREAM("callbackVelocitiesDesired");
  for (int i = 0; i < 6; i++)
  {
    cartesian_velocity_desired[i] = 0.0;
  }

    cartesian_velocity_desired[0] = msg->twist.linear.x;
    cartesian_velocity_desired[1] = msg->twist.linear.y;
    cartesian_velocity_desired[2] = msg->twist.linear.z;

    cartesian_velocity_desired[3] = msg->twist.angular.x;
    cartesian_velocity_desired[4] = msg->twist.angular.y;
    cartesian_velocity_desired[5] = msg->twist.angular.z;
    
    for(int i =0; i<3;i++)
    {
      if(cartesian_velocity_desired[i] != 0)
      {
        ik_.RequestUpdateAxisConstraints(i, 1.0);
      }
      else if(cartesian_velocity_desired[i] == 0 && cartesian_velocity_desired_prev[i] != 0)
      {
        ik_.RequestUpdateAxisConstraints(i, 0.001);
      }
      cartesian_velocity_desired_prev[i] = cartesian_velocity_desired[i];
    }
    for(int i =3; i<6;i++)
    {
      if(cartesian_velocity_desired[i] != 0)
      {
        ik_.RequestUpdateAxisConstraints(i, 1.0);
      }
      else if(cartesian_velocity_desired[i] == 0 && cartesian_velocity_desired_prev[i] != 0)
      {
        ik_.RequestUpdateAxisConstraints(i, 0.001);
      }
      cartesian_velocity_desired_prev[i] = cartesian_velocity_desired[i];
    }
  
}

void CartesianController::callbackLearningMode(const std_msgs::BoolPtr& msg)
{
  ROS_DEBUG_STREAM("callbackLearningMode");

  learning_mode = msg->data;
}

bool CartesianController::callbackCartesianEnable(niryo_one_msgs::SetInt::Request& req,
                                                  niryo_one_msgs::SetInt::Response& res)
{
  ROS_DEBUG_STREAM("callbackCartesianEnable");

  std::string result_message = "";
  int result = 0;
  if (req.value == 1)
  {
    if (can_enable())
    {
      enable_joy();
      result = 200;
      result_message = "Cartesian control has been enabled";
    }
    else
    {
      result = 400;
      result_message = "Wait for the end of command to enable the cartesian control";
    }
  }
  else
  {
    disable_joy();
    result = 200;
    result_message = "Cartesian control has been disabled";
  }
  res.status = result;
  res.message = result_message;
  return true;
}

bool CartesianController::can_enable()
{
  ros::service::waitForService("/niryo_one/commander/is_active");
  niryo_one_msgs::GetInt is_active;
  ros::ServiceClient commander_active_client =
      n_.serviceClient<niryo_one_msgs::GetInt>("/niryo_one/commander/is_active");
  commander_active_client.call(is_active);
  return (is_active.response.value == 0);
}

void CartesianController::enable_joy()
{
  ik_.Reset(current_joint_state);
  for (int i = 0; i < 6; i++)
  {
    joint_position_cmd[i] = current_joint_state.position[i];
  }
  enable_joy_ = true;
  ROS_INFO("Activate cartesian control : %d", 1);
}

void CartesianController::disable_joy()
{
  // TODO handle current motion : wait the arm reach a stable state
  enable_joy_ = false;
  ROS_INFO("Activate cartesian control : %d", 0);
}
}
