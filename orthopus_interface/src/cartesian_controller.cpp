// TODO copyright
#include <ros/ros.h>

#include <tf2/LinearMath/Quaternion.h>

#include <niryo_one_msgs/GetInt.h>
#include <niryo_one_msgs/RobotMove.h>

#include "orthopus_interface/cartesian_controller.h"

#define RATE 10
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

void CartesianController::init(PoseManager &pose_manager_, NiryoClient* ac_, ros::Publisher &command_pub_, ros::Publisher &debug_pub_, ros::Publisher &debug_des_pub_)
{
  ROS_DEBUG_STREAM("CartesianController init");
  this->pose_manager_ = pose_manager_;
  this->command_pub_ = command_pub_;
  this->debug_pub_ = debug_pub_;
  this->debug_des_pub_ = debug_des_pub_;
  this->ac_ = ac_;
  /* This is use to update joint state before running anything */
  ros::spinOnce();
  ik_.Init(debug_pub_, debug_des_pub_);
  ik_.Reset(current_joint_state);
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
  else if (req.value == FsmAction::GotoBackDrink)
  {
    action_requested = FsmAction::GotoBackDrink;
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
    else if (fsm_state == FsmState::Idle || fsm_state == FsmState::CartesianMode || fsm_state == FsmState::GotoRest)
    {
      if (action_requested == FsmAction::GotoHome)
      {
        fsm_state = FsmState::GotoHome;
        gotoHomeState();
      }
      else if (action_requested == FsmAction::GotoRest)
      {
        fsm_state = FsmState::GotoRest;
      }
      else if (action_requested == FsmAction::GotoDrink)
      {
        fsm_state = FsmState::GotoDrink;
        gotoDrinkState();
      }
      else if (action_requested == FsmAction::GotoBackDrink)
      {
        fsm_state = FsmState::GotoBackDrink;
        gotoBackDrinkState();
      }
      else if (action_requested == FsmAction::Cartesian &&
        (fsm_prev_state == FsmState::GotoHome || fsm_prev_state == FsmState::GotoBackDrink))  
        // TODO temporary hack to prevent go to cartesian if not in home
      {
        ik_.Reset(current_joint_state);
        for (int i = 0; i < 7; i++)
        {
          joint_position_cmd[i] = current_joint_state.position[i];
        }
        ros::service::waitForService("/orthopus_interface/move_groupe_node/move"); 
        niryo_one_msgs::RobotMove robot_move_msg;
        robot_move_msg.request.cmd.cmd_type = 666; //stop
        ros::ServiceClient move_group_client = n_.serviceClient<niryo_one_msgs::RobotMove>("/orthopus_interface/move_groupe_node/move");
        move_group_client.call(robot_move_msg);
        ROS_WARN_STREAM("robot_move_msg.response.status :" << robot_move_msg.response.status);
//         if(robot_move_msg.response.status == 8000)
//         {
//           planning_pending_ = true;
//         }
        fsm_state = FsmState::CartesianMode;
      }
    }
    else if (fsm_state == FsmState::GotoHome || fsm_state == FsmState::GotoDrink || fsm_state == FsmState::GotoBackDrink)
    {
      /* Do nothing */
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
  else if (fsm_state == FsmState::GotoBackDrink || fsm_state == FsmState::GotoDrink || fsm_state == FsmState::GotoRest || fsm_state == FsmState::GotoHome)
  {
    if(planning_pending_ == true)
    {
      ros::service::waitForService("/orthopus_interface/move_groupe_node/get_state");
      niryo_one_msgs::GetInt state_msg;
      ros::ServiceClient get_state_client = n_.serviceClient<niryo_one_msgs::GetInt>("/orthopus_interface/move_groupe_node/get_state");
      get_state_client.call(state_msg);
      planning_pending_ = (state_msg.response.value == 0)?false:true;
    }
    else
    {
      fsm_state = FsmState::Idle;
    }
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
  if (enable_joy_)
  {
    ROS_INFO("=== Start IK computation...");
    ik_.ResolveInverseKinematic(joint_position_cmd, current_joint_state, cartesian_velocity_desired);
    ROS_INFO("    Done.");

    ROS_INFO("=== Send Niryo One commands...");
    sendJointsCommand();
    ROS_INFO("    Done.");
  }
}

void CartesianController::gotoHomeState()
{
  ros::service::waitForService("/orthopus_interface/move_groupe_node/move"); 
  geometry_msgs::Pose target_pose = pose_manager_.getPose("Home");
  niryo_one_msgs::RobotMove robot_move_msg;
  robot_move_msg.request.cmd.cmd_type = 0; //no constraint 
  
  robot_move_msg.request.cmd.pose_quat.position.x = target_pose.position.x;
  robot_move_msg.request.cmd.pose_quat.position.y = target_pose.position.y;
  robot_move_msg.request.cmd.pose_quat.position.z = target_pose.position.z;
  
  robot_move_msg.request.cmd.pose_quat.orientation.x = target_pose.orientation.x;
  robot_move_msg.request.cmd.pose_quat.orientation.y = target_pose.orientation.y;
  robot_move_msg.request.cmd.pose_quat.orientation.z = target_pose.orientation.z;
  robot_move_msg.request.cmd.pose_quat.orientation.w = target_pose.orientation.w;
  
  ros::ServiceClient move_group_client = n_.serviceClient<niryo_one_msgs::RobotMove>("/orthopus_interface/move_groupe_node/move");
  move_group_client.call(robot_move_msg);
  ROS_WARN_STREAM("robot_move_msg.response.status :" << robot_move_msg.response.status);
  if(robot_move_msg.response.status == 8000)
  {
    planning_pending_ = true;
  }
  
}

void CartesianController::gotoRestState()
{
  ros::service::waitForService("/orthopus_interface/move_groupe_node/move"); 
  geometry_msgs::Pose target_pose = pose_manager_.getPose("Rest");
  niryo_one_msgs::RobotMove robot_move_msg;
  robot_move_msg.request.cmd.cmd_type = 0; //no constraint 
  
  robot_move_msg.request.cmd.pose_quat.position.x = target_pose.position.x;
  robot_move_msg.request.cmd.pose_quat.position.y = target_pose.position.y;
  robot_move_msg.request.cmd.pose_quat.position.z = target_pose.position.z;
  
  robot_move_msg.request.cmd.pose_quat.orientation.x = target_pose.orientation.x;
  robot_move_msg.request.cmd.pose_quat.orientation.y = target_pose.orientation.y;
  robot_move_msg.request.cmd.pose_quat.orientation.z = target_pose.orientation.z;
  robot_move_msg.request.cmd.pose_quat.orientation.w = target_pose.orientation.w;
  
  ros::ServiceClient move_group_client = n_.serviceClient<niryo_one_msgs::RobotMove>("/orthopus_interface/move_groupe_node/move");
  move_group_client.call(robot_move_msg);
  ROS_WARN_STREAM("robot_move_msg.response.status :" << robot_move_msg.response.status);
  if(robot_move_msg.response.status == 8000)
  {
    planning_pending_ = true;
  }
}

void CartesianController::gotoDrinkState()
{  
  pose_manager_.setJoints("BackDrink", current_joint_state.position);
  
  ros::service::waitForService("/orthopus_interface/move_groupe_node/move");
  geometry_msgs::Pose target_pose = pose_manager_.getPose("Drink");
  niryo_one_msgs::RobotMove robot_move_msg;
  robot_move_msg.request.cmd.cmd_type = 1;
  
  robot_move_msg.request.cmd.pose_quat.position.x = target_pose.position.x;
  robot_move_msg.request.cmd.pose_quat.position.y = target_pose.position.y;
  robot_move_msg.request.cmd.pose_quat.position.z = target_pose.position.z;
  
  robot_move_msg.request.cmd.pose_quat.orientation.x = target_pose.orientation.x;
  robot_move_msg.request.cmd.pose_quat.orientation.y = target_pose.orientation.y;
  robot_move_msg.request.cmd.pose_quat.orientation.z = target_pose.orientation.z;
  robot_move_msg.request.cmd.pose_quat.orientation.w = target_pose.orientation.w;
  
  ros::ServiceClient move_group_client = n_.serviceClient<niryo_one_msgs::RobotMove>("/orthopus_interface/move_groupe_node/move");
  move_group_client.call(robot_move_msg);
  ROS_WARN_STREAM("robot_move_msg.response.status :" << robot_move_msg.response.status);
  if(robot_move_msg.response.status == 8000)
  {
    planning_pending_ = true;
  }
}

void CartesianController::gotoBackDrinkState()
{   
  ros::service::waitForService("/orthopus_interface/move_groupe_node/move");
  geometry_msgs::Pose target_pose = pose_manager_.getPose("BackDrink");
  niryo_one_msgs::RobotMove robot_move_msg;
  robot_move_msg.request.cmd.cmd_type = 1;
  
  robot_move_msg.request.cmd.pose_quat.position.x = target_pose.position.x;
  robot_move_msg.request.cmd.pose_quat.position.y = target_pose.position.y;
  robot_move_msg.request.cmd.pose_quat.position.z = target_pose.position.z;
  
  robot_move_msg.request.cmd.pose_quat.orientation.x = target_pose.orientation.x;
  robot_move_msg.request.cmd.pose_quat.orientation.y = target_pose.orientation.y;
  robot_move_msg.request.cmd.pose_quat.orientation.z = target_pose.orientation.z;
  robot_move_msg.request.cmd.pose_quat.orientation.w = target_pose.orientation.w;
  
  ros::ServiceClient move_group_client = n_.serviceClient<niryo_one_msgs::RobotMove>("/orthopus_interface/move_groupe_node/move");
  move_group_client.call(robot_move_msg);
  ROS_WARN_STREAM("robot_move_msg.response.status :" << robot_move_msg.response.status);
  if(robot_move_msg.response.status == 8000)
  {
    planning_pending_ = true;
  }
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
  if (enable_joy_)
  {
    trajectory_msgs::JointTrajectory new_jt_traj;
    new_jt_traj.header.stamp = ros::Time::now();
    new_jt_traj.joint_names = current_joint_state.name;

    trajectory_msgs::JointTrajectoryPoint point;
    point.time_from_start = ros::Duration(1.0 / RATE);
    for (int i = 0; i < 6; i++)
    {
      point.positions.push_back(joint_position_cmd[i]);
    }

    // Important : do not send velocity as cubic interpolation is done !
    // It should be better to handle that by proper velocity command (force velocity ramp on xbox controller)
    // point.velocities = current_joint_state.velocity;
    new_jt_traj.points.push_back(point);
    command_pub_.publish(new_jt_traj);
  }
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
  for (int i = 0; i < 7; i++)
  {
    cartesian_velocity_desired[i] = 0.0;
  }

  if (enable_joy_)
  {
    cartesian_velocity_desired[0] = msg->twist.linear.x;
    cartesian_velocity_desired[1] = msg->twist.linear.y;
    cartesian_velocity_desired[2] = msg->twist.linear.z;

    tf2::Quaternion converted_quaternion;
    converted_quaternion.setRPY(msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z);

    cartesian_velocity_desired[3] = converted_quaternion.getW();
    cartesian_velocity_desired[4] = converted_quaternion.getX();
    cartesian_velocity_desired[5] = converted_quaternion.getY();
    cartesian_velocity_desired[6] = converted_quaternion.getZ();
    if (cartesian_velocity_desired[4] == 0 && cartesian_velocity_desired[5] == 0 && cartesian_velocity_desired[6] == 0)
    {
      cartesian_velocity_desired[3] = 0;
    }

    for (int i = 0; i < 7; i++)
    {
      if (i < 3)
      {
        if (cartesian_velocity_desired[i] != 0)
        {
          ik_.UpdateAxisConstraints(i, 1.0);
        }
        else if (cartesian_velocity_desired[i] == 0 && cartesian_velocity_desired_prev[i] != 0)
        {
          ik_.UpdateAxisConstraints(i, 0.005);
        }
      }
      cartesian_velocity_desired_prev[i] = cartesian_velocity_desired[i];
    }
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
  for (int i = 0; i < 7; i++)
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
