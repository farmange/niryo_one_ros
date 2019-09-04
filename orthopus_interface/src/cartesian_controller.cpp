// TODO copyright
#include <ros/ros.h>

#include <tf2/LinearMath/Quaternion.h>

#include <niryo_one_msgs/GetInt.h>

#include "orthopus_interface/cartesian_controller.h"

#define RATE 10
#define TOOL_ID_GRIPPER_1 12
#define INIT_TIME 2
#define MAX_VELOCITY 0.9

namespace cartesian_controller
{
CartesianController::CartesianController()
{
  ROS_INFO("CartesianController constructor");
  ros::Rate loop_rate = ros::Rate(RATE);

  joints_sub_ = n_.subscribe("joint_states", 1, &CartesianController::jointStatesCB, this);
  dx_des_sub_ = n_.subscribe("dx_des", 1, &CartesianController::dxDesCB, this);
  gripper_sub_ = n_.subscribe("gripper_des", 1, &CartesianController::gripperCB, this);
  learning_mode_sub_ = n_.subscribe("/niryo_one/learning_mode", 1, &CartesianController::learningModeCB, this);

  command_pub_ = n_.advertise<trajectory_msgs::JointTrajectory>("/niryo_one_follow_joint_trajectory_controller/command",
                                                                1);  // TODO check optimal queue size

  joystick_enabled_pub_ = n_.advertise<std_msgs::Bool>("/niryo_one/joystick_interface/is_enabled", 1);
  debug_pub_ = n_.advertise<geometry_msgs::Pose>("/debug_cartesian_pos", 1);
  debug_des_pub_ = n_.advertise<geometry_msgs::Pose>("/debug_cartesian_pos_des", 1);

  ros::ServiceServer control_service =
      n_.advertiseService("/niryo_one/orthopus_interface/action", &CartesianController::actionCB, this);

  ros::ServiceServer enable_service =
      n_.advertiseService("/niryo_one/joystick_interface/enable", &CartesianController::enableCB, this);

  tool_controller_.setToolId(TOOL_ID_GRIPPER_1);
  // Wait for initial messages
  ROS_INFO("Waiting for first joint msg.");
  ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states");
  ROS_INFO("Received first joint msg.");

  enable_joy_ = true;
  fsm_state = FsmState::Disable;
  // Initialize attributes
  for (int i = 0; i < 6; i++)
  {
    joint_position_cmd[i] = 0.0;
    cartesian_velocity_desired[i] = 0.0;
  }

  /* This is use to update joint state before running anything */
  ros::spinOnce();

  ik_.Init(debug_pub_, debug_des_pub_);
  ik_.Reset(current_joint_state);

  while (ros::ok())
  {
    ros::spinOnce();
    updateFsm();
    runFsm();

    std_msgs::Bool joystick_enable_msg;
    joystick_enable_msg.data = (fsm_state == FsmState::CartesianMode);
    joystick_enabled_pub_.publish(joystick_enable_msg);
    loop_rate.sleep();
  }
}

bool CartesianController::actionCB(niryo_one_msgs::SetInt::Request& req, niryo_one_msgs::SetInt::Response& res)
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
  else
  {
    /* Keep previous requested action */
    ROS_ERROR("Action requested unknown : %d", req.value);
  }
}
void CartesianController::printFsm()
{
  ROS_WARN_STREAM("FSM state : " << fsm_state.ToString() << " - Action requested : " << action_requested.ToString());
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
    else if (fsm_state == FsmState::Idle || fsm_state == FsmState::CartesianMode || fsm_state == FsmState::GotoHome ||
             fsm_state == FsmState::GotoRest)
    {
      if (action_requested == FsmAction::GotoHome)
      {
        fsm_state = FsmState::GotoHome;
      }
      else if (action_requested == FsmAction::GotoRest)
      {
        fsm_state = FsmState::GotoRest;
      }
      else if (action_requested == FsmAction::Cartesian &&
               fsm_prev_state == FsmState::GotoHome)  // TODO temporary hack to prevent go to cartesian if not in home
      {
        ik_.Reset(current_joint_state);
        for (int i = 0; i < 7; i++)
        {
          joint_position_cmd[i] = current_joint_state.position[i];
        }
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
  if (fsm_state == FsmState::CartesianMode)
  {
    cartesianState();
  }
  else if (fsm_state == FsmState::GotoHome)
  {
    gotoHomeState();
    fsm_state = FsmState::Idle;
  }
  else if (fsm_state == FsmState::GotoRest)
  {
    gotoRestState();
    fsm_state = FsmState::Idle;
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
    send6DofCommand();
    tool_controller_.sendGripperCommand(gripper_state_);
    ROS_INFO("    Done.");
  }
}

void CartesianController::gotoHomeState()
{
  const double positionsDesired[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  gotoPosition(positionsDesired);
}

void CartesianController::gotoRestState()
{
  const double positionsDesired[6] = { 0.0, 0.64, -1.397, 0.0, 0.0, 0.0 };
  gotoPosition(positionsDesired);
}

void CartesianController::gotoPosition(const double (&positionsDesired)[6])
{
  trajectory_msgs::JointTrajectory new_jt_traj;

  new_jt_traj.header.stamp = ros::Time::now();
  new_jt_traj.joint_names = current_joint_state.name;
  trajectory_msgs::JointTrajectoryPoint point;

  point.positions.resize(6);
  point.positions[0] = positionsDesired[0];
  point.positions[1] = positionsDesired[1];
  point.positions[2] = positionsDesired[2];
  point.positions[3] = positionsDesired[3];
  point.positions[4] = positionsDesired[4];
  point.positions[5] = positionsDesired[5];

  point.velocities.resize(6);
  point.velocities[0] = 0.0;
  point.velocities[1] = 0.0;
  point.velocities[2] = 0.0;
  point.velocities[3] = 0.0;
  point.velocities[4] = 0.0;
  point.velocities[5] = 0.0;

  point.time_from_start = ros::Duration(computeDuration(positionsDesired));

  new_jt_traj.points.push_back(point);
  command_pub_.publish(new_jt_traj);
}

double CartesianController::computeDuration(const double (&positionsDesired)[6])
{
  double delta_tmp = 0.0, delta_max = 0.0, duration = 0.0;
  for (int i = 0; i < 6; i++)
  {
    double delta_tmp = std::abs(positionsDesired[i] - current_joint_state.position[i]);
    if (delta_tmp > delta_max)
    {
      delta_max = delta_tmp;
    }
  }
  duration = delta_max / MAX_VELOCITY;
  return duration;
}

void CartesianController::send6DofCommand()
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
void CartesianController::jointStatesCB(const sensor_msgs::JointStateConstPtr& msg)
{
  current_joint_state = *msg;
}

void CartesianController::dxDesCB(const geometry_msgs::TwistStampedPtr& msg)
{
  //   ROS_DEBUG_STREAM("dxDesCB");
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

void CartesianController::learningModeCB(const std_msgs::BoolPtr& msg)
{
  learning_mode = msg->data;
}

void CartesianController::gripperCB(const std_msgs::BoolPtr& msg)
{
  gripper_state_ = msg->data;
}

bool CartesianController::enableCB(niryo_one_msgs::SetInt::Request& req, niryo_one_msgs::SetInt::Response& res)
{
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

using namespace cartesian_controller;
int main(int argc, char** argv)
{
  ros::init(argc, argv, "cartesian_controller");

  CartesianController cartesian_controller;

  return 0;
}
