// TODO copyright
#include <ros/ros.h>

#include "std_msgs/Bool.h"

#include "orthopus_interface/cartesian_controller.h"

#define RATE 10
#define TOOL_ID_GRIPPER_1 11
#define INIT_TIME 5

namespace cartesian_controller
{
CartesianController::CartesianController()
{
  ROS_INFO("CartesianController constructor");
  ros::Rate loop_rate = ros::Rate(RATE);

  joints_sub_ = n_.subscribe("joint_states", 1, &CartesianController::jointStatesCB, this);
  dx_des_sub_ = n_.subscribe("dx_des", 1, &CartesianController::dxDesCB, this);
  gripper_sub_ = n_.subscribe("gripper_des", 1, &CartesianController::gripperCB, this);

  command_pub_ = n_.advertise<trajectory_msgs::JointTrajectory>("/niryo_one_follow_joint_trajectory_controller/command", 10);  // TODO check optimal queue size
  debug_pub_ = n_.advertise<geometry_msgs::Pose>("/debug_cartesian_pos", 1);
  debug_des_pub_ = n_.advertise<geometry_msgs::Pose>("/debug_cartesian_pos_des", 1);

  tool_controller_.setToolId(TOOL_ID_GRIPPER_1);
  
  ROS_INFO("Initialize robot position");
  // initialize robot in zero position
  while (ros::ok())
  {
    ros::spinOnce();
    joint_position_cmd[0] = 0.0;
    joint_position_cmd[1] = 0.0;
    joint_position_cmd[2] = 0.0;
    joint_position_cmd[3] = 0.0;
    joint_position_cmd[4] = 0.0;
    joint_position_cmd[5] = 0.0;
    sendInitCommand();
    ros::Duration(INIT_TIME + 1.0).sleep();
    break;
  }

  while (ros::ok())
  {
    ros::spinOnce();
    
    ROS_INFO("=== Start IK computation...");
    ik_.ResolveInverseKinematic(joint_position_cmd, current_joint_state, cartesian_velocity_desired);
    ROS_INFO("    Done.");

    ROS_INFO("=== Send Niryo One commands...");
    send6DofCommand();
    tool_controller_.sendGripperCommand(gripper_state_);
    ROS_INFO("    Done.");

    loop_rate.sleep();
  }
}

void CartesianController::sendInitCommand()
{
  trajectory_msgs::JointTrajectory new_jt_traj;

  new_jt_traj.header.stamp = ros::Time::now();
  ROS_INFO_STREAM("new_jt_traj=" << new_jt_traj);
  const std::vector<std::string>& joint_names = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" };
  new_jt_traj.joint_names = joint_names;
  ROS_INFO_STREAM("new_jt_traj=" << new_jt_traj);
  trajectory_msgs::JointTrajectoryPoint point;
  point.time_from_start = ros::Duration(INIT_TIME);
  const std::vector<double>& positions = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  point.positions = positions;

  // Important : do not send velocity as cubic interpolation is done !
  // It should be better to handle that by proper velocity command (force velocity ramp on xbox controller)
  // point.velocities = current_joint_state.velocity;
  new_jt_traj.points.push_back(point);
  ROS_INFO_STREAM("new_jt_traj=" << new_jt_traj);
  command_pub_.publish(new_jt_traj);
};

void CartesianController::send6DofCommand()
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
  ROS_DEBUG_STREAM("current_joint_state: \n" << current_joint_state << "\n");
}

void CartesianController::dxDesCB(const geometry_msgs::TwistStampedPtr& msg)
{
  cartesian_velocity_desired[0] = msg->twist.linear.x;
  cartesian_velocity_desired[1] = msg->twist.linear.y;
  cartesian_velocity_desired[2] = msg->twist.linear.z;
  cartesian_velocity_desired[3] = msg->twist.angular.x;
  cartesian_velocity_desired[4] = msg->twist.angular.y;
  cartesian_velocity_desired[5] = msg->twist.angular.z;
}

  void CartesianController::gripperCB(const std_msgs::BoolPtr& msg)
  {
    gripper_state_ = msg->data;
  }

}

using namespace cartesian_controller;
int main(int argc, char** argv)
{
  ros::init(argc, argv, "cartesian_controller");

  CartesianController cartesian_controller;

  return 0;
}
