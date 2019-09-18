/*
  TODO copyright
*/
#ifndef CARTESIAN_CONTROLLER_POSITION_COMPENSATOR_H
#define CARTESIAN_CONTROLLER_POSITION_COMPENSATOR_H

#include "ros/ros.h"

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

namespace cartesian_controller
{
  class PositionCompensator
{
public:
  PositionCompensator();
  void init(int sampling_freq,
            ros::Publisher& debug_pose_current,
            ros::Publisher& debug_pose_desired,
            ros::Publisher& debug_pose_meas,
            double max_vel = 1.0);
  void run(double (&cartesian_velocity_compensated)[6], const double (&cartesian_velocity_desired)[6], sensor_msgs::JointState& current_joint_state);
  void reset();
  
protected:
private:
  ros::NodeHandle n_;
  ros::Publisher debug_pose_current_;
  ros::Publisher debug_pose_desired_;  
  ros::Publisher debug_pose_meas_;  
  
  int sampling_freq_;
  double sampling_period_;
  robot_model::RobotModelPtr kinematic_model;
  robot_state::RobotStatePtr kinematic_state;
  robot_state::JointModelGroup* joint_model_group;
  
  sensor_msgs::JointState current_joint_state_;
  bool free_position_[6];
  bool free_position_prev_[6];
  
  double prev_xyz_rpy_[6];
  double current_xyz_rpy_[6];
  double desired_xyz_rpy_[6];
  double proportional_gain_[6];
  double integral_gain_[6];
  double integral_sum_[6];
  
  double pi_min_, pi_max_;
  
  void detectFreePosition(const double (&cartesian_velocity_desired)[6]);
  void updateDesiredPosition();
  
};
}
#endif
