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
  void init(int sampling_freq, ros::Publisher& debug_pose_current, ros::Publisher& debug_pose_desired,
            ros::Publisher& debug_pose_meas, double max_vel = 1.0);
  void activeConstraints(double (&dx_response)[6], const double (&dx_request)[6], const double (&x_request)[6],
                         bool force_xyz_constraints);
  void executeTrajectory(double (&cartesian_velocity_computed)[6], const double (&x_request)[6]);
  void run(double (&dx_response)[6]);
  void reset();
  bool isTrajectoryRequested();
  bool isTrajectoryCompleted();

  void setJointState(sensor_msgs::JointState& current_joint_state);
  void setVelocitiesCommand(const double (&cartesian_velocity_desired)[6]);
  void setTrajectoryPose(const geometry_msgs::Pose& traj_des_pose);

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

  double velocity_command_[6];
  double velocity_command_prev_[6];

  double dx_trajectory_[6];
  double dx_trajectory_prev_[6];

  bool constraint_axis_[6];
  bool constraint_axis_prev_[6];
  bool traj_requested_;

  double prev_xyz_rpy_[6];
  double current_xyz_rpy_[6];
  double desired_xyz_rpy_[6];

  /* Constraint PI controller */
  double constraint_ctrl_p_gain_;
  double constraint_ctrl_i_gain_;
  double constraint_ctrl_i_sum_[6];

  double euler_factor_[6];

  /* Trajectory PI controller */
  double trajectory_ctrl_p_gain_;
  double trajectory_ctrl_i_gain_;
  double trajectory_ctrl_i_sum_[6];

  double traj_desired_pose_[6];
  double traj_position_tolerance_;
  double traj_orientation_tolerance_;

  double pi_min_, pi_max_;

  double cartesian_max_vel_;

  void updateContraints(const double (&dx_des)[6]);
  void rolloverHandling();
  void updateDesiredPosition();
  bool trajectoryIsAchieved();

  enum
  {
    X_AXIS = 0,
    Y_AXIS,
    Z_AXIS,
    ROLL_AXIS,
    PITCH_AXIS,
    YAW_AXIS
  };
};
}
#endif
