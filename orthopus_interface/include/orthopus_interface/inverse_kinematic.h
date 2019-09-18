/*
  TODO copyright
*/

#ifndef CARTESIAN_CONTROLLER_INVERSE_KINEMATIC_H
#define CARTESIAN_CONTROLLER_INVERSE_KINEMATIC_H

#include <ros/ros.h>

// #include "geometry_msgs/TwistStamped.h"
// #include "sensor_msgs/Joy.h"
// #include "std_msgs/Bool.h"

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

// QPOASES
#include <qpOASES.hpp>

// Eigen3
#include <Eigen/Dense>

// Messages
#include <geometry_msgs/TwistStamped.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

typedef Eigen::Matrix<double, 6, 6, Eigen::RowMajor> Matrix6d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

namespace cartesian_controller
{
class InverseKinematic
{
public:
  InverseKinematic();
  void Init(int sampling_freq,
            ros::Publisher& debug_pose_current_,
            ros::Publisher& debug_pose_desired_,
            ros::Publisher& debug_pose_meas_,
            ros::Publisher& debug_joint_desired_,
            ros::Publisher& debug_joint_min_limit_,
            ros::Publisher& debug_joint_max_limit_);
  void Reset(sensor_msgs::JointState& current_joint_state);

  void ResolveInverseKinematic(double (&joint_position_command)[6], sensor_msgs::JointState& current_joint_state,
                               double (&cartesian_velocity_desired)[6]);  // TODO should not use magic number

  void UpdateAxisConstraints();
  void RequestUpdateAxisConstraints(int axis);
  void RequestUpdateAxisConstraints(int axis, double tolerance);


protected:
private:
  ros::NodeHandle n_;
  ros::Publisher debug_pose_current_;
  ros::Publisher debug_pose_desired_;  
  ros::Publisher debug_pose_meas_;  
  ros::Publisher debug_joint_desired_;
  ros::Publisher debug_joint_min_limit_;
  ros::Publisher debug_joint_max_limit_;

  int sampling_freq_;
  double sampling_period_;
  
  double alpha_1, alpha_2, alpha_3, alpha_4, alpha_5, alpha_6, alpha_7;
  double beta_1, beta_2, beta_3, beta_4, beta_5, beta_6;
  double gamma_1, gamma_2, gamma_3, gamma_4, gamma_5, gamma_6, gamma_7;
  double joints_limits_max[6];
  double joints_limits_min[6];
  // Weight for cartesian velocity minimization
  Matrix6d alpha_weight;
  // Weight for joint velocity minimization
  Matrix6d beta_weight;
  // Weight for cartesian position minimization
  Matrix6d gamma_weight;
  // QP solver
  qpOASES::QProblem* IK;

  robot_model::RobotModelPtr kinematic_model;
  robot_state::RobotStatePtr kinematic_state;
  robot_state::JointModelGroup* joint_model_group;
  
  bool qp_init_required = true;
  double theta[6];


  Vector6d x_min_limit;
  Vector6d x_max_limit;

  Vector6d x_cmd_prev;
  Vector6d x_cmd_prev_without_jump;
  Vector6d x_meas;
  Vector6d x_cmd_prev_saved;
  Vector6d x_computed; 
  Vector6d x_des; 
  Vector6d x_des_init; 
  
  int cartesian_mode_;

  geometry_msgs::Pose current_pose;

  tf2::Quaternion q_saved;
  tf2::Quaternion q_des;
  tf2::Quaternion q_rot, q_new;
bool request_update_constraint[6];
double request_update_constraint_tolerance[6];
    

  double xlimit_max;
  double ylimit_max;
  double zlimit_max;
  double xlimit_min;
  double ylimit_min;
  double zlimit_min;
  
  
  void PrintVector(const std::string name, const Vector6d vector) const;
  
};
}
#endif
