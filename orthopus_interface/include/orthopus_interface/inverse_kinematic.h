/*
 *  inverse_kinematic.h
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
#ifndef CARTESIAN_CONTROLLER_INVERSE_KINEMATIC_H
#define CARTESIAN_CONTROLLER_INVERSE_KINEMATIC_H

#include <ros/ros.h>

#include <orthopus_interface/types/joint_position.h>
#include <orthopus_interface/types/joint_velocity.h>
#include <orthopus_interface/types/space_velocity.h>
#include <orthopus_interface/types/space_position.h>

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
  InverseKinematic(const int joint_number, const bool use_quaternion);
  void init(const std::string end_effector_link, const double sampling_period, const bool use_quaternion = false);

  void reset();

  void resolveInverseKinematic(JointVelocity& dq_computed, const SpaceVelocity& dx_desired);

  void requestUpdateAxisConstraints(int axis);
  void requestUpdateAxisConstraints(int axis, double tolerance);

  void setQCurrent(const JointPosition& q_current);
  void setXCurrent(const SpacePosition& x_current);

  void setDqBounds(const JointVelocity& dq_bounds);

  void setAlphaWeight(const std::vector<double>& alpha_weight);
  void setBetaWeight(const std::vector<double>& beta_weight);
  void setGammaWeight(const std::vector<double>& gamma_weight);

protected:
private:
  ros::NodeHandle n_;

  JointPosition q_current_;
  SpacePosition x_current_;
  /* Copy of x_current_ pose in eigen vector format */
  Vector6d x_current_eigen_;
  Vector6d x_des;

  /* Set joint velocities bound */
  double lower_bounds_[6];
  double upper_bounds_[6];

  double sampling_period_;

  std::string end_effector_link_;

  double alpha_1, alpha_2, alpha_3, alpha_4, alpha_5, alpha_6, alpha_7;
  double beta_1, beta_2, beta_3, beta_4, beta_5, beta_6;
  double gamma_1, gamma_2, gamma_3, gamma_4, gamma_5, gamma_6, gamma_7;
  double q_limit_max_[6];
  double q_limit_min_[6];
  // Weight for cartesian velocity minimization
  Matrix6d alpha_weight_;
  // Weight for joint velocity minimization
  Matrix6d beta_weight_;
  // Weight for cartesian position minimization
  Matrix6d gamma_weight_;
  // QP solver
  qpOASES::SQProblem* IK;

  robot_model::RobotModelPtr kinematic_model;
  robot_state::RobotStatePtr kinematic_state;
  robot_state::JointModelGroup* joint_model_group;

  bool qp_init_required_;
  double theta[6];
  double joint_max_vel_;

  Vector6d x_min_limit;
  Vector6d x_max_limit;

  Vector6d x_cmd_prev_without_jump;
  Vector6d x_meas;
  Vector6d x_cmd_prev_saved;
  Vector6d x_computed;
  Vector6d x_des_init;

  bool use_quaternion_;
  int joint_number_;

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

  void updateAxisConstraints();
  void printVector(const std::string name, const Vector6d vector) const;
};
}
#endif
