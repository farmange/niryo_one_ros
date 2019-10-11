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

#include "ros/ros.h"

#include "orthopus_space_control/types/joint_position.h"
#include "orthopus_space_control/types/joint_velocity.h"
#include "orthopus_space_control/types/space_position.h"
#include "orthopus_space_control/types/space_velocity.h"

// MoveIt!
#include "moveit/robot_model/robot_model.h"
#include "moveit/robot_model_loader/robot_model_loader.h"
#include "moveit/robot_state/robot_state.h"

// QPOASES
#include "qpOASES.hpp"

// Eigen
#include "Eigen/Dense"

typedef Eigen::Matrix<double, 6, 6, Eigen::RowMajor> Matrix6d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 7, 7, Eigen::RowMajor> Matrix7d;
typedef Eigen::Matrix<double, 7, 1> Vector7d;

namespace space_control
{
/**
* \brief Compute inverse kinematic
*
* This class computes inverse kinematic based on quadratic programming (QP). It relies on MoveIt RobotState and
* RobotModel to compute jacobian and qpOASES project (https://projects.coin-or.org/qpOASES/) to solve the QP
* optimization problem of the following form :
*     min   1/2*x'Hx + x'g
*     s.t.  lb  <=  x <= ub
*           lbA <= Ax <= ubA
*/
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
  void setDqBounds(const JointVelocity& dq_bound);

protected:
private:
  ros::NodeHandle n_;
  int joint_number_;
  bool use_quaternion_;
  double sampling_period_;
  bool qp_init_required_; /*!< Flag to track the first iteration of QP solver */

  JointPosition q_current_;  /*!< Current joint position */
  SpacePosition x_current_;  /*!< Current space position */
  Vector6d x_current_eigen_; /*!< Copy of x_current in eigen format (use in matrix computation) */
  Vector6d x_des;            /*!< TODO */

  std::string end_effector_link_;

  JointPosition dq_lower_limit_; /*!< Joint lower velocity bound (vector lb) */
  JointPosition dq_upper_limit_; /*!< Joint upper velocity bound (vector ub) */

  JointPosition q_lower_limit_; /*!< Joint lower limit used in lower constraints bound vector lbA */
  JointPosition q_upper_limit_; /*!< Joint upper limit used in upper constraints bound vector ubA */

  SpacePosition x_min_limit_; /*!< Space min limit used in lower constraints bound vector lbA */
  SpacePosition x_max_limit_; /*!< Space max limit used in upper constraints bound vector ubA */

  Matrix6d alpha_weight_; /*!< Diagonal matrix which contains weight for space velocity minimization */
  Matrix6d beta_weight_;  /*!< Diagonal matrix which contains weight for joint velocity minimization */
  Matrix6d gamma_weight_; /*!< Diagonal matrix which contains weight for space position minimization */

  qpOASES::SQProblem* QP_; /*!< QP solver instance pointer */

  robot_model::RobotModelPtr kinematic_model_;      /*!< MoveIt RobotModel pointer */
  robot_state::RobotStatePtr kinematic_state_;      /*!< MoveIt RobotState pointer */
  robot_state::JointModelGroup* joint_model_group_; /*!< MoveIt JointModelGroup pointer */

  bool request_update_constraint_[6];
  double request_update_constraint_tolerance_[6];

  void updateAxisConstraints_();
  void setAlphaWeight_(const std::vector<double>& alpha_weight);
  void setBetaWeight_(const std::vector<double>& beta_weight);
  void setGammaWeight_(const std::vector<double>& gamma_weight);
};
}
#endif
