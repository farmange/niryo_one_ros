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

typedef Eigen::VectorXd VectorXd;
typedef Eigen::MatrixXd MatrixXd;

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
  enum class ControlFrame
  {
    World,
    Tool
  };

  InverseKinematic(const int joint_number);
  void init(const std::string end_effector_link, const double sampling_period);
  void reset();
  void resolveInverseKinematic(JointVelocity& dq_computed, const SpaceVelocity& dx_desired);
  void setQCurrent(const JointPosition& q_current);
  void setXCurrent(const SpacePosition& x_current);
  void setPositionControlFrame(const ControlFrame frame);
  void setOrientationControlFrame(const ControlFrame frame);
  const ControlFrame& getPositionControlFrame() const;
  const ControlFrame& getOrientationControlFrame() const;

protected:
private:
  ros::NodeHandle n_;
  int joint_number_;
  int space_dimension_;
  double sampling_period_;
  bool qp_init_required_; /*!< Flag to track the first iteration of QP solver */
  bool jacobian_init_flag_;

  ControlFrame position_ctrl_frame_;
  ControlFrame orientation_ctrl_frame_;

  Eigen::Quaterniond jacobian_quat_prev_;  // TODO used
  std::vector<double> gamma_weight_vec;

  JointPosition q_current_;  /*!< Current joint position */
  SpacePosition x_current_;  /*!< Current space position */
  VectorXd x_current_eigen_; /*!< Copy of x_current in eigen format (use in matrix computation) */
  VectorXd x_des;            /*!< TODO */

  bool reset_axis[7];
  bool reset_axis2[7];
  std::string end_effector_link_;

  Eigen::Vector3d pos_snap;
  double eps_inf_pos[3];

  JointPosition dq_lower_limit_; /*!< Joint lower velocity bound (vector lb) */
  JointPosition dq_upper_limit_; /*!< Joint upper velocity bound (vector ub) */

  JointPosition q_lower_limit_; /*!< Joint lower limit used in lower constraints bound vector lbA */
  JointPosition q_upper_limit_; /*!< Joint upper limit used in upper constraints bound vector ubA */

  // SpacePosition x_min_limit_; /*!< Space min limit used in lower constraints bound vector lbA */
  // SpacePosition x_max_limit_; /*!< Space max limit used in upper constraints bound vector ubA */
  double x_min_limit[7]; /*!< Space min limit used in lower constraints bound vector lbA */
  double x_max_limit[7]; /*!< Space max limit used in upper constraints bound vector ubA */

  MatrixXd alpha_weight_;   /*!< Diagonal matrix which contains weight for space velocity minimization */
  MatrixXd beta_weight_;    /*!< Diagonal matrix which contains weight for joint velocity minimization */
  MatrixXd gamma_weight_;   /*!< Diagonal matrix which contains weight for space position minimization */
  MatrixXd delta_weight_;   /*!< TODO */
  MatrixXd epsilon_weight_; /*!< TODO */

  qpOASES::SQProblem* QP_; /*!< QP solver instance pointer */

  robot_model::RobotModelPtr kinematic_model_;      /*!< MoveIt RobotModel pointer */
  robot_state::RobotStatePtr kinematic_state_;      /*!< MoveIt RobotState pointer */
  robot_state::JointModelGroup* joint_model_group_; /*!< MoveIt JointModelGroup pointer */

  Eigen::Quaterniond quat_des;
  bool flag_save[3];
  bool flag_orient_save[3];
  Eigen::Vector4d Qsnap[3];

  bool request_update_constraint_[6];
  double request_update_constraint_tolerance_[6];

  void setAlphaWeight_(const std::vector<double>& alpha_weight);
  void setBetaWeight_(const std::vector<double>& beta_weight);
  void setDqBounds_(const JointVelocity& dq_bound);

  /**
  * \brief Compute jacobian
  *
  * This is an updated version of MoveIt (RobotState class) implementation with quaternion discontinuity handling.
  */
  bool getJacobian_(const robot_state::RobotStatePtr kinematic_state, const robot_state::JointModelGroup* group,
                    const robot_state::LinkModel* link, const Eigen::Vector3d& reference_point_position,
                    Eigen::MatrixXd& jacobian, bool use_quaternion_representation);

  Eigen::Matrix4d xR(Eigen::Quaterniond& quat);
  Eigen::Matrix4d Rx(Eigen::Quaterniond& quat);

  Eigen::Quaterniond r_snap;
  Eigen::Quaterniond r_snap_cong;
  Eigen::MatrixXd Rs_cong;
};
}
#endif
