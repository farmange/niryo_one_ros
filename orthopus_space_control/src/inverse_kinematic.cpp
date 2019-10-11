/*
 *  inverse_kinematic.cpp
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
#include <ros/ros.h>

#include "orthopus_space_control/inverse_kinematic.h"

#include <eigen_conversions/eigen_msg.h>
#include <tf/tf.h>

namespace space_control
{
InverseKinematic::InverseKinematic(const int joint_number, const bool use_quaternion)
  : joint_number_(joint_number)
  , use_quaternion_(use_quaternion)
  , x_current_(use_quaternion)
  , q_current_(joint_number)
  , q_upper_limit_(joint_number)
  , q_lower_limit_(joint_number)
  , dq_upper_limit_(joint_number)
  , dq_lower_limit_(joint_number)
  , x_min_limit_(use_quaternion)
  , x_max_limit_(use_quaternion)
  , sampling_period_(0)
  , qp_init_required_(true)
{
  ROS_DEBUG_STREAM("InverseKinematic constructor");

  std::vector<double> alpha_weight_vec;
  std::vector<double> beta_weight_vec;
  std::vector<double> gamma_weight_vec;
  ros::param::get("~alpha_weight", alpha_weight_vec);
  ros::param::get("~beta_weight", beta_weight_vec);
  ros::param::get("~gamma_weight", gamma_weight_vec);

  setAlphaWeight_(alpha_weight_vec);
  setBetaWeight_(beta_weight_vec);
  setGammaWeight_(gamma_weight_vec);

  ROS_DEBUG("Setting up bounds on joint position (q) of the QP");
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j1/min", q_lower_limit_[0]);
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j1/max", q_upper_limit_[0]);
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j2/min", q_lower_limit_[1]);
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j2/max", q_upper_limit_[1]);
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j3/min", q_lower_limit_[2]);
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j3/max", q_upper_limit_[2]);
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j4/min", q_lower_limit_[3]);
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j4/max", q_upper_limit_[3]);
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j5/min", q_lower_limit_[4]);
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j5/max", q_upper_limit_[4]);
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j6/min", q_lower_limit_[5]);
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j6/max", q_upper_limit_[5]);
  /* HACK: Limit joint_3 max reach to prevent a singularity observe in some cases. */
  q_upper_limit_[2] = 1.4;

  ROS_DEBUG("Setting up bounds on joint velocity (dq) of the QP");
  double joint_max_vel;
  ros::param::get("~joint_max_vel", joint_max_vel);
  JointVelocity limit = JointVelocity(joint_number_);
  for (int i = 0; i < joint_number_; i++)
  {
    limit[i] = joint_max_vel;
  }
  setDqBounds(limit);

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  kinematic_model_ = robot_model_loader.getModel();
  kinematic_state_ = std::make_shared<robot_state::RobotState>(kinematic_model_);
  kinematic_state_->setToDefaultValues();
  joint_model_group_ = kinematic_model_->getJointModelGroup("arm");

  // TODO Comment that part
  for (int i = 0; i < 6; i++)
  {
    x_min_limit_[i] = 0.0;
    x_max_limit_[i] = 0.0;
  }
}

void InverseKinematic::init(const std::string end_effector_link, const double sampling_period,
                            const bool use_quaternion)
{
  use_quaternion_ = use_quaternion;
  end_effector_link_ = end_effector_link;
  sampling_period_ = sampling_period;
}

void InverseKinematic::setAlphaWeight_(const std::vector<double>& alpha_weight)
{
  // Minimize cartesian velocity : dx
  alpha_weight_ = Matrix6d::Identity(6, 6);
  alpha_weight_(0, 0) = alpha_weight[0];
  alpha_weight_(1, 1) = alpha_weight[1];
  alpha_weight_(2, 2) = alpha_weight[2];
  alpha_weight_(3, 3) = alpha_weight[3];
  alpha_weight_(4, 4) = alpha_weight[4];
  alpha_weight_(5, 5) = alpha_weight[5];
  ROS_DEBUG_STREAM("Set alpha weight to : \n" << alpha_weight_ << "\n");
}

void InverseKinematic::setBetaWeight_(const std::vector<double>& beta_weight)
{
  // Minimize cartesian velocity : dx
  beta_weight_ = Matrix6d::Identity(6, 6);
  beta_weight_(0, 0) = beta_weight[0];
  beta_weight_(1, 1) = beta_weight[1];
  beta_weight_(2, 2) = beta_weight[2];
  beta_weight_(3, 3) = beta_weight[3];
  beta_weight_(4, 4) = beta_weight[4];
  beta_weight_(5, 5) = beta_weight[5];
  ROS_DEBUG_STREAM("Set alpha weight to : \n" << beta_weight_ << "\n");
}

void InverseKinematic::setGammaWeight_(const std::vector<double>& gamma_weight)
{
  // Minimize cartesian velocity : dx
  gamma_weight_ = Matrix6d::Identity(6, 6);
  gamma_weight_(0, 0) = gamma_weight[0];
  gamma_weight_(1, 1) = gamma_weight[1];
  gamma_weight_(2, 2) = gamma_weight[2];
  gamma_weight_(3, 3) = gamma_weight[3];
  gamma_weight_(4, 4) = gamma_weight[4];
  gamma_weight_(5, 5) = gamma_weight[5];
  ROS_DEBUG_STREAM("Set alpha weight to : \n" << gamma_weight_ << "\n");
}

void InverseKinematic::setQCurrent(const JointPosition& q_current)
{
  q_current_ = q_current;
}

void InverseKinematic::setXCurrent(const SpacePosition& x_current)
{
  x_current_ = x_current;
  /* As most of computation in this class are eigen matrix operation,
  the current state is also copied in eigen vector */
  x_current_eigen_(0, 0) = x_current[0];
  x_current_eigen_(1, 0) = x_current[1];
  x_current_eigen_(2, 0) = x_current[2];
  x_current_eigen_(3, 0) = x_current[3];
  x_current_eigen_(4, 0) = x_current[4];
  x_current_eigen_(5, 0) = x_current[5];
}

void InverseKinematic::setDqBounds(const JointVelocity& dq_bound)
{
  for (int i = 0; i < dq_bound.size(); i++)
  {
    dq_lower_limit_[i] = -dq_bound[i];
    dq_upper_limit_[i] = dq_bound[i];
  }
}

void InverseKinematic::reset()
{
  /* Initialize a flag used to init QP if required */
  qp_init_required_ = true;

  /* Initialize all cartesian constraints */
  for (int i = 0; i < 6; i++)
  {
    request_update_constraint_[i] = true;
    request_update_constraint_tolerance_[i] = 0.001;
  }
}

void InverseKinematic::resolveInverseKinematic(JointVelocity& dq_computed, const SpaceVelocity& dx_desired)
{
  if (qp_init_required_)
  {
    x_des = x_current_eigen_;
  }

  /* Set kinemtic state of the robot to the previous joint positions computed */
  kinematic_state_->setVariablePositions(q_current_);
  kinematic_state_->updateLinkTransforms();

  updateAxisConstraints_();

  /* Get jacobian from kinematic state (Moveit) */
  Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
  Eigen::MatrixXd jacobian;
  if (!kinematic_state_->getJacobian(joint_model_group_, kinematic_state_->getLinkModel(end_effector_link_),
                                     reference_point_position, jacobian))
  {
    ROS_ERROR("Jacobian computation issue !");
    for (int i = 0; i < dq_computed.size(); i++)
    {
      dq_computed[i] = 0.0;
    }
    return;
  }

  /* Store desired velocity in eigen vector */
  const Vector6d dx_desired_eigen = Vector6d(dx_desired.data());

  /* Hessian computation */
  Matrix6d hessian = (jacobian.transpose() * alpha_weight_ * jacobian) + beta_weight_ +
                     (jacobian.transpose() * sampling_period_ * gamma_weight_ * sampling_period_ * jacobian);

  /* Gradient vector computation */
  Vector6d g = (-jacobian.transpose() * alpha_weight_ * dx_desired_eigen) +
               (jacobian.transpose() * sampling_period_ * gamma_weight_ * x_current_eigen_) -
               (jacobian.transpose() * sampling_period_ * gamma_weight_ * x_des);

  /*
   * Taylor developpement of joint position is :
   *           q = q0 + dq*T
   * with q0 the initial joint position.
   *
   * So affine inequality constraint could be written as :
   *           lb < q < ub
   *       lb < q0 + dq*T < ub
   *    (lb-q0) < dq.T < (ub-q0)
   */
  Eigen::Matrix<double, 12, 6, Eigen::RowMajor> A;
  A.topLeftCorner(6, 6) = Eigen::MatrixXd::Identity(6, 6) * sampling_period_;
  A.bottomLeftCorner(6, 6) = jacobian.topLeftCorner(6, 6) * sampling_period_;

  double lbA[] = { /* Joints min hard limits constraints */
                   (q_lower_limit_[0] - q_current_[0]),        (q_lower_limit_[1] - q_current_[1]),
                   (q_lower_limit_[2] - q_current_[2]),        (q_lower_limit_[3] - q_current_[3]),
                   (q_lower_limit_[4] - q_current_[4]),        (q_lower_limit_[5] - q_current_[5]),
                   (x_min_limit_[0] - x_current_eigen_(0, 0)), (x_min_limit_[1] - x_current_eigen_(1, 0)),
                   (x_min_limit_[2] - x_current_eigen_(2, 0)), (x_min_limit_[3] - x_current_eigen_(3, 0)),
                   (x_min_limit_[4] - x_current_eigen_(4, 0)), (x_min_limit_[5] - x_current_eigen_(5, 0))
  };

  double ubA[] = { /* Joints max hard limits constraints */
                   (q_upper_limit_[0] - q_current_[0]),        (q_upper_limit_[1] - q_current_[1]),
                   (q_upper_limit_[2] - q_current_[2]),        (q_upper_limit_[3] - q_current_[3]),
                   (q_upper_limit_[4] - q_current_[4]),        (q_upper_limit_[5] - q_current_[5]),
                   (x_max_limit_[0] - x_current_eigen_(0, 0)), (x_max_limit_[1] - x_current_eigen_(1, 0)),
                   (x_max_limit_[2] - x_current_eigen_(2, 0)), (x_max_limit_[3] - x_current_eigen_(3, 0)),
                   (x_max_limit_[4] - x_current_eigen_(4, 0)), (x_max_limit_[5] - x_current_eigen_(5, 0))
  };

  // Solve first QP.
  qpOASES::real_t xOpt[6];
  qpOASES::int_t nWSR = 10;
  int qp_return = 0;
  if (qp_init_required_)
  {
    // Initialize QP solver
    QP_ = new qpOASES::SQProblem(6, 9);
    qpOASES::Options options;
    options.setToReliable();
    options.printLevel = qpOASES::PL_NONE;
    QP_->setOptions(options);
    qp_return = QP_->init(hessian.data(), g.data(), A.data(), dq_lower_limit_.data(), dq_upper_limit_.data(), lbA, ubA,
                          nWSR, 0);
    qp_init_required_ = false;
  }
  else
  {
    qp_return = QP_->hotstart(hessian.data(), g.data(), A.data(), dq_lower_limit_.data(), dq_upper_limit_.data(), lbA,
                              ubA, nWSR, 0);
  }

  if (qp_return == qpOASES::SUCCESSFUL_RETURN)
  {
    ROS_DEBUG_STREAM("qpOASES : succesfully return");

    // Get and print solution of first QP
    QP_->getPrimalSolution(xOpt);
    dq_computed[0] = xOpt[0];
    dq_computed[1] = xOpt[1];
    dq_computed[2] = xOpt[2];
    dq_computed[3] = xOpt[3];
    dq_computed[4] = xOpt[4];
    dq_computed[5] = xOpt[5];
  }
  else
  {
    ROS_ERROR("qpOASES : Failed with code : %d !", qp_return);
    dq_computed[0] = 0.0;
    dq_computed[1] = 0.0;
    dq_computed[2] = 0.0;
    dq_computed[3] = 0.0;
    dq_computed[4] = 0.0;
    dq_computed[5] = 0.0;
  }

  if (qp_return != qpOASES::SUCCESSFUL_RETURN && qp_return != qpOASES::RET_MAX_NWSR_REACHED)
  {
    exit(0);  // TODO return state to signal error instead of exit process
  }
}

void InverseKinematic::updateAxisConstraints_()
{
  for (int i = 0; i < 6; i++)
  {
    if (request_update_constraint_[i])
    {
      ROS_WARN_STREAM("Update axis " << i << " constraints with new tolerance of "
                                     << request_update_constraint_tolerance_[i] << " m.");

      x_min_limit_[i] = x_current_eigen_(i, 0) - request_update_constraint_tolerance_[i];
      x_max_limit_[i] = x_current_eigen_(i, 0) + request_update_constraint_tolerance_[i];

      request_update_constraint_[i] = false;

      /* Update the current desired cartesian position.
       * This allows the axis to drift and so to be control.
       * It is a dirty way of handle that !
       */
      x_des(i, 0) = x_current_eigen_(i, 0);
      ROS_WARN_STREAM("Update the new desired position of " << i << " to " << x_des(i, 0));
    }
  }
}

// Keep last set tolerance
void InverseKinematic::requestUpdateAxisConstraints(int axis)
{
  ROS_WARN_STREAM("Update axis " << axis << " constraints with the last tolerance set ("
                                 << request_update_constraint_tolerance_[axis] << ")");
  request_update_constraint_[axis] = true;
}
void InverseKinematic::requestUpdateAxisConstraints(int axis, double tolerance)
{
  ROS_WARN_STREAM("Update axis " << axis << " constraints with new tolerance of " << tolerance << "");
  request_update_constraint_[axis] = true;
  request_update_constraint_tolerance_[axis] = tolerance;
}
}
