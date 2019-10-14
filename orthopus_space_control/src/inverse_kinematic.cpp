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

  if (use_quaternion_)
  {
    space_dimension_ = 7;
  }
  else
  {
    space_dimension_ = 6;
  }

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

  /*
   * To limit the joint velocity, we define a constraint for the QP optimisation. Assuming a commun limit of dq_max for
   * all joints, we could write the lower and upper bounds as :
   *        (-dq_max) < dq < dq_max
   * so :
   *      lb = -dq_max
   *      ub = dq_max
   */
  ROS_DEBUG("Setting up bounds on joint velocity (dq) of the QP");
  double dq_max;
  ros::param::get("~joint_max_vel", dq_max);
  JointVelocity limit = JointVelocity(joint_number_);
  for (int i = 0; i < joint_number_; i++)
  {
    limit[i] = dq_max;
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
  // Minimize cartesian velocity (dx) weight
  alpha_weight_ = MatrixXd::Identity(space_dimension_, space_dimension_);
  for (int i = 0; i < space_dimension_; i++)
  {
    alpha_weight_(i, i) = alpha_weight[i];
  }
  ROS_DEBUG_STREAM("Set alpha weight to : \n" << alpha_weight_ << "\n");
}

void InverseKinematic::setBetaWeight_(const std::vector<double>& beta_weight)
{
  // Minimize joint velocity (dq) weight
  beta_weight_ = MatrixXd::Identity(joint_number_, joint_number_);
  for (int i = 0; i < joint_number_; i++)
  {
    beta_weight_(i, i) = beta_weight[i];
  }
  ROS_DEBUG_STREAM("Set alpha weight to : \n" << beta_weight_ << "\n");
}

void InverseKinematic::setGammaWeight_(const std::vector<double>& gamma_weight)
{
  // Minimize cartesian velocity : dx
  gamma_weight_ = MatrixXd::Identity(space_dimension_, space_dimension_);
  for (int i = 0; i < space_dimension_; i++)
  {
    gamma_weight_(i, i) = gamma_weight[i];
  }
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
  the current state is also copied in an eigen vector */
  x_current_eigen_ = VectorXd(space_dimension_);
  for (int i = 0; i < space_dimension_; i++)
  {
    x_current_eigen_(i) = x_current[i];
  }
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

  updateAxisConstraints_();

  /* Set kinemtic state of the robot to the current joint positions */
  kinematic_state_->setVariablePositions(q_current_);
  kinematic_state_->updateLinkTransforms();
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
  VectorXd dx_desired_eigen = VectorXd(space_dimension_);
  for (int i = 0; i < space_dimension_; i++)
  {
    dx_desired_eigen(i) = dx_desired[i];
  }

  /*
   * qpOASES solves QPs of the following form :
   * [1]    min   1/2*x'Hx + x'g
   *        s.t.  lb  <=  x <= ub
   *             lbA <= Ax <= ubA
   * where :
   *      - x' is the transpose of x
   *      - H is the hesiian matrix
   *      - g is the gradient vector
   *      - lb and ub are respectively the lower and upper bound constraint vectors
   *      - lbA and ubA are respectively the lower and upper inequality constraint vectors
   *      - A is the constraint matrix
   *
   * .:!:. NOTE : the symbol ' refers to the transpose of a matrix
   *
   * === Context ==============================================================================================
   * Usually, to resolve inverse kinematic, people use the well-known inverse jacobian formula :
   *        dX = J.dq     =>      dq = J^-1.dX
   * where :
   *      - dX is the cartesian velocity
   *      - dq is the joint velocity
   *      - J the jacobian
   * Unfortunatly, this method lacks when robot is in singular configuration and can lead to unstabilities.
   * ==========================================================================================================
   *
   * In our case, to resolve the inverse kinematic, we use a QP to minimize the joint velocity (dq). This minimization
   * intends to follow a cartesian velocity (dX_des) and position (X_des) trajectory while minimizing joint
   * velocity (dq) :
   *        min(dq) (1/2 * || dX - dX_des ||_alpha^2 + 1/2 * || dq ||_beta^2 + 1/2 * || X - X_des ||_gamma^2)
   * where :
   *      - alpha is the cartesian velocity weight matrix
   *      - beta is the joint velocity weight matrix
   *      - gamma is the cartesian position weight matrix
   *
   * Knowing that dX = J.dq, we have :
   * [2]    min(dq) (1/2 * || J.dq - dX_des ||_alpha^2 + 1/2 * || dq ||_beta^2 + 1/2 * || X - X_des ||_gamma^2)
   *
   * In order to reduce X, we perform a Taylor development (I also show how I develop this part):
   * [3]      1/2 * || X - X_des ||_gamma^2 = 1/2 * || X_0 + T.dX - X_des ||_gamma^2
   *                                      = 1/2 * || X_0 + T.J.dq - X_des ||_gamma^2
   *                                      = 1/2 * (X_0'.gamma.X_0 + dq'.J'.gamma.J.dq*T^2 + X_des'.gamma.X_des)
   *                                      + dq'.J'.gamma.X_0*T - X_des'.gamma.X_0 - X_des'.gamma.J.dq*T
   * where :
   *      - X_0 is the initial position
   *
   * Then, after developing the rest of the equation [2] as shown in [3]:
   *        min(dq) (1/2 * dq'.(J'.alpha.J + beta + J'.gamma.J*T^2).dq
   *               + dq'(-J'.alpha.dX_des + (J'.gamma.X_0 - J'.gamma.Xdes)*T))
   *
   * After identification with [1], we have :
   *        H = J'.alpha.J + beta + J'.gamma.J*T^2
   *        g = -J'.alpha.dX_des + (J'.gamma.X_0 - J'.gamma.Xdes)*T
   *
   */

  /* Hessian computation */
  MatrixXd hessian = (jacobian.transpose() * alpha_weight_ * jacobian) + beta_weight_ +
                     (jacobian.transpose() * sampling_period_ * gamma_weight_ * sampling_period_ * jacobian);

  /* Gradient vector computation */
  VectorXd g = (-jacobian.transpose() * alpha_weight_ * dx_desired_eigen) +
               (jacobian.transpose() * sampling_period_ * gamma_weight_ * x_current_eigen_) -
               (jacobian.transpose() * sampling_period_ * gamma_weight_ * x_des);

  /* In order to limit the joint position, we define a inequality constraint for the QP optimisation.
   * Taylor developpement of joint position is :
   *        q = q_0 + dq*T
   * with
   *      - q_0 the initial joint position.
   *
   * So affine inequality constraint could be written as :
   *        q_min < q < q_max
   *        q_min < q0 + dq*T < q_max
   *        (q_min-q0) < dq.T < (q_max-q0)
   *
   * so :
   *        A = I.T
   *        lbA = q_min-q0
   *        ubA = q_max-q0
   * where :
   *      - I is the identity matrix
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

  /* Solve QP */
  qpOASES::real_t xOpt[6];
  qpOASES::int_t nWSR = 10;
  int qp_return = 0;
  if (qp_init_required_)
  {
    /* Initialize QP solver */
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

    /* Get solution of the QP */
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
    exit(0);  // TODO improve error handling. Crash of the application is neither safe nor beautiful
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

      /*
       * Update the current desired cartesian position.
       * This allows the axis to drift and so to be control.
       * It is a dirty way of handle that !
       */
      x_des(i, 0) = x_current_eigen_(i, 0);
      ROS_WARN_STREAM("Update the new desired position of " << i << " to " << x_des(i, 0));
    }
  }
}

void InverseKinematic::requestUpdateAxisConstraints(int axis, double tolerance)
{
  ROS_WARN_STREAM("Update axis " << axis << " constraints with new tolerance of " << tolerance << "");
  request_update_constraint_[axis] = true;
  request_update_constraint_tolerance_[axis] = tolerance;
}
}
