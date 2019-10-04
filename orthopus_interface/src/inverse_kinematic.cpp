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

#include "orthopus_interface/inverse_kinematic.h"

#include <eigen_conversions/eigen_msg.h>
#include <tf/tf.h>

namespace cartesian_controller
{
InverseKinematic::InverseKinematic(const int joint_number, const bool use_quaternion)
  : joint_number_(joint_number), use_quaternion_(use_quaternion), x_current_(use_quaternion), q_current_(joint_number)
{
  ROS_DEBUG_STREAM("InverseKinematic constructor");

  std::vector<double> alpha_weight_vec;
  std::vector<double> beta_weight_vec;
  std::vector<double> gamma_weight_vec;
  ros::param::get("~alpha_weight", alpha_weight_vec);
  ros::param::get("~beta_weight", beta_weight_vec);
  ros::param::get("~gamma_weight", gamma_weight_vec);

  setAlphaWeight(alpha_weight_vec);
  setBetaWeight(beta_weight_vec);
  setGammaWeight(gamma_weight_vec);

  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j1/min", q_limit_min_[0]);
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j1/max", q_limit_max_[0]);
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j2/min", q_limit_min_[1]);
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j2/max", q_limit_max_[1]);
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j3/min", q_limit_min_[2]);
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j3/max", q_limit_max_[2]);
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j4/min", q_limit_min_[3]);
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j4/max", q_limit_max_[3]);
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j5/min", q_limit_min_[4]);
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j5/max", q_limit_max_[4]);
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j6/min", q_limit_min_[5]);
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j6/max", q_limit_max_[5]);

  /* Force max limit of joint_3 to prevent singularity */
  q_limit_max_[2] = 1.4;

  ros::param::get("~joint_max_vel", joint_max_vel_);

  sampling_period_ = 0;
  qp_init_required_ = true;

  // Wait for initial messages
  ROS_INFO("Waiting for first joint msg.");
  ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states");
  ROS_INFO("Received first joint msg.");

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  kinematic_model = robot_model_loader.getModel();
  ROS_DEBUG("Model frame: %s", kinematic_model->getModelFrame().c_str());
  kinematic_state = std::make_shared<robot_state::RobotState>(kinematic_model);
  kinematic_state->setToDefaultValues();
  joint_model_group = kinematic_model->getJointModelGroup("arm");

  ROS_DEBUG("Setting up bound limit of the QP");
  JointVelocity limit = JointVelocity(joint_number_);
  for (int i = 0; i < joint_number_; i++)
  {
    limit[i] = joint_max_vel_;
  }
  ROS_DEBUG("DONE %d !!!!!!!!!!!!", joint_number_);
  setDqBounds(limit);
  ROS_DEBUG("DONE !!!!!!!!!!!!");

  for (int i = 0; i < 6; i++)
  {
    x_min_limit(i, 0) = 0.0;
    x_max_limit(i, 0) = 0.0;
  }
}

void InverseKinematic::init(const std::string end_effector_link, const double sampling_period,
                            const bool use_quaternion)
{
  use_quaternion_ = use_quaternion;
  end_effector_link_ = end_effector_link;
  sampling_period_ = sampling_period;
}

void InverseKinematic::setAlphaWeight(const std::vector<double>& alpha_weight)
{
  // TODO check size (quaternion or not)
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

void InverseKinematic::setBetaWeight(const std::vector<double>& beta_weight)
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

void InverseKinematic::setGammaWeight(const std::vector<double>& gamma_weight)
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
  x_current_eigen_(0, 0) = x_current[0];
  x_current_eigen_(1, 0) = x_current[1];
  x_current_eigen_(2, 0) = x_current[2];
  x_current_eigen_(3, 0) = x_current[3];
  x_current_eigen_(4, 0) = x_current[4];
  x_current_eigen_(5, 0) = x_current[5];
}

void InverseKinematic::setDqBounds(const JointVelocity& dq_bounds)
{
  for (int i = 0; i < dq_bounds.size(); i++)
  {
    lower_bounds_[i] = -dq_bounds[i];
    upper_bounds_[i] = dq_bounds[i];
  }
}

void InverseKinematic::reset()
{
  /* Initialize a flag used to init QP if required */
  qp_init_required_ = true;

  /* Initialize all cartesian constraints */
  for (int i = 0; i < 6; i++)
  {
    request_update_constraint[i] = true;
    request_update_constraint_tolerance[i] = 0.001;
  }
  //   updateAxisConstraints();
}

void InverseKinematic::resolveInverseKinematic(JointVelocity& dq_computed, const SpaceVelocity& dx_desired)
{
  if (qp_init_required_)
  {
    x_des = x_current_eigen_;
  }

  /* Set kinemtic state of the robot to the previous joint positions computed */
  kinematic_state->setVariablePositions(q_current_);
  kinematic_state->updateLinkTransforms();

  updateAxisConstraints();

  /* Get jacobian from kinematic state (Moveit) */
  Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
  Eigen::MatrixXd jacobian;
  if (!kinematic_state->getJacobian(joint_model_group, kinematic_state->getLinkModel(end_effector_link_),
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

  // Taylor developpement
  // q = q0 + dq*T
  // inequality is :
  // lb < q < ub
  // lb < q0 + dq*T < ub
  // (lb-q0) < dq.T < (ub-q0)
  Eigen::Matrix<double, 12, 6, Eigen::RowMajor> A;
  A.topLeftCorner(6, 6) = Eigen::MatrixXd::Identity(6, 6) * sampling_period_;
  A.bottomLeftCorner(6, 6) = jacobian.topLeftCorner(6, 6) * sampling_period_;

  double lbA[] = { /* Joints min hard limits constraints */
                   (q_limit_min_[0] - q_current_[0]),
                   (q_limit_min_[1] - q_current_[1]),
                   (q_limit_min_[2] - q_current_[2]),
                   (q_limit_min_[3] - q_current_[3]),
                   (q_limit_min_[4] - q_current_[4]),
                   (q_limit_min_[5] - q_current_[5]),
                   (x_min_limit(0, 0) - x_current_eigen_(0, 0)),
                   (x_min_limit(1, 0) - x_current_eigen_(1, 0)),
                   (x_min_limit(2, 0) - x_current_eigen_(2, 0)),
                   (x_min_limit(3, 0) - x_current_eigen_(3, 0)),
                   (x_min_limit(4, 0) - x_current_eigen_(4, 0)),
                   (x_min_limit(5, 0) - x_current_eigen_(5, 0))
  };

  double ubA[] = { /* Joints max hard limits constraints */
                   (q_limit_max_[0] - q_current_[0]),
                   (q_limit_max_[1] - q_current_[1]),
                   (q_limit_max_[2] - q_current_[2]),
                   (q_limit_max_[3] - q_current_[3]),
                   (q_limit_max_[4] - q_current_[4]),
                   (q_limit_max_[5] - q_current_[5]),
                   (x_max_limit(0, 0) - x_current_eigen_(0, 0)),
                   (x_max_limit(1, 0) - x_current_eigen_(1, 0)),
                   (x_max_limit(2, 0) - x_current_eigen_(2, 0)),
                   (x_max_limit(3, 0) - x_current_eigen_(3, 0)),
                   (x_max_limit(4, 0) - x_current_eigen_(4, 0)),
                   (x_max_limit(5, 0) - x_current_eigen_(5, 0))
  };

  // Solve first QP.
  qpOASES::real_t xOpt[6];
  qpOASES::int_t nWSR = 10;
  int qp_return = 0;
  if (qp_init_required_)
  {
    // Initialize QP solver
    IK = new qpOASES::SQProblem(6, 9);
    qpOASES::Options options;
    options.setToReliable();
    options.printLevel = qpOASES::PL_NONE;
    IK->setOptions(options);
    qp_return = IK->init(hessian.data(), g.data(), A.data(), lower_bounds_, upper_bounds_, lbA, ubA, nWSR, 0);
    qp_init_required_ = false;
  }
  else
  {
    qp_return = IK->hotstart(hessian.data(), g.data(), A.data(), lower_bounds_, upper_bounds_, lbA, ubA, nWSR, 0);
  }

  if (qp_return == qpOASES::SUCCESSFUL_RETURN)
  {
    ROS_DEBUG_STREAM("qpOASES : succesfully return");

    // Get and print solution of first QP
    IK->getPrimalSolution(xOpt);
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

  //   printVector("q_limit_min_", Vector6d(q_limit_min_));
  //   printVector("q_current_", Vector6d(q_current_.data()));
  //   printVector("q_limit_max_", Vector6d(q_limit_max_));
  //   printVector("dq_computed", Vector6d(dq_computed.data()));
  //   ROS_DEBUG_STREAM("================");
  // //   ROS_DEBUG_STREAM("Jacobian: \n" << jacobian);
  //   printVector("dx_desired_eigen", dx_desired_eigen);
  // //   ROS_DEBUG_STREAM("hessian: \n" << hessian );
  // //   ROS_DEBUG_STREAM("g = \n" << g);
  // //   ROS_DEBUG_STREAM("A: \n" << A );
  //   ROS_DEBUG_STREAM("================");
  //   printVector("x_min_limit", x_min_limit);
  //   printVector("x_current_eigen_", x_current_eigen_);
  //   printVector("x_max_limit", x_max_limit);
  //   ROS_DEBUG_STREAM("================");
  //   Vector6d dx_computed = Vector6d::Zero();
  //   dx_computed = jacobian*Vector6d(dq_computed.data());
  //   printVector("dx_computed", dx_computed);
  //   Vector6d j_dq_t = dx_computed*sampling_period_;
  //   ROS_DEBUG_STREAM("================");
  //   printVector("x_computed (old)", x_computed);
  //   x_computed = x_current_eigen_ + j_dq_t;
  //   ROS_DEBUG_STREAM("================");
  //   printVector("x_computed (new)", x_computed);
  //   ROS_DEBUG_STREAM("================");

  if (qp_return != qpOASES::SUCCESSFUL_RETURN && qp_return != qpOASES::RET_MAX_NWSR_REACHED)
  {
    exit(0);  // TODO return state to signal error instead of exit process
  }
}

void InverseKinematic::printVector(const std::string name, const Vector6d vector) const
{
  std::string reformatted_name = name;
  reformatted_name.resize(14, ' ');
  ROS_DEBUG("%s : %5f, \t%5f, \t%5f, \t%5f, \t%5f, \t%5f", reformatted_name.c_str(), vector(0, 0), vector(1, 0),
            vector(2, 0), vector(3, 0), vector(4, 0), vector(5, 0));
}

void InverseKinematic::updateAxisConstraints()
{
  for (int i = 0; i < 6; i++)
  {
    if (request_update_constraint[i])
    {
      ROS_WARN_STREAM("Update axis " << i << " constraints with new tolerance of "
                                     << request_update_constraint_tolerance[i] << " m.");

      x_min_limit(i, 0) = x_current_eigen_(i, 0) - request_update_constraint_tolerance[i];
      x_max_limit(i, 0) = x_current_eigen_(i, 0) + request_update_constraint_tolerance[i];

      request_update_constraint[i] = false;

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
                                 << request_update_constraint_tolerance[axis] << ")");
  request_update_constraint[axis] = true;
}
void InverseKinematic::requestUpdateAxisConstraints(int axis, double tolerance)
{
  ROS_WARN_STREAM("Update axis " << axis << " constraints with new tolerance of " << tolerance << "");
  request_update_constraint[axis] = true;
  request_update_constraint_tolerance[axis] = tolerance;
}
}
