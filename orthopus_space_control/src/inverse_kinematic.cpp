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
#include <math.h>
#include <tf/tf.h>

namespace space_control
{
InverseKinematic::InverseKinematic(const int joint_number, const bool use_quaternion)
  : joint_number_(joint_number)
  , use_quaternion_(use_quaternion)
  , q_current_(joint_number)
  , q_upper_limit_(joint_number)
  , q_lower_limit_(joint_number)
  , dq_upper_limit_(joint_number)
  , dq_lower_limit_(joint_number)
  , x_current_()
  // , x_min_limit_()
  // , x_max_limit_()
  , sampling_period_(0)
  , qp_init_required_(true)
  , jacobian_init_flag_(true)
{
  ROS_DEBUG_STREAM("InverseKinematic constructor");

  // TODO QUAT improve that !
  space_dimension_ = 7;

  std::vector<double> alpha_weight_vec;
  std::vector<double> beta_weight_vec;
  std::vector<double> delta_weight_vec;
  std::vector<double> epsilon_weight_vec;
  ros::param::get("~alpha_weight", alpha_weight_vec);
  ros::param::get("~beta_weight", beta_weight_vec);
  ros::param::get("~gamma_weight", gamma_weight_vec);
  ros::param::get("~delta_weight", delta_weight_vec);
  ros::param::get("~epsilon_weight", epsilon_weight_vec);

  setAlphaWeight_(alpha_weight_vec);
  setBetaWeight_(beta_weight_vec);
  setDeltaWeight_(delta_weight_vec);
  setEpsilonWeight_(epsilon_weight_vec);

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
  setDqBounds_(limit);

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  kinematic_model_ = robot_model_loader.getModel();
  kinematic_state_ = std::make_shared<robot_state::RobotState>(kinematic_model_);
  kinematic_state_->setToDefaultValues();
  joint_model_group_ = kinematic_model_->getJointModelGroup("arm");

  // TODO Comment that part
  // for (int i = 0; i < 6; i++)
  // {
  //   x_min_limit_[i] = 0.0;
  //   x_max_limit_[i] = 0.0;
  // }
}

void InverseKinematic::init(const std::string end_effector_link, const double sampling_period)
{
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
  ROS_DEBUG_STREAM("Set beta weight to : \n" << beta_weight_ << "\n");
}

void InverseKinematic::setDeltaWeight_(const std::vector<double>& delta_weight)
{
  // TODO
  delta_weight_ = MatrixXd::Identity(3, 3);
  for (int i = 0; i < 3; i++)
  {
    delta_weight_(i, i) = delta_weight[i];
  }
  ROS_DEBUG_STREAM("Set delta weight to : \n" << delta_weight_ << "\n");
}

void InverseKinematic::setEpsilonWeight_(const std::vector<double>& epsilon_weight)
{
  // TODO
  epsilon_weight_ = MatrixXd::Identity(4, 4);
  for (int i = 0; i < 4; i++)
  {
    epsilon_weight_(i, i) = epsilon_weight[i];
  }
  ROS_DEBUG_STREAM("Set epsilon weight to : \n" << epsilon_weight_ << "\n");
}

void InverseKinematic::setGammaWeight_(const std::vector<double>& gamma_weight, const Eigen::Quaterniond& q)
{
  // Minimize cartesian velocity : dx
  gamma_weight_ = MatrixXd::Identity(space_dimension_, space_dimension_);
  for (int i = 0; i < space_dimension_; i++)
  {
    gamma_weight_(i, i) = gamma_weight[i];
  }
  double w = q.w(), x = q.x(), y = q.y(), z = q.z();
  Eigen::MatrixXd quat_transfert_matrix(4, 3);
  quat_transfert_matrix << -x, -y, -z, w, -z, y, z, w, -x, -y, x, w;
  Eigen::MatrixXd quat_transfert_matrix_pseudo_inv(3, 4);
  quat_transfert_matrix_pseudo_inv = 4 * quat_transfert_matrix.transpose() / (w * w + x * x + y * y + z * z);
  gamma_weight_.bottomRightCorner(4, 4) = quat_transfert_matrix_pseudo_inv.transpose() *
                                          gamma_weight_.bottomRightCorner(3, 3) * quat_transfert_matrix_pseudo_inv;

  ROS_DEBUG_STREAM("Set gamma weight to : \n" << gamma_weight_ << "\n");
}

void InverseKinematic::setQCurrent(const JointPosition& q_current)
{
  q_current_ = q_current;
}

void InverseKinematic::setOmega(const SpaceVelocity& omega)
{
  dx_omega_ = omega;
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

  // TODO QUAT improve that to allow copy
  // x_current_eigen_ = x_current.getRawVector();
}

void InverseKinematic::setDqBounds_(const JointVelocity& dq_bound)
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
  jacobian_init_flag_ = true;
  for (int i = 0; i < space_dimension_; i++)
  {
    reset_axis[i] = true;
    reset_axis2[i] = true;
  }
}

void InverseKinematic::resolveInverseKinematic(JointVelocity& dq_computed, const SpaceVelocity& dx_desired)
{
  if (qp_init_required_)
  {
    x_des = x_current_eigen_;  // HACK : store the initial space position
    Eigen::Quaterniond quat_current(x_current_[3], x_current_[4], x_current_[5], x_current_[6]);
    // quat_des = quat_current;
  }
  /* Store desired velocity in eigen vector */
  VectorXd dx_desired_eigen = VectorXd(space_dimension_);
  for (int i = 0; i < space_dimension_; i++)
  {
    dx_desired_eigen(i) = dx_desired[i];
  }

  // Eigen::Quaterniond quat_cool(x_current_.getQw(), x_current_.getQx(), x_current_.getQy(), x_current_.getQz());
  // setGammaWeight_(gamma_weight_vec, quat_cool);

  ////////////////////////////// position XYZ //////////////////////////////
  for (int i = 0; i < 3; i++)
  {
    if (dx_desired[i] != 0.0)
    {
      reset_axis[i] = true;
    }
    else
    {
      if (reset_axis[i] == true)
      {
        x_des[i] = x_current_[i];
        reset_axis[i] = false;
      }
    }
    // compute desired position according to taylor development
    x_des[i] = x_des[i] + dx_desired[i] * sampling_period_;
  }
  Eigen::Vector3d x_current_eigen_p_;
  x_current_eigen_p_(0) = x_current_eigen_(0);
  x_current_eigen_p_(1) = x_current_eigen_(1);
  x_current_eigen_p_(2) = x_current_eigen_(2);
  Eigen::Vector3d x_des_p_;
  x_des_p_(0) = x_des(0);
  x_des_p_(1) = x_des(1);
  x_des_p_(2) = x_des(2);

  //////////////////////////// orientation Qwxyz ///////////////////////////
  Eigen::Quaterniond omega_quat_tool(0, dx_omega_[4], dx_omega_[5], dx_omega_[6]);
  ROS_DEBUG("omega_quat_tool = %5f, %5f, %5f; %5f ", omega_quat_tool.w(), omega_quat_tool.x(), omega_quat_tool.y(),
            omega_quat_tool.z());
  ROS_DEBUG("|omega_quat_tool| = %5f ", omega_quat_tool.norm());

  Eigen::Quaterniond quat_current(x_current_[3], x_current_[4], x_current_[5], x_current_[6]);
  Eigen::Vector4d quat_current_vec;
  quat_current_vec(0) = quat_current.w();
  quat_current_vec(1) = quat_current.x();
  quat_current_vec(2) = quat_current.y();
  quat_current_vec(3) = quat_current.z();
  ROS_DEBUG("quat_current = %5f, %5f, %5f; %5f ", quat_current.w(), quat_current.x(), quat_current.y(),
            quat_current.z());
  ROS_DEBUG("|quat_current| = %5f ", quat_current.norm());

  // Eigen::Quaterniond omega_quat_world = quat_current * omega_quat_tool * quat_current.conjugate();
  Eigen::Quaterniond omega_quat_world = omega_quat_tool;

  ROS_DEBUG("|omega_quat_world| = %5f ", omega_quat_world.norm());

  double delta_quat_raw[4];
  double temp = 0.5 * omega_quat_world.norm() * sampling_period_;
  if (omega_quat_tool.norm() > 0)
  {
    delta_quat_raw[0] = cos(temp);
    delta_quat_raw[1] = (omega_quat_world.x() / omega_quat_world.norm()) * sin(temp);
    delta_quat_raw[2] = (omega_quat_world.y() / omega_quat_world.norm()) * sin(temp);
    delta_quat_raw[3] = (omega_quat_world.z() / omega_quat_world.norm()) * sin(temp);
  }
  else
  {
    delta_quat_raw[0] = 1.0;
    delta_quat_raw[1] = 0.0;
    delta_quat_raw[2] = 0.0;
    delta_quat_raw[3] = 0.0;
  }
  ROS_DEBUG("delta_quat_raw = %5f, %5f, %5f; %5f ", delta_quat_raw[0], delta_quat_raw[1], delta_quat_raw[2],
            delta_quat_raw[3]);
  Eigen::Quaterniond delta_quat(delta_quat_raw[0], delta_quat_raw[1], delta_quat_raw[2], delta_quat_raw[3]);
  ROS_DEBUG("delta_quat = %5f, %5f, %5f; %5f ", delta_quat.w(), delta_quat.x(), delta_quat.y(), delta_quat.z());

  // quat_des = delta_quat * quat_des;
  // ROS_DEBUG("quat_des = %5f, %5f, %5f; %5f ", quat_des.w(), quat_des.x(), quat_des.y(), quat_des.z());
  // ROS_DEBUG("|quat_des| = %5f ", quat_des.norm());

  // Eigen::Quaterniond quat_error = quat_des * quat_current.conjugate();
  // ROS_DEBUG("quat_error = %5f, %5f, %5f; %5f ", quat_error.w(), quat_error.x(), quat_error.y(), quat_error.z());
  // ROS_DEBUG("|quat_error| = %5f ", quat_error.norm());

  // double w = quat_des.w(), x = quat_des.x(), y = quat_des.y(), z = quat_des.z();
  // Eigen::MatrixXd quat_des_transfert_matrix(4, 4);
  // quat_des_transfert_matrix << w, x, y, z, -x, w, -z, y, -y, z, w, -x, -z, -y, x, w;
  // ROS_ERROR_STREAM("quat_des_transfert_matrix = \n" << quat_des_transfert_matrix);

  /* Set kinemtic state of the robot to the current joint positions */
  kinematic_state_->setVariablePositions(q_current_);
  kinematic_state_->updateLinkTransforms();

  /* Get jacobian from kinematic state (Moveit) */
  Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
  Eigen::MatrixXd jacobian;

  getJacobian_(kinematic_state_, joint_model_group_, kinematic_state_->getLinkModel(end_effector_link_),
               reference_point_position, jacobian, true, false);

  /**********************************/
  Eigen::MatrixXd jacob_temp;
  getJacobian_(kinematic_state_, joint_model_group_, kinematic_state_->getLinkModel(end_effector_link_),
               reference_point_position, jacob_temp, false, false);
  Eigen::MatrixXd Jw_R0(3, 6);
  Jw_R0 = jacob_temp.bottomLeftCorner(3, 6);

  Eigen::MatrixXd Rc_R0(4, 3);
  Eigen::MatrixXd Rc_R1(4, 3);
  double rc_w = quat_current.w(), rc_x = quat_current.x(), rc_y = quat_current.y(), rc_z = quat_current.z();
  Rc_R0 << -rc_x, -rc_y, -rc_z, rc_w, rc_z, -rc_y, -rc_z, rc_w, rc_x, rc_y, -rc_x, rc_w;
  Rc_R1 << -rc_x, -rc_y, -rc_z, rc_w, -rc_z, rc_y, rc_z, rc_w, -rc_x, -rc_y, rc_x, rc_w;

  Eigen::MatrixXd Jw_R1 = Rc_R1.transpose() * Rc_R0 * Jw_R0;
  // ROS_ERROR_STREAM("Rc_R0 = \n" << Rc_R0);
  // ROS_ERROR_STREAM("Rc_R1 = \n" << Rc_R1);
  // ROS_ERROR_STREAM("Jw_R0 = \n" << Jw_R0);
  // ROS_ERROR_STREAM("Jw_R1 = \n" << Jw_R1);

  Eigen::MatrixXd Jr_R0 = 0.5 * Rc_R0 * Jw_R0;
  Eigen::MatrixXd Jr_R00 = 0.5 * Rc_R1 * Jw_R1;
  Eigen::MatrixXd Jr_R1 = 0.5 * Rc_R0 * Jw_R1;
  // ROS_ERROR_STREAM("jacobian (world) = \n" << jacobian);
  // jacobian.bottomLeftCorner(4, 6) = Jr_R1;
  // ROS_ERROR_STREAM("Jr_R1 = \n" << Jr_R1);

  // ROS_ERROR_STREAM("Jr_R0 = \n" << Jr_R0);
  // ROS_ERROR_STREAM("Jr_R00 = \n" << Jr_R00);
  // ROS_ERROR_STREAM("=======================");
  //  ROS_ERROR_STREAM("jacobian_w = \n" << jacobian.bottomLeftCorner(4, 6));

  /**********************************/
  Eigen::MatrixXd jacobian_p = jacobian.topLeftCorner(3, 6);
  Eigen::MatrixXd jacobian_q = jacobian.bottomLeftCorner(4, 6);

  // ROS_ERROR_STREAM("jacobian = \n" << jacobian);
  // ROS_ERROR_STREAM("jacobian_p = \n" << jacobian_p);
  // ROS_ERROR_STREAM("jacobian_q = \n" << jacobian_q);
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
  MatrixXd hessian = (jacobian.transpose() * alpha_weight_ * jacobian) + beta_weight_ /* +
                     (jacobian_p.transpose() * sampling_period_ * delta_weight_ * sampling_period_ * jacobian_p) +
                     (jacobian_q.transpose() * quat_des_transfert_matrix.transpose() * epsilon_weight_ *
                      quat_des_transfert_matrix * jacobian_q * sampling_period_ * sampling_period_) */;

  /* Gradient vector computation */
  VectorXd g = (-jacobian.transpose() * alpha_weight_ * dx_desired_eigen) /* +
      (jacobian_p.transpose() * sampling_period_ * delta_weight_ * x_current_eigen_p_) -
      (jacobian_p.transpose() * sampling_period_ * delta_weight_ * x_des_p_) +
      (jacobian_q.transpose() * sampling_period_ * epsilon_weight_ * quat_des_transfert_matrix * quat_current_vec) */;

  // for (int i = 0; i < space_dimension_; i++)
  // {
  //   ROS_ERROR("Delta between x_current and x_des for axis %d : %2f mm", i, (x_current_eigen_[i] - x_des[i]) *
  //   1000.0);
  // }

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
  A.block(6, 0, 3, 6) = jacobian.topLeftCorner(3, 6) * sampling_period_;

  const double eps_pos = 0.001;
  const double eps_orientation = 0.001;
  const double inf = 1.0;
  Eigen::Quaterniond r_c_cong = quat_current.conjugate();

  Eigen::MatrixXd snap_transfert_matrix(4, 4);
  double ws = quat_current.w(), xs = quat_current.x(), ys = quat_current.y(), zs = quat_current.z();
  snap_transfert_matrix << ws, xs, ys, zs, -xs, ws, -zs, ys, -ys, zs, ws, -xs, -zs, -ys, xs, ws;
  if (qp_init_required_)
  {
    for (int i = 0; i < 3; i++)
    {
      x_min_limit[i] = x_current_[i] - eps_pos;
      x_max_limit[i] = x_current_[i] + eps_pos;
      flag_save[i] = true;
    }

    Qsnap[0] = snap_transfert_matrix.block(1, 0, 1, 4).transpose();
    Qsnap[1] = snap_transfert_matrix.block(2, 0, 1, 4).transpose();
    Qsnap[2] = snap_transfert_matrix.block(3, 0, 1, 4).transpose();

    r_snap = quat_current;
    r_snap_cong = r_snap.conjugate();
    Rs_cong = xR(quat_current) * Rx(r_c_cong) * xR(r_snap_cong);
  }

  for (int i = 0; i < 3; i++)
  {
    if (dx_desired[i] != 0)
    {
      flag_save[i] = true;
      x_min_limit[i] = x_current_[i] - inf;
      x_max_limit[i] = x_current_[i] + inf;
    }
    else
    {
      if (flag_save[i] == true)
      {
        flag_save[i] = false;
        x_min_limit[i] = x_current_[i] - eps_pos;
        x_max_limit[i] = x_current_[i] + eps_pos;
      }
    }
  }

  x_min_limit[3] = -1.1;
  x_max_limit[3] = +1.1;

  for (int i = 0; i < 3; i++)
  {
    if (dx_omega_[4 + i] != 0.0)
    {
      flag_orient_save[i] = true;
      x_min_limit[4 + i] = -10.0;
      x_max_limit[4 + i] = +10.0;
    }
    else
    {
      if (flag_orient_save[i] == true)
      {
        flag_orient_save[i] = false;
        double ws = quat_current.w(), xs = quat_current.x(), ys = quat_current.y(), zs = quat_current.z();
        r_snap.w() = ws;
        r_snap.x() = xs;
        r_snap.y() = ys;
        r_snap.z() = zs;
        r_snap_cong = r_snap.conjugate();
      }
      // Rs_cong = xR(quat_current) * Rx(r_c_cong) * xR(r_snap_cong);
      Rs_cong = xR(r_snap_cong);
      Eigen::Vector4d Rsrc = Rs_cong * quat_current_vec;
      double QdQc0 = Rsrc(1 + i);
      x_min_limit[4 + i] = -eps_orientation - QdQc0;
      x_max_limit[4 + i] = +eps_orientation - QdQc0;
    }
  }

  /* constraint of quaternion part */
  A.bottomLeftCorner(3, 6) = (Rs_cong * jacobian_q.bottomLeftCorner(4, 6) * sampling_period_).bottomLeftCorner(3, 6);
  ROS_WARN_STREAM("A = \n" << A);
  ROS_WARN_STREAM("x_min_limit = \n" << SpacePosition(x_min_limit));
  ROS_WARN_STREAM("x_current_ = \n" << x_current_);
  ROS_WARN_STREAM("x_max_limit = \n" << SpacePosition(x_max_limit));

  // TODO add function to handle constaints beautifuly
  double lbA[] = { /* Joints min hard limits constraints */
                   (q_lower_limit_[0] - q_current_[0]),
                   (q_lower_limit_[1] - q_current_[1]),
                   (q_lower_limit_[2] - q_current_[2]),
                   (q_lower_limit_[3] - q_current_[3]),
                   (q_lower_limit_[4] - q_current_[4]),
                   (q_lower_limit_[5] - q_current_[5]),
                   (x_min_limit[0] - x_current_eigen_(0, 0)),
                   (x_min_limit[1] - x_current_eigen_(1, 0)),
                   (x_min_limit[2] - x_current_eigen_(2, 0)),
                   x_min_limit[4],
                   x_min_limit[5],
                   x_min_limit[6]
  };

  double ubA[] = { /* Joints max hard limits constraints */
                   (q_upper_limit_[0] - q_current_[0]),
                   (q_upper_limit_[1] - q_current_[1]),
                   (q_upper_limit_[2] - q_current_[2]),
                   (q_upper_limit_[3] - q_current_[3]),
                   (q_upper_limit_[4] - q_current_[4]),
                   (q_upper_limit_[5] - q_current_[5]),
                   (x_max_limit[0] - x_current_eigen_(0, 0)),
                   (x_max_limit[1] - x_current_eigen_(1, 0)),
                   (x_max_limit[2] - x_current_eigen_(2, 0)),
                   x_max_limit[4],
                   x_max_limit[5],
                   x_max_limit[6]
  };

  /* Solve QP */
  qpOASES::real_t xOpt[6];
  qpOASES::int_t nWSR = 10;
  int qp_return = 0;
  if (qp_init_required_)
  {
    /* Initialize QP solver */
    QP_ = new qpOASES::SQProblem(6, 12);  // HACK : ONLY JOINT VELOCITY LIMIT
    qpOASES::Options options;
    options.setToReliable();
    // options.printLevel = qpOASES::PL_NONE;
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

    /*********** DEBUG **************/
    Eigen::Matrix<double, 6, 1> dq_eigen;
    for (int i = 0; i < 6; i++)
    {
      dq_eigen(i) = dq_computed[i];
    };
    ROS_DEBUG_STREAM("result = \n" << A.bottomLeftCorner(3, 6) * dq_eigen);
    /********************************/
  }
  else
  {
    ROS_ERROR("qpOASES : Failed with code : %d !", qp_return);
    /*********** DEBUG **************/
    Eigen::Matrix<double, 6, 1> dq_eigen;
    Eigen::Matrix<double, 4, 1> eps_eigen;
    Eigen::Matrix<double, 4, 1> min_l;
    Eigen::Matrix<double, 4, 1> max_l;
    for (int i = 0; i < 6; i++)
    {
      dq_eigen(i) = dq_computed[i];
    };
    for (int i = 0; i < 4; i++)
    {
      eps_eigen(i) = 0.001;
      min_l(i) = x_min_limit[3 + i];
      max_l(i) = x_max_limit[3 + i];
    };
    // ROS_DEBUG_STREAM("Rc_R1 = \n" << Rc_R1);
    // ROS_DEBUG_STREAM("(snap_transfert_matrix * Rc_R1) = \n" << (snap_transfert_matrix * Rc_R1));

    // ROS_DEBUG_STREAM("min = \n" << -eps_eigen - (snap_transfert_matrix * Rc_R1));
    ROS_DEBUG_STREAM("x_min_limit = \n" << min_l);
    ROS_DEBUG_STREAM("snap_transfert_matrix* Jr_R1* sampling_period_* dq_eigen = \n"
                     << snap_transfert_matrix * Jr_R1 * sampling_period_ * dq_eigen);
    // ROS_DEBUG_STREAM("max = \n" << eps_eigen - (snap_transfert_matrix * Rc_R1));
    ROS_DEBUG_STREAM("x_max_limit = \n" << max_l);
    /********************************/

    dq_computed[0] = 0.0;
    dq_computed[1] = 0.0;
    dq_computed[2] = 0.0;
    dq_computed[3] = 0.0;
    dq_computed[4] = 0.0;
    dq_computed[5] = 0.0;

    // std::cout << "Press Enter to Continue";
    // std::cin.ignore();
  }

  if (qp_return != qpOASES::SUCCESSFUL_RETURN && qp_return != qpOASES::RET_MAX_NWSR_REACHED)
  {
    exit(0);  // TODO improve error handling. Crash of the application is neither safe nor beautiful
  }
}

bool InverseKinematic::getJacobian_(const robot_state::RobotStatePtr kinematic_state,
                                    const robot_state::JointModelGroup* group, const robot_state::LinkModel* link,
                                    const Eigen::Vector3d& reference_point_position, Eigen::MatrixXd& jacobian,
                                    bool use_quaternion_representation, bool impl)
{
  if (!group->isChain())
  {
    ROS_ERROR("The group '%s' is not a chain. Cannot compute Jacobian.", group->getName().c_str());
    return false;
  }

  if (!group->isLinkUpdated(link->getName()))
  {
    ROS_ERROR("Link name '%s' does not exist in the chain '%s' or is not a child for this chain",
              link->getName().c_str(), group->getName().c_str());
    return false;
  }

  const robot_model::JointModel* root_joint_model = group->getJointModels()[0];
  const robot_model::LinkModel* root_link_model = root_joint_model->getParentLinkModel();
  Eigen::Affine3d reference_transform = root_link_model ?
                                            kinematic_state->getGlobalLinkTransform(root_link_model).inverse() :
                                            Eigen::Affine3d::Identity();
  int rows = use_quaternion_representation ? 7 : 6;
  int columns = group->getVariableCount();
  jacobian = Eigen::MatrixXd::Zero(rows, columns);

  Eigen::Affine3d link_transform = reference_transform * kinematic_state->getGlobalLinkTransform(link);
  Eigen::Vector3d point_transform = link_transform * reference_point_position;

  Eigen::Vector3d joint_axis;
  Eigen::Affine3d joint_transform;

  while (link)
  {
    const robot_model::JointModel* pjm = link->getParentJointModel();
    if (pjm->getVariableCount() > 0)
    {
      unsigned int joint_index = group->getVariableGroupIndex(pjm->getName());
      if (pjm->getType() == robot_model::JointModel::REVOLUTE)
      {
        joint_transform = reference_transform * kinematic_state->getGlobalLinkTransform(link);
        joint_axis = joint_transform.rotation() * static_cast<const robot_model::RevoluteJointModel*>(pjm)->getAxis();
        jacobian.block<3, 1>(0, joint_index) =
            jacobian.block<3, 1>(0, joint_index) + joint_axis.cross(point_transform - joint_transform.translation());
        jacobian.block<3, 1>(3, joint_index) = jacobian.block<3, 1>(3, joint_index) + joint_axis;
      }
      else if (pjm->getType() == robot_model::JointModel::PRISMATIC)
      {
        joint_transform = reference_transform * kinematic_state->getGlobalLinkTransform(link);
        joint_axis = joint_transform * static_cast<const robot_model::PrismaticJointModel*>(pjm)->getAxis();
        jacobian.block<3, 1>(0, joint_index) = jacobian.block<3, 1>(0, joint_index) + joint_axis;
      }
      else if (pjm->getType() == robot_model::JointModel::PLANAR)
      {
        joint_transform = reference_transform * kinematic_state->getGlobalLinkTransform(link);
        joint_axis = joint_transform * Eigen::Vector3d(1.0, 0.0, 0.0);
        jacobian.block<3, 1>(0, joint_index) = jacobian.block<3, 1>(0, joint_index) + joint_axis;
        joint_axis = joint_transform * Eigen::Vector3d(0.0, 1.0, 0.0);
        jacobian.block<3, 1>(0, joint_index + 1) = jacobian.block<3, 1>(0, joint_index + 1) + joint_axis;
        joint_axis = joint_transform * Eigen::Vector3d(0.0, 0.0, 1.0);
        jacobian.block<3, 1>(0, joint_index + 2) = jacobian.block<3, 1>(0, joint_index + 2) +
                                                   joint_axis.cross(point_transform - joint_transform.translation());
        jacobian.block<3, 1>(3, joint_index + 2) = jacobian.block<3, 1>(3, joint_index + 2) + joint_axis;
      }
      else
        ROS_ERROR("Unknown type of joint in Jacobian computation");
    }
    if (pjm == root_joint_model)
      break;
    link = pjm->getParentLinkModel();
  }
  if (use_quaternion_representation)
  {
    /* Convert rotation matrix to quaternion */
    Eigen::Quaterniond conv_quat(link_transform.rotation());
    /* Warning : During the convertion in quaternion, sign could change as there are tow quaternion definitions possible
     * (q and -q) for the same rotation. The following code ensure quaternion continuity between to occurence of this
     * method call
     */
    if (jacobian_init_flag_)
    {
      jacobian_init_flag_ = false;
    }
    else
    {
      /* Detect if a discontinuity happened between new quaternion and the previous one */
      double diff_norm =
          sqrt(pow(conv_quat.w() - jacobian_quat_prev_.w(), 2) + pow(conv_quat.x() - jacobian_quat_prev_.x(), 2) +
               pow(conv_quat.y() - jacobian_quat_prev_.y(), 2) + pow(conv_quat.z() - jacobian_quat_prev_.z(), 2));
      if (diff_norm > 1)
      {
        ROS_DEBUG_NAMED("InverseKinematic", "A discontinuity has been detected during quaternion conversion.");
        /* If discontinuity happened, change sign of the quaternion */
        conv_quat.w() = -conv_quat.w();
        conv_quat.x() = -conv_quat.x();
        conv_quat.y() = -conv_quat.y();
        conv_quat.z() = -conv_quat.z();
      }
      else
      {
        /* Else, do nothing and keep quaternion sign */
      }
    }
    jacobian_quat_prev_ = conv_quat;

    double w = conv_quat.w(), x = conv_quat.x(), y = conv_quat.y(), z = conv_quat.z();
    Eigen::MatrixXd quaternion_update_matrix(4, 3);
    if (impl)
    {
      // vrai pour omega exprimé dans R tool
      // Le quaternion resultant est exprimé dans R world ???
      // d/dt ( [w] ) = 1/2 * [ -x -y -z ]  * [ omega_1 ]
      //        [x]           [  w -z  y ]    [ omega_2 ]
      //        [y]           [  z  w -x ]    [ omega_3 ]
      //        [z]           [ -y  x  w ]
      quaternion_update_matrix << -x, -y, -z, w, -z, y, z, w, -x, -y, x, w;
    }
    else
    {
      // vrai pour omega exprimé dans R world
      // d/dt ( [w] ) = 1/2 * [ -x -y -z ]  * [ omega_1 ]
      //        [x]           [  w  z -y ]    [ omega_2 ]
      //        [y]           [ -z  w  x ]    [ omega_3 ]
      //        [z]           [  y -x  w ]
      quaternion_update_matrix << -x, -y, -z, w, z, -y, -z, w, x, y, -x, w;
    }
    jacobian.block(3, 0, 4, columns) = 0.5 * quaternion_update_matrix * jacobian.block(3, 0, 3, columns);
  }
  return true;
}

Eigen::Matrix4d InverseKinematic::xR(Eigen::Quaterniond& quat)
{
  double w = quat.w(), x = quat.x(), y = quat.y(), z = quat.z();
  Eigen::Matrix4d ret_mat;
  ret_mat << w, -x, -y, -z, x, w, z, -y, y, -z, w, x, z, y, -x, w;
  return ret_mat;
}

Eigen::Matrix4d InverseKinematic::Rx(Eigen::Quaterniond& quat)
{
  double w = quat.w(), x = quat.x(), y = quat.y(), z = quat.z();
  Eigen::Matrix4d ret_mat;
  ret_mat << w, -x, -y, -z, x, w, -z, y, y, z, w, -x, z, -y, x, w;
  return ret_mat;
}
}
