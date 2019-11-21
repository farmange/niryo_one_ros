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

namespace space_control
{
InverseKinematic::InverseKinematic(const int joint_number)
  : joint_number_(joint_number)
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
  , position_ctrl_frame_(ControlFrame::World)
  , orientation_ctrl_frame_(ControlFrame::Tool)
{
  ROS_DEBUG_STREAM("InverseKinematic constructor");

  // TODO QUAT improve that !
  space_dimension_ = 7;

  std::vector<double> alpha_weight_vec;
  std::vector<double> beta_weight_vec;

  ros::param::get("~alpha_weight", alpha_weight_vec);
  ros::param::get("~beta_weight", beta_weight_vec);

  setAlphaWeight_(alpha_weight_vec);
  setBetaWeight_(beta_weight_vec);

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

void InverseKinematic::setPositionControlFrame(const ControlFrame frame)
{
  ROS_WARN("setPositionControlFrame = %d", frame);

  position_ctrl_frame_ = frame;
}

void InverseKinematic::setOrientationControlFrame(const ControlFrame frame)
{
  ROS_WARN("setOrientationControlFrame = %d", frame);

  orientation_ctrl_frame_ = frame;
}

const InverseKinematic::ControlFrame& InverseKinematic::getPositionControlFrame() const
{
  return position_ctrl_frame_;
}

const InverseKinematic::ControlFrame& InverseKinematic::getOrientationControlFrame() const
{
  return orientation_ctrl_frame_;
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
}

void InverseKinematic::resolveInverseKinematic(JointVelocity& dq_computed, const SpaceVelocity& dx_desired)
{
  /*
   * Setup IK inputs according to selected control frame
   * Note : Depending on position_ctrl_frame_ and orientation_ctrl_frame_, the dx_desired parameter could be interpreted
   * as tool frame or world frame setpoint
   */
  SpaceVelocity dx_desired_in_frame;

  // position_ctrl_frame_ = ControlFrame::World;
  // orientation_ctrl_frame_ = ControlFrame::World;
  // TODO add function to setup the frame

  /* Compute the desired linear velocity according to the selected control frame :
   *       p0 = R_0to1 * p1
   * with :
   *     - p0 : the linear velocity in world frame
   *     - p1 : the linear velocity in tool frame
   *     - R_0to1 : the rotation matrix of the tool link in the world frame
   *
   * Note :
   *       r_dot = 1/2 * r_c * omega
   */
  // TODO need to add documentation here and reread the orientation part to
  // TODO refactor variable name
  Eigen::Matrix3d R_0to1 = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d R_0to1_transpose = Eigen::Matrix3d::Identity();

  if (position_ctrl_frame_ == ControlFrame::World)
  {
    R_0to1_transpose = Eigen::Matrix3d::Identity();
  }
  else if (position_ctrl_frame_ == ControlFrame::Tool)
  {
    R_0to1 = x_current_.getOrientation().toRotationMatrix();
    R_0to1_transpose = R_0to1.transpose();
  }
  else
  {
    ROS_ERROR("This control frame is not already handle !");
  }
  dx_desired_in_frame.setPosition(R_0to1 * dx_desired.getPosition());

  /* Compute the desired andular velocity according to the selected control frame into a quaternion velocity
   * using the formula :
   *       r_dot = 1/2 * omega * r_c
   * with :
   *     - r_dot : the orientation velocity in quaternion representation
   *     - r_c : the current orientation in quaternion representation
   *     - omega : the angular velocity in rad/s
   *
   * Warning : "omega * r_c" is quaternion product !
   *
   * Note : velocity in tool frame can be written as :
   *       r_dot = 1/2 * r_c * omega
   */
  /* The omega vector (store in dx_desired) can be consider as a quaternion with a scalar part equal to zero */
  Eigen::Quaterniond r_half_omega(0.0, 0.5 * dx_desired.orientation.x(), 0.5 * dx_desired.orientation.y(),
                                  0.5 * dx_desired.orientation.z());
  Eigen::Quaterniond r_dot;
  if (orientation_ctrl_frame_ == ControlFrame::World)
  {
    r_dot = r_half_omega * x_current_.getOrientation();
  }
  else if (orientation_ctrl_frame_ == ControlFrame::Tool)
  {
    r_dot = x_current_.getOrientation() * r_half_omega;
  }
  else
  {
    ROS_ERROR("This control frame is not already handle !");
  }
  dx_desired_in_frame.setOrientation(r_dot);

  /*********************************************************/

  /* Set kinemtic state of the robot to the current joint positions */
  kinematic_state_->setVariablePositions(q_current_);
  kinematic_state_->updateLinkTransforms();

  /* Get jacobian */
  Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
  Eigen::MatrixXd jacobian;
  getJacobian_(kinematic_state_, joint_model_group_, kinematic_state_->getLinkModel(end_effector_link_),
               reference_point_position, jacobian, true);

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
  MatrixXd hessian = (jacobian.transpose() * alpha_weight_ * jacobian) + beta_weight_;

  /* Gradient vector computation */
  VectorXd g = (-jacobian.transpose() * alpha_weight_ * dx_desired_in_frame.getRawVector());

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
  A.block(6, 0, 3, 6) = R_0to1_transpose * jacobian.topLeftCorner(3, 6) * sampling_period_;

  const double eps_pos = 0.001;
  const double eps_orientation = 0.001;
  const double inf = 10.0;

  if (qp_init_required_)
  {
    for (int i = 0; i < 3; i++)
    {
      x_min_limit[i] = x_current_[i] - eps_pos;
      x_max_limit[i] = x_current_[i] + eps_pos;
      flag_save[i] = true;
      eps_inf_pos[i] = eps_pos;
    }

    pos_snap = x_current_.getPosition();
    r_snap = x_current_.getOrientation();
    r_snap_cong = r_snap.conjugate();
  }

  for (int i = 0; i < 3; i++)
  {
    if (dx_desired[i] != 0)
    {
      flag_save[i] = true;
      // x_min_limit[i] = x_current_[i] - inf;
      // x_max_limit[i] = x_current_[i] + inf;
      eps_inf_pos[i] = inf;
    }
    else
    {
      if (flag_save[i] == true)
      {
        flag_save[i] = false;
        pos_snap = x_current_.getPosition();
        // x_min_limit[i] = x_current_[i] - eps_pos;
        // x_max_limit[i] = x_current_[i] + eps_pos;
        eps_inf_pos[i] = eps_pos;
      }
    }
  }

  Eigen::Vector3d lim_pos_vec = R_0to1_transpose * (pos_snap - x_current_.getPosition());
  for (int i = 0; i < 3; i++)
  {
    x_min_limit[i] = lim_pos_vec(i) - eps_inf_pos[i];
    x_max_limit[i] = lim_pos_vec(i) + eps_inf_pos[i];
  }

  /* Define the quaternion conjugate matrix as :
   * conjugate_mat = [1, 0, 0, 0]
   *                 [0,-1, 0, 0]
   *                 [0, 0,-1, 0]
   *                 [0, 0, 0,-1]
   */
  Eigen::Matrix4d conjugate_mat;
  conjugate_mat << 1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1;

  for (int i = 0; i < 3; i++)
  {
    if (dx_desired[4 + i] != 0.0)
    {
      flag_orient_save[i] = true;
      x_min_limit[4 + i] = -inf;
      x_max_limit[4 + i] = inf;
    }
    else
    {
      if (flag_orient_save[i] == true)
      {
        flag_orient_save[i] = false;
        r_snap = x_current_.getOrientation();
        r_snap_cong = r_snap.conjugate();
      }

      if (orientation_ctrl_frame_ == ControlFrame::World)
      {
        Rs_cong = Rx(r_snap) * conjugate_mat;
      }
      else if (orientation_ctrl_frame_ == ControlFrame::Tool)
      {
        Rs_cong = xR(r_snap) * conjugate_mat;
      }

      Eigen::Vector4d Rsrc = Rs_cong * x_current_.getOrientation().toVector();
      double QdQc0 = Rsrc(1 + i);

      x_min_limit[4 + i] = -eps_orientation - QdQc0;
      x_max_limit[4 + i] = +eps_orientation - QdQc0;
    }
  }

  /* constraint of quaternion part */
  A.bottomLeftCorner(3, 6) = (Rs_cong * jacobian.bottomLeftCorner(4, 6) * sampling_period_).bottomLeftCorner(3, 6);

  // TODO add function to handle constaints beautifuly
  double lbA[] = { /* Joints min hard limits constraints */
                   (q_lower_limit_[0] - q_current_[0]),
                   (q_lower_limit_[1] - q_current_[1]),
                   (q_lower_limit_[2] - q_current_[2]),
                   (q_lower_limit_[3] - q_current_[3]),
                   (q_lower_limit_[4] - q_current_[4]),
                   (q_lower_limit_[5] - q_current_[5]),
                   x_min_limit[0],
                   x_min_limit[1],
                   x_min_limit[2],
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
                   x_max_limit[0],
                   x_max_limit[1],
                   x_max_limit[2],
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
    // options.enableInertiaCorrection = qpOASES::BT_TRUE;
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

  // enableInertiaCorrection ? ?

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

    // /*********** DEBUG **************/
    // Eigen::Matrix<double, 6, 1> dq_eigen;
    // Eigen::Matrix<double, 4, 1> min_l;
    // Eigen::Matrix<double, 4, 1> max_l;
    // Eigen::Matrix<double, 12, 1> lbA_eigen;
    // Eigen::Matrix<double, 12, 1> ubA_eigen;
    // for (int i = 0; i < 6; i++)
    // {
    //   dq_eigen(i) = dq_computed[i];
    // }
    // Eigen::Matrix<double, 12, 1> Aq = A * dq_eigen;
    // for (int i = 0; i < 12; i++)
    // {
    //   if (Aq(i) > ubA[i] || Aq(i) < lbA[i])
    //   {
    //     ROS_ERROR("%d : %5f \t < %5f \t < %5f", i, lbA[i], Aq(i), ubA[i]);
    //   }
    //   else
    //   {
    //     ROS_DEBUG("%d : %5f \t < %5f \t < %5f", i, lbA[i], Aq(i), ubA[i]);
    //   }
    // };
    // /********************************/
  }
  else
  {
    ROS_ERROR("qpOASES : Failed with code : %d !", qp_return);
    // /*********** DEBUG **************/
    // Eigen::Matrix<double, 6, 1> dq_eigen;
    // Eigen::Matrix<double, 4, 1> min_l;
    // Eigen::Matrix<double, 4, 1> max_l;
    // Eigen::Matrix<double, 12, 1> lbA_eigen;
    // Eigen::Matrix<double, 12, 1> ubA_eigen;
    // for (int i = 0; i < 6; i++)
    // {
    //   dq_eigen(i) = dq_computed[i];
    // }
    // Eigen::Matrix<double, 12, 1> Aq = A * dq_eigen;
    // for (int i = 0; i < 12; i++)
    // {
    //   if (Aq(i) > ubA[i] || Aq(i) < lbA[i])
    //   {
    //     ROS_ERROR("%d : %5f \t < %5f \t < %5f", i, lbA[i], Aq(i), ubA[i]);
    //   }
    //   else
    //   {
    //     ROS_DEBUG("%d : %5f \t < %5f \t < %5f", i, lbA[i], Aq(i), ubA[i]);
    //   }
    // };
    // /********************************/

    dq_computed[0] = 0.0;
    dq_computed[1] = 0.0;
    dq_computed[2] = 0.0;
    dq_computed[3] = 0.0;
    dq_computed[4] = 0.0;
    dq_computed[5] = 0.0;
  }

  if (qp_return != qpOASES::SUCCESSFUL_RETURN && qp_return != qpOASES::RET_MAX_NWSR_REACHED)
  {
    reset();
    exit(0);  // TODO improve error handling. Crash of the application is neither safe nor beautiful
  }
}

bool InverseKinematic::getJacobian_(const robot_state::RobotStatePtr kinematic_state,
                                    const robot_state::JointModelGroup* group, const robot_state::LinkModel* link,
                                    const Eigen::Vector3d& reference_point_position, Eigen::MatrixXd& jacobian,
                                    bool use_quaternion_representation)
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

  const robot_state::LinkModel* tool_link = kinematic_state_->getLinkModel("tool_link");
  Eigen::Affine3d reference_transform_2 =
      tool_link ? kinematic_state->getGlobalLinkTransform(tool_link).inverse() : Eigen::Affine3d::Identity();
  Eigen::Affine3d link_transform_2 = reference_transform_2 * kinematic_state->getGlobalLinkTransform(tool_link);

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
    // Eigen::MatrixXd quaternion_update_matrix(4, 3);

    /* d/dt ( [w] ) = 1/2 * [ -x -y -z ]  * [ omega_1 ]
     *        [x]           [  w  z -y ]    [ omega_2 ]
     *        [y]           [ -z  w  x ]    [ omega_3 ]
     *        [z]           [  y -x  w ]
     */
    // quaternion_update_matrix << -x, -y, -z, w, z, -y, -z, w, x, y, -x, w;

    // Eigen::Vector4d omega;
    // omega(0) = 0.0;
    // omega.bottomLeftCorner(3, 1) =
    jacobian.block(3, 0, 4, columns) = 0.5 * xR(conv_quat).block(0, 1, 4, 3) * jacobian.block(3, 0, 3, columns);
  }
  return true;
}

/* Quaternion post-product matrix */
Eigen::Matrix4d InverseKinematic::xR(Eigen::Quaterniond& quat)
{
  double w = quat.w(), x = quat.x(), y = quat.y(), z = quat.z();
  Eigen::Matrix4d ret_mat;
  ret_mat << w, -x, -y, -z, x, w, z, -y, y, -z, w, x, z, y, -x, w;
  return ret_mat;
}

/* Quaternion pre-product matrix */
Eigen::Matrix4d InverseKinematic::Rx(Eigen::Quaterniond& quat)
{
  double w = quat.w(), x = quat.x(), y = quat.y(), z = quat.z();
  Eigen::Matrix4d ret_mat;
  ret_mat << w, -x, -y, -z, x, w, -z, y, y, z, w, -x, z, -y, x, w;
  return ret_mat;
}
}
