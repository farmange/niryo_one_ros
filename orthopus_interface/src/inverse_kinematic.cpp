// TODO copyright
#include <ros/ros.h>
// #include <tf/transform_listener.h>
// #include "geometry_msgs/TwistStamped.h"
// #include "niryo_one_msgs/SetInt.h"
// #include "std_msgs/Bool.h"
#include "orthopus_interface/inverse_kinematic.h"

// #include <moveit/move_group_interface.h>
// #include <moveit/move_group_interface/move_group_interface.h>

#include <eigen_conversions/eigen_msg.h>

// static const int NUM_SPINNERS = 2;
// static const int JOINT_SUB_QUEUE_LENGTH = 1;
// static const int X_DOT_DES_QUEUE_LENGTH = 1;
// static const double LINEAR_SCALE = 1.0;
// static const double ROTATIONAL_SCALE = 1.0;
// static const double TO_RAD = (3.141589 / 180.0);
//
static const int RATE = 10;
static const double CALC_PERIOD = 1.0 / RATE;
// static const double INIT_DURATION = 1;
//
// int TOOL_ID = 11;  // change this depenidning on the tool mounted

namespace cartesian_controller
{
InverseKinematic::InverseKinematic()
{
  ROS_INFO("InverseKinematic constructor");

  //       ros::ServiceServer service = n_.advertiseService("/niryo_one/joystick_interface/enable",
  //                                                      &JoystickCartesianInterface::joystickEnableCB, this);
  ros::param::get("~alpha_1", alpha_1);
  ros::param::get("~alpha_2", alpha_2);
  ros::param::get("~alpha_3", alpha_3);
  ros::param::get("~alpha_4", alpha_4);
  ros::param::get("~alpha_5", alpha_5);
  ros::param::get("~alpha_6", alpha_6);
  ros::param::get("~beta_1", beta_1);
  ros::param::get("~beta_2", beta_2);
  ros::param::get("~beta_3", beta_3);
  ros::param::get("~beta_4", beta_4);
  ros::param::get("~beta_5", beta_5);
  ros::param::get("~beta_6", beta_6);
  ros::param::get("~gamma_1", gamma_1);
  ros::param::get("~gamma_2", gamma_2);
  ros::param::get("~gamma_3", gamma_3);
  ros::param::get("~gamma_4", gamma_4);
  ros::param::get("~gamma_5", gamma_5);
  ros::param::get("~gamma_6", gamma_6);

  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j1/min", joints_limits_min[0]);
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j1/max", joints_limits_max[0]);
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j2/min", joints_limits_min[1]);
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j2/max", joints_limits_max[1]);
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j3/min", joints_limits_min[2]);
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j3/max", joints_limits_max[2]);
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j4/min", joints_limits_min[3]);
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j4/max", joints_limits_max[3]);
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j5/min", joints_limits_min[4]);
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j5/max", joints_limits_max[4]);
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j6/min", joints_limits_min[5]);
  n_.getParam("/niryo_one/robot_command_validation/joint_limits/j6/max", joints_limits_max[5]);


  x_min = 0.235;
  x_max = 0.24;
  y_min = -0.55;
  y_max = 0.55;
  z_min = -0.05;
  z_max = 0.55;
  r_min = 0.48;
  r_max = 0.52;
  p_min = 0.48;
  p_max = 0.52;
  yaw_min = 0.48;
  yaw_max = 0.52;
  
  // Wait for initial messages
  ROS_INFO("Waiting for first joint msg.");
  ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states");
  ROS_INFO("Received first joint msg.");

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  kinematic_model = robot_model_loader.getModel();
  ROS_DEBUG("Model frame: %s", kinematic_model->getModelFrame().c_str());

  //     robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  kinematic_state = std::make_shared<robot_state::RobotState>(kinematic_model);

  kinematic_state->setToDefaultValues();
  joint_model_group = kinematic_model->getJointModelGroup("arm");

  // Minimize cartesian velocity : dx
  alpha_weight = Matrix6d::Identity(6, 6);
  alpha_weight(0, 0) = alpha_1;
  alpha_weight(1, 1) = alpha_2;
  alpha_weight(2, 2) = alpha_3;
  alpha_weight(3, 3) = alpha_4;
  alpha_weight(4, 4) = alpha_5;
  alpha_weight(5, 5) = alpha_6;
  ROS_DEBUG_STREAM("alpha_weight: \n" << alpha_weight << "\n");

  // Minimize joints velocities : dq
  beta_weight = Matrix6d::Identity(6, 6);
  beta_weight(0, 0) = beta_1;
  beta_weight(1, 1) = beta_2;
  beta_weight(2, 2) = beta_3;
  beta_weight(3, 3) = beta_4;
  beta_weight(4, 4) = beta_5;
  beta_weight(5, 5) = beta_6;
  ROS_DEBUG_STREAM("beta_weight: \n" << beta_weight << "\n");

  // Minimize cartesian position : x
  gamma_weight = Matrix6d::Identity(6, 6);
  gamma_weight(0, 0) = gamma_1;
  gamma_weight(1, 1) = gamma_2;
  gamma_weight(2, 2) = gamma_3;
  gamma_weight(3, 3) = gamma_4;
  gamma_weight(4, 4) = gamma_5;
  gamma_weight(5, 5) = gamma_6;
  ROS_DEBUG_STREAM("gamma_weight: \n" << gamma_weight << "\n");

  // Initialize QP solver
  IK = new qpOASES::SQProblem(6, 12);
  qpOASES::Options options;
  options.printLevel = qpOASES::PL_NONE;
  IK->setOptions(options);
}
void InverseKinematic::ResolveInverseKinematic(double (&joint_position_command)[6], sensor_msgs::JointState &current_joint_state, double (&cartesian_velocity_desired)[6] )
{
  kinematic_state->setVariableValues(current_joint_state);
  
  
// -2.20717e-13     -0.02345     -0.00245  -5.6137e-15      0.00055  1.65443e-30
//      0.02452 -1.19674e-13 -1.25032e-14      0.00055  2.80685e-15 -1.62093e-19
//            0      0.02452      0.02452 -8.36963e-15      0.00237  8.83524e-30
//            0  5.10336e-13  5.10336e-13          0.1  5.10336e-13          0.1
//            0         -0.1         -0.1  1.02067e-12         -0.1  1.02067e-12
//          0.1            0            0 -2.22045e-17            0 -2.22045e-17
         
  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
  std::vector<double> joint_values;
  std::vector<double> joint_values_copy = std::vector<double>(6);
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO("Joint %s: %f (current_joint_state=%f)", joint_names[i].c_str(), joint_values[i], current_joint_state.position[i]);
    joint_values_copy[i] = joint_values[i];
    // joint_values_copy[i] = theta[i];
  }

  const Eigen::Affine3d& end_effector_state = kinematic_state->getGlobalLinkTransform("hand_link");  // TODO Add configuration parameter
  geometry_msgs::Pose current_pose;
  tf::poseEigenToMsg(end_effector_state, current_pose);
  //debug_pub_.publish(current_pose);

  Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
  Eigen::MatrixXd jacobian;
  kinematic_state->getJacobian(joint_model_group,
                               kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                               reference_point_position, jacobian);
  ROS_DEBUG_STREAM("Jacobian: \n" << jacobian << "\n");

  const Vector6d dx_des_vect = Vector6d(cartesian_velocity_desired);
  ROS_DEBUG_STREAM("dx_des_vect: \n" << dx_des_vect << "\n");
  ROS_DEBUG_STREAM("alpha_weight: \n" << alpha_weight << "\n");
  ROS_DEBUG_STREAM("beta_weight: \n" << beta_weight << "\n");
  ROS_DEBUG_STREAM("gamma_weight: \n" << gamma_weight << "\n");

  Matrix6d hessian = (jacobian.transpose() * alpha_weight * jacobian) + beta_weight +
                     (jacobian.transpose() * CALC_PERIOD * gamma_weight * CALC_PERIOD * jacobian);
  Vector6d currentPosition;
  currentPosition(0, 0) = current_pose.position.x;
  currentPosition(1, 0) = current_pose.position.y;
  currentPosition(2, 0) = current_pose.position.z;
  currentPosition(3, 0) = current_pose.orientation.x;
  currentPosition(4, 0) = current_pose.orientation.y;
  currentPosition(5, 0) = current_pose.orientation.z;

  Vector6d desiredPosition;
  desiredPosition = currentPosition + dx_des_vect * CALC_PERIOD;
  desiredPosition(0, 0) = 0.2379;  // TODO initial_pose.position.x;
  desiredPosition(3, 0) = 0.5;
  desiredPosition(4, 0) = 0.5;
  desiredPosition(5, 0) = 0.5;

  ROS_DEBUG_STREAM("currentPosition: \n" << currentPosition << "\n");
  ROS_DEBUG_STREAM("desiredPosition: \n" << desiredPosition << "\n");

  Vector6d g = (-jacobian.transpose() * alpha_weight * dx_des_vect) +
               (jacobian.transpose() * CALC_PERIOD * gamma_weight * currentPosition) -
               (jacobian.transpose() * CALC_PERIOD * gamma_weight * desiredPosition);
  ROS_DEBUG_STREAM("g = \n" << g << "\n");

  // control max and min velocity of joint (dq)
  double maxVel = 0.5;
  double lb[] = { -maxVel, -maxVel, -maxVel, -maxVel, -maxVel, -maxVel };
  double ub[] = { +maxVel, +maxVel, +maxVel, +maxVel, +maxVel, +maxVel };

  // Taylor developpement
  // q = q0 + dq*T
  // inequality is :
  // lb < q < ub
  // lb < q0 + dq*T < ub
  // (lb-q0) < dq.T < (ub-q0)

  Eigen::Matrix<double, 12, 6, Eigen::RowMajor> A;
  A.topLeftCorner(6, 6) = Eigen::MatrixXd::Identity(6, 6) * CALC_PERIOD;
  A.bottomLeftCorner(6, 6) = jacobian * CALC_PERIOD;

  double lbA[] = { (joints_limits_min[0] - current_joint_state.position[0]),
                   (joints_limits_min[1] - current_joint_state.position[1]),
                   (joints_limits_min[2] - current_joint_state.position[2]),
                   (joints_limits_min[3] - current_joint_state.position[3]),
                   (joints_limits_min[4] - current_joint_state.position[4]),
                   (joints_limits_min[5] - current_joint_state.position[5]),
                   x_min - currentPosition(0, 0),
                   y_min - currentPosition(1, 0),
                   z_min - currentPosition(2, 0),
                   r_min - currentPosition(3, 0),
                   p_min - currentPosition(4, 0),
                   yaw_min - currentPosition(5, 0) };
  double ubA[] = { (joints_limits_max[0] - current_joint_state.position[0]),
                   (joints_limits_max[1] - current_joint_state.position[1]),
                   (joints_limits_max[2] - current_joint_state.position[2]),
                   (joints_limits_max[3] - current_joint_state.position[3]),
                   (joints_limits_max[4] - current_joint_state.position[4]),
                   (joints_limits_max[5] - current_joint_state.position[5]),
                   x_max - currentPosition(0, 0),
                   y_max - currentPosition(1, 0),
                   z_max - currentPosition(2, 0),
                   r_max - currentPosition(3, 0),
                   p_max - currentPosition(4, 0),
                   yaw_max - currentPosition(5, 0) };

  ROS_DEBUG_STREAM("A = \n" << A << "\n");

  // Solve first QP.
  qpOASES::int_t nWSR = 10;
  int qp_return = 0;
  if (start_flag)
  {
    qp_return = IK->init(hessian.data(), g.data(), A.data(), lb, ub, lbA, ubA, nWSR, 0);
    start_flag = false;
  }
  else
  {
    qp_return = IK->hotstart(hessian.data(), g.data(), A.data(), lb, ub, lbA, ubA, nWSR, 0);
  }

  if (qp_return == qpOASES::SUCCESSFUL_RETURN)
  {
    ROS_INFO_STREAM("qpOASES : succesfully return");

    // Get and print solution of first QP
    qpOASES::real_t xOpt[6];

    IK->getPrimalSolution(xOpt);
    double theta_tmp[6];
    theta_tmp[0] = current_joint_state.position[0] + xOpt[0] * CALC_PERIOD;
    theta_tmp[1] = current_joint_state.position[1] + xOpt[1] * CALC_PERIOD;
    theta_tmp[2] = current_joint_state.position[2] + xOpt[2] * CALC_PERIOD;
    theta_tmp[3] = current_joint_state.position[3] + xOpt[3] * CALC_PERIOD;
    theta_tmp[4] = current_joint_state.position[4] + xOpt[4] * CALC_PERIOD;
    theta_tmp[5] = current_joint_state.position[5] + xOpt[5] * CALC_PERIOD;

    bool limit_detected = false;
    // Check joints limits
    for (int i = 0; i < 6; i++)
    {
      if (theta_tmp[i] > joints_limits_max[i])
      {
        ROS_WARN_STREAM("joint_" << i + 1 << " max limit overshoot : " << theta_tmp[i] << ">" << joints_limits_max[i]);
        limit_detected = true;
      }
      else if (theta_tmp[i] < joints_limits_min[i])
      {
        ROS_WARN_STREAM("joint_" << i + 1 << " min limit overshoot : " << theta_tmp[i] << "<" << joints_limits_min[i]);
        limit_detected = true;
      }
    }
    if (!limit_detected)
    {
      joint_position_command[0] = theta_tmp[0];
      joint_position_command[1] = theta_tmp[1];
      joint_position_command[2] = theta_tmp[2];
      joint_position_command[3] = theta_tmp[3];
      joint_position_command[4] = theta_tmp[4];
      joint_position_command[5] = theta_tmp[5];
    }
  }
  else
  {
    ROS_ERROR_STREAM("qpOASES : Failed !!!");
  }

  ROS_DEBUG_STREAM("joint_position_command: \n[" << joint_position_command[0] << ", " << joint_position_command[1] << ", " << joint_position_command[2] << ", " << joint_position_command[3] << ", "
                                       << joint_position_command[4] << ", " << joint_position_command[5] << "]\n");

  geometry_msgs::Pose pose_des;
  pose_des.position.x = desiredPosition(0, 0);
  pose_des.position.y = desiredPosition(1, 0);
  pose_des.position.z = desiredPosition(2, 0);
  pose_des.orientation.x = desiredPosition(3, 0);
  pose_des.orientation.y = desiredPosition(4, 0);
  pose_des.orientation.z = desiredPosition(5, 0);
//   debug_des_pub_.publish(pose_des);
  
}
}
