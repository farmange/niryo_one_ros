// TODO copyright
#include <ros/ros.h>

#include "orthopus_interface/inverse_kinematic.h"

#include <eigen_conversions/eigen_msg.h>
#include <tf/tf.h>

namespace cartesian_controller
{
InverseKinematic::InverseKinematic()
{
  ros::param::get("~alpha_1", alpha_1);
  ros::param::get("~alpha_2", alpha_2);
  ros::param::get("~alpha_3", alpha_3);
  ros::param::get("~alpha_4", alpha_4);
  ros::param::get("~alpha_5", alpha_5);
  ros::param::get("~alpha_6", alpha_6);
  ros::param::get("~alpha_7", alpha_7);
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
  ros::param::get("~gamma_7", gamma_7);

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

  // TODO check init all members
  // TODO for loop

  sampling_freq_ = 0;
  sampling_period_ = 0;
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
}

void InverseKinematic::Init(int sampling_freq,
                            ros::Publisher& debug_pose_current,
                            ros::Publisher& debug_pose_desired,
                            ros::Publisher& debug_pose_meas,
                            ros::Publisher& debug_joint_desired,
                            ros::Publisher& debug_joint_min_limit,
                            ros::Publisher& debug_joint_max_limit)
{
  debug_pose_current_ = debug_pose_current;
  debug_pose_desired_ = debug_pose_desired;  
  debug_pose_meas_ = debug_pose_meas;  
  debug_joint_desired_ = debug_joint_desired;
  debug_joint_min_limit_ = debug_joint_min_limit;
  debug_joint_max_limit_ = debug_joint_max_limit;
  sampling_freq_ = sampling_freq;
  if(sampling_freq == 0)
  {
    ROS_ERROR("Invalid sampling frequency. Cannot be zero !");
    sampling_period_ = 0;
    return;
  }
  sampling_period_ = 1.0 / sampling_freq_;
}

void InverseKinematic::Reset(sensor_msgs::JointState& current_joint_state)
{
  ROS_WARN("InverseKinematic::Reset");
  kinematic_state->setVariableValues(current_joint_state);

  const Eigen::Affine3d& end_effector_state =
  kinematic_state->getGlobalLinkTransform("tool_link");  // TODO Add configuration parameter
  tf::poseEigenToMsg(end_effector_state, current_pose);

  tf::Quaternion q(current_pose.orientation.x, 
                   current_pose.orientation.y, 
                   current_pose.orientation.z, 
                   current_pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  
//   if(std::abs(roll) >= M_PI/2)
//   {
//     pitch = -pitch;
//     yaw = -yaw;
//   }
//   if(std::abs(pitch) >= M_PI/2)
//   {
//     roll = -roll;
//     yaw = -yaw;
//   }
//   if(std::abs(yaw) >= M_PI/2)
//   {
//     roll = -roll;
//     pitch = -pitch;
//   }
  x_cmd_prev(0, 0) = current_pose.position.x;
  x_cmd_prev(1, 0) = current_pose.position.y;
  x_cmd_prev(2, 0) = current_pose.position.z;
  x_cmd_prev(3, 0) = roll;
  x_cmd_prev(4, 0) = pitch;
  x_cmd_prev(5, 0) = yaw;

  /* Initialize previous cartesian state used to order to detect euler jump */
  x_cmd_prev_saved = x_cmd_prev;

  /* Initialize computed cartesian state. This state is only used for debugging purpose 
   * in order to check concistency between joint positions computed and 
   * resulting cartesian state */
  // TODO not used for now because we take the x_cmd_prev each time 
  x_computed = x_cmd_prev;
  /* Initialize a flag used to init QP if required */ 
  qp_init_required = true; 

  /* Initialize all cartesian constraints */
  for(int i=0; i<6; i++)
  {
    request_update_constraint[i] = true;
    request_update_constraint_tolerance[i] = 0.001;
  }
  request_update_constraint_tolerance[3] = 0.01;
  request_update_constraint_tolerance[4] = 0.01;
  request_update_constraint_tolerance[5] = 0.01;
  UpdateAxisConstraints();
  
  x_des = x_cmd_prev;
}

void InverseKinematic::ResolveInverseKinematic(double (&joint_position_command)[6],
                                               sensor_msgs::JointState& current_joint_state,
                                               double (&cartesian_velocity_desired)[6])
{
  sensor_msgs::JointState q_cmd_prev = current_joint_state;
  ROS_WARN("current_joint_state \t| joint_position_command \t| q_cmd_prev");
  for (std::size_t i = 0; i < 6; ++i)
  {
    q_cmd_prev.position[i] = joint_position_command[i];
  }
  
  /* Set kinemtic state of the robot to the previous joint positions computed */
  kinematic_state->setVariableValues(q_cmd_prev);
  kinematic_state->updateLinkTransforms();
 
  /* Get the cartesian state of the tool_link frame */
  const Eigen::Affine3d& end_effector_state = 
  kinematic_state->getGlobalLinkTransform(kinematic_state->getLinkModel("tool_link"));  // TODO Add configuration parameter  

  /* Convert cartesian state to geometry_msgs::Pose */
  tf::poseEigenToMsg(end_effector_state, current_pose);
  geometry_msgs::Pose saved_quat_pose;
  tf::poseEigenToMsg(end_effector_state, saved_quat_pose);
  ROS_WARN("current_pose");
  ROS_WARN_STREAM(current_pose);
  
  /* Convert quaternion pose in RPY */
  tf::Quaternion q(current_pose.orientation.x, 
                   current_pose.orientation.y, 
                   current_pose.orientation.z, 
                   current_pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  
//   if(std::abs(roll) >= M_PI/2)
//   {
//     pitch = -pitch;
//     yaw = -yaw;
//   }
//   if(std::abs(pitch) >= M_PI/2)
//   {
//     roll = -roll;
//     yaw = -yaw;
//   }
//   if(std::abs(yaw) >= M_PI/2)
//   {
//     roll = -roll;
//     pitch = -pitch;
//   }
  
  /* Store RPY pose in x_cmd_prev vector */
  x_cmd_prev(0, 0) = current_pose.position.x;
  x_cmd_prev(1, 0) = current_pose.position.y;
  x_cmd_prev(2, 0) = current_pose.position.z;
  x_cmd_prev(3, 0) = roll;
  x_cmd_prev(4, 0) = pitch;
  x_cmd_prev(5, 0) = yaw;
  
  
//   /* Look for euler jump in RPY and update constraints if needed */
//   for(int i=3; i<6; i++)
//   {
//     if(std::abs(x_cmd_prev(i, 0) - x_cmd_prev_saved(i, 0)) >= M_PI/2)
//     {
//       ROS_ERROR_STREAM("==============> Euler jump detected on axis " << i << " !!!! delta=" << x_cmd_prev(i, 0) - x_cmd_prev_saved(i, 0) <<"");
//       RequestUpdateAxisConstraints(i);
//       if(x_cmd_prev(i, 0) - x_cmd_prev_saved(i, 0) >= M_PI/2)
//       {
//         x_des[i] = x_des[i] + M_PI*2;
//       }
//       else if(x_cmd_prev(i, 0) - x_cmd_prev_saved(i, 0) <= M_PI/2)
//       {
//         x_des[i] = x_des[i] - M_PI*2;
//       }
//     }
//   }
  //x_des = x_des_init;// + dx_des_vect * sampling_period_;
  
//   x_cmd_prev_saved = x_cmd_prev;
   UpdateAxisConstraints();

  /* Get jacobian from kinematic state (Moveit) */
  Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
  Eigen::MatrixXd jacobian;
  if (!kinematic_state->getJacobian(joint_model_group,
                                    kinematic_state->getLinkModel("tool_link"),
                                    reference_point_position, 
                                    jacobian))
  {
    ROS_ERROR_STREAM("JACOBIAN COMPUTATION ISSUE");
    exit(0);
  }

  /* Store desired velocity in dx_des_vect for eigen computation */
  const Vector6d dx_des_vect = Vector6d(cartesian_velocity_desired);
  
  Matrix6d hessian =  (jacobian.transpose() * alpha_weight * jacobian) + beta_weight +
                      (jacobian.transpose() * sampling_period_ * gamma_weight * sampling_period_ * jacobian);
                      
  Vector6d g =  (-jacobian.transpose() * alpha_weight * dx_des_vect) +
                (jacobian.transpose() * sampling_period_ * gamma_weight * x_cmd_prev) -
                (jacobian.transpose() * sampling_period_ * gamma_weight * x_des);

  /* Set joint velocities bound */
  double maxVel = 0.9;
  double lb[] = { -maxVel, -maxVel, -maxVel, -maxVel, -maxVel, -maxVel };
  double ub[] = { +maxVel, +maxVel, +maxVel, +maxVel, +maxVel, +maxVel };
  
  // Taylor developpement
  // q = q0 + dq*T
  // inequality is :
  // lb < q < ub
  // lb < q0 + dq*T < ub
  // (lb-q0) < dq.T < (ub-q0)
  Eigen::Matrix<double, 12, 6, Eigen::RowMajor> A;
   A.topLeftCorner(6, 6) = Eigen::MatrixXd::Identity(6, 6) * sampling_period_;
   A.bottomLeftCorner(6, 6) = jacobian.topLeftCorner(6, 6) * sampling_period_;

  double lbA[] = { 
                  /* Joints min hard limits constraints */
                  (joints_limits_min[0] - q_cmd_prev.position[0]),
                  (joints_limits_min[1] - q_cmd_prev.position[1]),  
                  (joints_limits_min[2] - q_cmd_prev.position[2]),
                  (joints_limits_min[3] - q_cmd_prev.position[3]),
                  (joints_limits_min[4] - q_cmd_prev.position[4]),
                  (joints_limits_min[5] - q_cmd_prev.position[5]),
                  (x_min_limit(0, 0) - x_cmd_prev(0, 0)),
                  (x_min_limit(1, 0) - x_cmd_prev(1, 0)),
                  (x_min_limit(2, 0) - x_cmd_prev(2, 0)),                  
                  (x_min_limit(3, 0) - x_cmd_prev(3, 0)),
                  (x_min_limit(4, 0) - x_cmd_prev(4, 0)),
                  (x_min_limit(5, 0) - x_cmd_prev(5, 0))

  };
  double ubA[] = {
                  /* Joints min hard limits constraints */
                  (joints_limits_max[0] - q_cmd_prev.position[0]),
                  (joints_limits_max[1] - q_cmd_prev.position[1]),
                  (joints_limits_max[2] - q_cmd_prev.position[2]),
                  (joints_limits_max[3] - q_cmd_prev.position[3]),
                  (joints_limits_max[4] - q_cmd_prev.position[4]),
                  (joints_limits_max[5] - q_cmd_prev.position[5]),
                  (x_max_limit(0, 0) - x_cmd_prev(0, 0)),
                  (x_max_limit(1, 0) - x_cmd_prev(1, 0)),
                  (x_max_limit(2, 0) - x_cmd_prev(2, 0)),
                  (x_max_limit(3, 0) - x_cmd_prev(3, 0)),
                  (x_max_limit(4, 0) - x_cmd_prev(4, 0)),
                  (x_max_limit(5, 0) - x_cmd_prev(5, 0))

  };
                   
  // Solve first QP.
  qpOASES::real_t dq_computed[6];
  qpOASES::real_t yOpt[6+1];
  qpOASES::int_t nWSR = 20;
  int qp_return = 0;
  if (qp_init_required)
  {
    // Initialize QP solver
    IK = new qpOASES::SQProblem(6, 9);
    qpOASES::Options options;
    options.setToReliable( );
    // options.printLevel = qpOASES::PL_NONE;
    IK->setOptions(options);
    
    qp_return = IK->init(hessian.data(), g.data(), A.data(), lb, ub, lbA, ubA, nWSR, 0);
    qp_init_required = false;
  }
  else
  {
    qp_return = IK->hotstart(hessian.data(), g.data(), A.data(), lb, ub, lbA, ubA, nWSR, 0);
  }  
  
  if (qp_return == qpOASES::SUCCESSFUL_RETURN)
  {
    ROS_DEBUG_STREAM("qpOASES : succesfully return");

    // Get and print solution of first QP
    IK->getPrimalSolution(dq_computed);
    double theta_tmp[6];
    theta_tmp[0] = q_cmd_prev.position[0] + dq_computed[0] * sampling_period_;
    theta_tmp[1] = q_cmd_prev.position[1] + dq_computed[1] * sampling_period_;
    theta_tmp[2] = q_cmd_prev.position[2] + dq_computed[2] * sampling_period_;
    theta_tmp[3] = q_cmd_prev.position[3] + dq_computed[3] * sampling_period_;
    theta_tmp[4] = q_cmd_prev.position[4] + dq_computed[4] * sampling_period_;
    theta_tmp[5] = q_cmd_prev.position[5] + dq_computed[5] * sampling_period_;
    
    bool limit_detected = false;
    // Check joints limits
    for (int i = 0; i < 6; i++)
    {
      if (theta_tmp[i] > joints_limits_max[i])
      {
        ROS_WARN_STREAM("joint_" << i + 1 << " max limit overshoot : " << theta_tmp[i] << " > "
                                 << joints_limits_max[i]);
        limit_detected = true;
      }
      else if (theta_tmp[i] < joints_limits_min[i])
      {
        ROS_WARN_STREAM("joint_" << i + 1 << " min limit overshoot : " << theta_tmp[i] << " < "
                                 << joints_limits_min[i]);
        limit_detected = true;
      }
    }
//     if (!limit_detected)
//     {
      joint_position_command[0] = theta_tmp[0];
      joint_position_command[1] = theta_tmp[1];
      joint_position_command[2] = theta_tmp[2];
      joint_position_command[3] = theta_tmp[3];
      joint_position_command[4] = theta_tmp[4];
      joint_position_command[5] = theta_tmp[5];
//     }
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
//   PrintVector("joint_state", Vector6d(current_joint_state.position.data()));
//   PrintVector("joints_lim_min", Vector6d(joints_limits_min));
//   PrintVector("q_cmd_prev", Vector6d(q_cmd_prev.position.data()));
//   PrintVector("q_cmd_new", Vector6d(joint_position_command));
//   PrintVector("joints_lim_max", Vector6d(joints_limits_max));
//   PrintVector("dq_computed", Vector6d(dq_computed));
//   ROS_DEBUG_STREAM("================");
// //   ROS_DEBUG_STREAM("Jacobian: \n" << jacobian);
//   PrintVector("dx_des_vect", dx_des_vect);
// //   ROS_DEBUG_STREAM("hessian: \n" << hessian );
// //   ROS_DEBUG_STREAM("g = \n" << g);
//    ROS_DEBUG_STREAM("A: \n" << A );
//   ROS_DEBUG_STREAM("================");
//   PrintVector("x_min_limit", x_min_limit);
//   PrintVector("x_cmd_prev", x_cmd_prev);
//   PrintVector("x_max_limit", x_max_limit);
//   ROS_DEBUG_STREAM("================");
//   Vector6d dx_computed = Vector6d::Zero();
//   dx_computed = jacobian*Vector6d(dq_computed);
//   PrintVector("dx_computed", dx_computed);
//   Vector6d j_dq_t = dx_computed*sampling_period_;
//   ROS_DEBUG_STREAM("================");
//   PrintVector("x_computed (old)", x_computed);
//   PrintVector("x_cmd_prev", x_cmd_prev);
//   x_computed = x_cmd_prev + j_dq_t;
//   ROS_DEBUG_STREAM("================");
//   PrintVector("x_computed (new)", x_computed);
//   ROS_DEBUG_STREAM("================");

  
//   /* Publish debug topics */
//   geometry_msgs::Pose current_pose_debug;
//   current_pose_debug.position.x = x_cmd_prev(0, 0); 
//   current_pose_debug.position.y = x_cmd_prev(1, 0); 
//   current_pose_debug.position.z = x_cmd_prev(2, 0); 
//   current_pose_debug.orientation.x = x_cmd_prev(3, 0); 
//   current_pose_debug.orientation.y = x_cmd_prev(4, 0); 
//   current_pose_debug.orientation.z = x_cmd_prev(5, 0); 
//   debug_pose_current_.publish(current_pose_debug);
//   
//   geometry_msgs::Pose desired_pose;
//   desired_pose.position.x = saved_quat_pose.position.x; 
//   desired_pose.position.y = saved_quat_pose.position.y; 
//   desired_pose.position.z = saved_quat_pose.position.z; 
//   desired_pose.orientation.x = saved_quat_pose.orientation.x; 
//   desired_pose.orientation.y = saved_quat_pose.orientation.y; 
//   desired_pose.orientation.z = saved_quat_pose.orientation.z; 
//   desired_pose.orientation.w = saved_quat_pose.orientation.w; 
//   debug_pose_desired_.publish(desired_pose);  
//   
//   geometry_msgs::Pose meas_pose;
//   meas_pose.position.x = x_des(0, 0); 
//   meas_pose.position.y = x_des(1, 0); 
//   meas_pose.position.z = x_des(2, 0); 
//   meas_pose.orientation.x = x_des(3, 0); 
//   meas_pose.orientation.y = x_des(4, 0); 
//   meas_pose.orientation.z = x_des(5, 0); 
//   debug_pose_meas_.publish(meas_pose);  
  
//   sensor_msgs::JointState desired_joints;
//   desired_joints.header = current_joint_state.header;
//   desired_joints.position.resize(6);
//   desired_joints.position[0] = joint_position_command[0];
//   desired_joints.position[1] = joint_position_command[1];
//   desired_joints.position[2] = joint_position_command[2];
//   desired_joints.position[3] = joint_position_command[3];
//   desired_joints.position[4] = joint_position_command[4];
//   desired_joints.position[5] = joint_position_command[5];
//   debug_joint_desired_.publish(desired_joints);
//   
//   sensor_msgs::JointState min_joints;
//   min_joints.header = current_joint_state.header;
//   min_joints.position.resize(6);
//   min_joints.position[0] = joints_limits_min[0];
//   min_joints.position[1] = joints_limits_min[1];
//   min_joints.position[2] = joints_limits_min[2];
//   min_joints.position[3] = joints_limits_min[3];
//   min_joints.position[4] = joints_limits_min[4];
//   min_joints.position[5] = joints_limits_min[5];
//   debug_joint_min_limit_.publish(min_joints);
// 
//   sensor_msgs::JointState max_joints;
//   max_joints.header = current_joint_state.header;
//   max_joints.position.resize(6);
//   max_joints.position[0] = joints_limits_max[0];
//   max_joints.position[1] = joints_limits_max[1];
//   max_joints.position[2] = joints_limits_max[2];
//   max_joints.position[3] = joints_limits_max[3];
//   max_joints.position[4] = joints_limits_max[4];
//   max_joints.position[5] = joints_limits_max[5];
//   debug_joint_max_limit_.publish(max_joints);
  
  if (qp_return != qpOASES::SUCCESSFUL_RETURN && qp_return != qpOASES::RET_MAX_NWSR_REACHED)
  {
    exit(0);
  }
}

void InverseKinematic::PrintVector(const std::string name, const Vector6d vector) const 
{
  std::string reformatted_name = name;
  reformatted_name.resize(14, ' ');
  ROS_DEBUG("%s : %5f, \t%5f, \t%5f, \t%5f, \t%5f, \t%5f", reformatted_name.c_str(), vector(0,0), vector(1,0), vector(2,0), vector(3,0), vector(4,0), vector(5,0));
}

void InverseKinematic::UpdateAxisConstraints()
{
  ROS_WARN_STREAM("InverseKinematic::UpdateAxisConstraints");
  for(int i=0;i<6;i++)
  {
    if(request_update_constraint[i])
    {
      ROS_WARN_STREAM("Update axis " << i << " constraints with new tolerance of " << request_update_constraint_tolerance[i] << " m.");
      if(i<6)
      {
          x_min_limit(i,0) = x_cmd_prev(i, 0) - request_update_constraint_tolerance[i];
          x_max_limit(i,0) = x_cmd_prev(i, 0) + request_update_constraint_tolerance[i];
      }
      else
      {
        x_min_limit(i,0) = - request_update_constraint_tolerance[i];
        x_max_limit(i,0) = + request_update_constraint_tolerance[i];
      }
      request_update_constraint[i] = false;
      x_des[i] =  x_cmd_prev[i];      
      ROS_WARN_STREAM("Update the new desired position of " << i << " to " << x_des[i]);
    }
  }
}

// Keep last set tolerance
void InverseKinematic::RequestUpdateAxisConstraints(int axis)
{
  ROS_WARN_STREAM("Update axis " << axis << " constraints with the last tolerance set (" << request_update_constraint_tolerance[axis] << ")");
  request_update_constraint[axis] = true;
}
void InverseKinematic::RequestUpdateAxisConstraints(int axis, double tolerance)
{
  ROS_WARN_STREAM("Update axis " << axis << " constraints with new tolerance of " << tolerance << "");
  request_update_constraint[axis] = true;
  request_update_constraint_tolerance[axis] = tolerance;
}
}
