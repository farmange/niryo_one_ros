
#include <ros/ros.h>
// #include <ros/callback_queue.h>
#include <std_msgs/String.h>

// Messages
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <niryo_one_msgs/RobotMoveAction.h>
#include <niryo_one_msgs/SetInt.h>
#include <niryo_one_msgs/GetInt.h>
#include <niryo_one_msgs/OpenGripper.h>
#include <niryo_one_msgs/CloseGripper.h>

// actionlib
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/action_server.h>
#include <actionlib/client/action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <actionlib/client/terminal_state.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
// #include <moveit/move_group_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// QPOASES
#include <qpOASES.hpp>

// Eigen3
#include <Eigen/Dense>

#include <eigen_conversions/eigen_msg.h>

// static const int NUM_SPINNERS = 2;
// static const int JOINT_SUB_QUEUE_LENGTH = 1;
// static const int X_DOT_DES_QUEUE_LENGTH = 1;
static const double LINEAR_SCALE = 1.0;
static const double ROTATIONAL_SCALE = 1.0;
static const double TO_RAD = (3.141589 / 180.0);

static const int RATE = 10;
static const double CALC_PERIOD = 1.0 / RATE;
static const double INIT_DURATION = 8;

int TOOL_ID = 12;  // change this depenidning on the tool mounted

using namespace std;
typedef actionlib::SimpleActionClient<niryo_one_msgs::RobotMoveAction> NiryoClient;


class JoystickCartesianInterface
{
public:
  JoystickCartesianInterface()  :  ac("/niryo_one/commander/robot_action/", true)//: spinner_(NUM_SPINNERS)
  {
    command_pub_ = n_.advertise<trajectory_msgs::JointTrajectory>(
        "/niryo_one_follow_joint_trajectory_controller/command", 10);  // TODO check optimal queue size
    joints_sub_ = n_.subscribe("joint_states", 1, &JoystickCartesianInterface::jointStatesCB, this);
    dx_des_sub_ = n_.subscribe("dx_des", 1, &JoystickCartesianInterface::dxDesCB, this);
    gripper_des_sub_ = n_.subscribe("gripper_des", 1, &JoystickCartesianInterface::gripperDesCB, this);

    debug_pub_ = n_.advertise<geometry_msgs::Pose>("/debug_cartesian_pos", 1);
    changeToolClient_ = n_.serviceClient<niryo_one_msgs::SetInt>("/niryo_one/change_tool/");
    
    ros::ServiceServer service = n_.advertiseService("/niryo_one/joystick_interface/enable", &JoystickCartesianInterface::joystickEnableCB, this);


   
    // Connecting to the robot ===========================================
    ROS_INFO("Connecting to robot  ========================");
//     NiryoClient ac ("/niryo_one/commander/robot_action/", true);
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();
    ROS_INFO("Action server started, sending goal.");
    
    // wait for the action server to start
    bool connection_success = false;
    while (!connection_success)
    {
      connection_success = ac.waitForServer(ros::Duration(3.0));
      if (connection_success)
      {
        ROS_INFO("  Robot Connection established");
      }
      else
      {
        ROS_WARN("  Error connecting to Robot. Trying again");
      }
    }

    // Setting Gripper
    ROS_INFO("Setting gripper");
    niryo_one_msgs::SetInt change_tool_srv;
    change_tool_srv.request.value = TOOL_ID;
    while (!changeToolClient_.call(change_tool_srv))
    {
      ROS_WARN("  Could not set the tool type. Trying again in one second");
      ros::Duration(1.0).sleep();
    }
    ROS_INFO("  Success");

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

    double joints_limits_max[6];
    double joints_limits_min[6];
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

    for (int i = 0; i < 6; i++)
    {
      ROS_INFO_STREAM("joint_" << i + 1 << " limits : [" << joints_limits_min[i] << ";" << joints_limits_max[i] << "]");
    }

    // Wait for initial messages
    ROS_INFO("Waiting for first joint msg.");
    ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states");
    ROS_INFO("Received first joint msg.");

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm");

    ros::Rate loop_rate = ros::Rate(RATE);

    // Minimize cartesian velocity : dx
    Eigen::MatrixXd alpha_weight(6, 6);
    alpha_weight = Eigen::MatrixXd::Identity(6, 6);
    alpha_weight(0, 0) = alpha_1;
    alpha_weight(1, 1) = alpha_2;
    alpha_weight(2, 2) = alpha_3;
    alpha_weight(3, 3) = alpha_4;
    alpha_weight(4, 4) = alpha_5;
    alpha_weight(5, 5) = alpha_6;
    ROS_DEBUG_STREAM("alpha_weight: \n" << alpha_weight << "\n");

    // Minimize joints velocities : dq
    Eigen::MatrixXd beta_weight(6, 6);
    beta_weight = Eigen::MatrixXd::Identity(6, 6);
    beta_weight(0, 0) = beta_1;
    beta_weight(1, 1) = beta_2;
    beta_weight(2, 2) = beta_3;
    beta_weight(3, 3) = beta_4;
    beta_weight(4, 4) = beta_5;
    beta_weight(5, 5) = beta_6;
    ROS_DEBUG_STREAM("beta_weight: \n" << beta_weight << "\n");

    // Minimize cartesian position : x
    Eigen::MatrixXd gamma_weight(6, 6);
    gamma_weight = Eigen::MatrixXd::Identity(6, 6);
    gamma_weight(0, 0) = gamma_1;
    gamma_weight(1, 1) = gamma_2;
    gamma_weight(2, 2) = gamma_3;
    gamma_weight(3, 3) = gamma_4;
    gamma_weight(4, 4) = gamma_5;
    gamma_weight(5, 5) = gamma_6;
    ROS_DEBUG_STREAM("gamma_weight: \n" << gamma_weight << "\n");

    IK = qpOASES::SQProblem(6, 6);
    qpOASES::Options options;
    // options.printLevel = qpOASES::PL_NONE;
    IK.setOptions(options);

    ROS_INFO("Initialize robot position");
    // initialize robot in zero position
    while (ros::ok())
    {
      ros::spinOnce();
      theta[0] = 0.0;
      theta[1] = 0.0;
      theta[2] = 0.0;
      theta[3] = 0.0;
      theta[4] = 0.0;
      theta[5] = 0.0;
      sendInitCommand();
      //       if(q_meas_.position[0]==0 &&
      //         q_meas_.position[1]==0 &&
      //         q_meas_.position[2]==0 &&
      //         q_meas_.position[3]==0 &&
      //         q_meas_.position[4]==0 &&
      //         q_meas_.position[5]==0)
      //         {
      ros::Duration(INIT_DURATION+1.0).sleep();
      break;
      //         }
    }
    ros::Duration(2.0).sleep();

    ROS_INFO("Run QP processing");
    while (ros::ok())
    {
      ros::spinOnce();

      kinematic_state->setVariableValues(q_meas_);

      const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
      std::vector<double> joint_values;
      std::vector<double> joint_values_copy = std::vector<double>(6);
      kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
      for (std::size_t i = 0; i < joint_names.size(); ++i)
      {
        ROS_INFO("Joint %s: %f (q_meas_=%f)", joint_names[i].c_str(), joint_values[i], q_meas_.position[i]);
        joint_values_copy[i] = joint_values[i];
      }

      const Eigen::Affine3d& end_effector_state =
          kinematic_state->getGlobalLinkTransform("hand_link");  // TODO is it the right link to control ?
      geometry_msgs::Pose current_pose;
      tf::poseEigenToMsg(end_effector_state, current_pose);
      debug_pub_.publish(current_pose);

      Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
      Eigen::MatrixXd jacobian;
      kinematic_state->getJacobian(joint_model_group,
                                   kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                                   reference_point_position, jacobian);
      ROS_DEBUG_STREAM("Jacobian: \n" << jacobian << "\n");

      const Eigen::VectorXd dx_des_vect = scaleCartesianCommand(dx_des_);
      ROS_INFO_STREAM("dx_des_vect: \n" << dx_des_vect << "\n");
      ROS_DEBUG_STREAM("alpha_weight: \n" << alpha_weight << "\n");
      ROS_DEBUG_STREAM("beta_weight: \n" << beta_weight << "\n");
      ROS_DEBUG_STREAM("gamma_weight: \n" << gamma_weight << "\n");

      // Compute matrix for the quadratic problem
      //         Eigen::MatrixXd jacobianTranspose = jacobian.transpose();
      //         ROS_DEBUG_STREAM("jacobianTranspose: \n" << jacobianTranspose << "\n");

      Eigen::MatrixXd hessian = (jacobian.transpose() * alpha_weight * jacobian) + beta_weight +
                                (jacobian.transpose() * CALC_PERIOD * gamma_weight * CALC_PERIOD * jacobian);
      ROS_INFO_STREAM("hessian: \n" << hessian << "\n");
      ROS_INFO_STREAM("hessian.determinant(): \n" << hessian.determinant() << "\n");
      //       Eigen::MatrixXd L = hessian.llt().matrixL();

      Eigen::LLT<Eigen::MatrixXd> lltOfA(hessian);  // compute the Cholesky decomposition of A
      Eigen::MatrixXd L = lltOfA.matrixL();
      ROS_INFO_STREAM("L: \n" << L << "\n");
      ROS_INFO_STREAM("lltOfA.info(): \n" << lltOfA.info() << "\n");

      Eigen::MatrixXd currentPosition(6, 1);
      currentPosition(0, 0) = current_pose.position.x;
      currentPosition(1, 0) = current_pose.position.y;
      currentPosition(2, 0) = current_pose.position.z;
      currentPosition(3, 0) = current_pose.orientation.x;
      currentPosition(4, 0) = current_pose.orientation.y;
      currentPosition(5, 0) = current_pose.orientation.z;

      Eigen::MatrixXd desiredPosition(6, 1);
      desiredPosition = currentPosition + dx_des_vect * CALC_PERIOD;
      desiredPosition(0, 0) = 0.2379;  // initial_pose.position.x;
      desiredPosition(3, 0) = 0.5;
      desiredPosition(4, 0) = 0.5;
      desiredPosition(5, 0) = 0.5;

      double x_min = 0.2;
      double x_max = 0.3;
      double y_min = -0.25;
      double y_max = 0.25;
      double z_min = 0.05;
      double z_max = 0.55;
      double r_min = -2.0;
      double r_max = 2.0;
      double p_min = -2.0;
      double p_max = 2.0;
      double yaw_min = -2.0;
      double yaw_max = 2.0;
      desiredPosition(1, 0) = std::min(desiredPosition(1, 0), y_max);
      desiredPosition(1, 0) = std::max(desiredPosition(1, 0), y_min);
      desiredPosition(2, 0) = std::min(desiredPosition(2, 0), z_max);
      desiredPosition(2, 0) = std::max(desiredPosition(2, 0), z_min);

      ROS_INFO_STREAM("currentPosition: \n" << currentPosition << "\n");
      ROS_INFO_STREAM("desiredPosition: \n" << desiredPosition << "\n");

      Eigen::MatrixXd g = (-jacobian.transpose() * alpha_weight * dx_des_vect) +
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

      Eigen::MatrixXd A(12, 6);
      A.topLeftCorner(6, 6) = Eigen::MatrixXd::Identity(6, 6) * CALC_PERIOD;
      A.bottomLeftCorner(6, 6) = jacobian * CALC_PERIOD;

      //       joints_limits_min[0] = -20 * TO_RAD;
      //       joints_limits_max[0] = 20 * TO_RAD;
      //       joints_limits_min[1] = -20 * TO_RAD;
      //       joints_limits_max[1] = 20 * TO_RAD;
      //       joints_limits_min[2] = -20 * TO_RAD;
      //       joints_limits_max[2] = 20 * TO_RAD;

      double lbA[] = { (joints_limits_min[0] - joint_values_copy[0]),
                       (joints_limits_min[1] - joint_values_copy[1]),
                       (joints_limits_min[2] - joint_values_copy[2]),
                       (joints_limits_min[3] - joint_values_copy[3]),
                       (joints_limits_min[4] - joint_values_copy[4]),
                       (joints_limits_min[5] - joint_values_copy[5]),
                       x_min - currentPosition(0, 0),
                       y_min - currentPosition(1, 0),
                       z_min - currentPosition(2, 0),
                       r_min - currentPosition(3, 0),
                       p_min - currentPosition(4, 0),
                       yaw_min - currentPosition(5, 0) };
      double ubA[] = { (joints_limits_max[0] - joint_values_copy[0]),
                       (joints_limits_max[1] - joint_values_copy[1]),
                       (joints_limits_max[2] - joint_values_copy[2]),
                       (joints_limits_max[3] - joint_values_copy[3]),
                       (joints_limits_max[4] - joint_values_copy[4]),
                       (joints_limits_max[5] - joint_values_copy[5]),
                       x_max - currentPosition(0, 0),
                       y_max - currentPosition(1, 0),
                       z_max - currentPosition(2, 0),
                       r_max - currentPosition(3, 0),
                       p_max - currentPosition(4, 0),
                       yaw_max - currentPosition(5, 0) };

      //       ROS_INFO_STREAM("A = \n" << A << "\n");

      // Solve first QP.
      qpOASES::int_t nWSR = 10;
      int qp_return = 0;
      if (start_flag)
      {
        qp_return = IK.init(hessian.data(), g.data(), A.data(), lb, ub, lbA, ubA, nWSR, 0);
        start_flag = false;
      }
      else
      {
        qp_return = IK.hotstart(hessian.data(), g.data(), A.data(), lb, ub, lbA, ubA, nWSR, 0);
      }

      if (qp_return == qpOASES::SUCCESSFUL_RETURN)
      {
        ROS_INFO_STREAM("qpOASES : succesfully return");

        // Get and print solution of first QP
        qpOASES::real_t xOpt[6];

        IK.getPrimalSolution(xOpt);
        ROS_INFO_STREAM("theta[1] = " << theta[1]);
        ROS_INFO_STREAM("joint_values[1] = " << joint_values[1]);
        ROS_INFO_STREAM("joint_values_copy[1] = " << joint_values_copy[1]);
        ROS_INFO_STREAM("xOpt[1] = " << xOpt[1]);
        ROS_INFO_STREAM("CALC_PERIOD = " << CALC_PERIOD);
        double theta_tmp[6];
        theta_tmp[0] = joint_values_copy[0] + xOpt[0] * CALC_PERIOD;
        theta_tmp[1] = joint_values_copy[1] + xOpt[1] * CALC_PERIOD;
        theta_tmp[2] = joint_values_copy[2] + xOpt[2] * CALC_PERIOD;
        theta_tmp[3] = joint_values_copy[3] + xOpt[3] * CALC_PERIOD;
        theta_tmp[4] = joint_values_copy[4] + xOpt[4] * CALC_PERIOD;
        theta_tmp[5] = joint_values_copy[5] + xOpt[5] * CALC_PERIOD;
        ROS_INFO_STREAM("theta_tmp = " << theta_tmp);

        bool limit_detected = false;
        // Check joints limits
        for (int i = 0; i < 6; i++)
        {
          if (theta_tmp[i] > joints_limits_max[i])
          {
            ROS_WARN_STREAM("joint_" << i + 1 << " max limit overshoot : " << theta_tmp[i] << ">"
                                     << joints_limits_max[i]);
            limit_detected = true;
          }
          else if (theta_tmp[i] < joints_limits_min[i])
          {
            ROS_WARN_STREAM("joint_" << i + 1 << " min limit overshoot : " << theta_tmp[i] << "<"
                                     << joints_limits_min[i]);
            limit_detected = true;
          }
        }
        if (!limit_detected)
        {
          theta[0] = theta_tmp[0];
          theta[1] = theta_tmp[1];
          theta[2] = theta_tmp[2];
          theta[3] = theta_tmp[3];
          theta[4] = theta_tmp[4];
          theta[5] = theta_tmp[5];
        }
        ROS_INFO_STREAM("xOpt: \n[" << xOpt[0] << ", " << xOpt[1] << ", " << xOpt[2] << ", " << xOpt[3] << ", "
                                    << xOpt[4] << ", " << xOpt[5] << "]\n");
      }
      else
      {
        ROS_ERROR_STREAM("qpOASES : Failed !!!");
      }

      ROS_INFO_STREAM("Theta Command: \n[" << theta[0] << ", " << theta[1] << ", " << theta[2] << ", " << theta[3]
                                           << ", " << theta[4] << ", " << theta[5] << "]\n");

      send6DofCommand();
      sendGripperCommand();
      loop_rate.sleep();
    }
  };
  void send6DofCommand()
  {
    trajectory_msgs::JointTrajectory new_jt_traj;
    new_jt_traj.header.stamp = ros::Time::now();
    new_jt_traj.joint_names = q_meas_.name;

    trajectory_msgs::JointTrajectoryPoint point;
    point.time_from_start = ros::Duration(1.0 / RATE);
    point.positions = q_meas_.position;
    for (int i = 0; i < 6; i++)
    {
      point.positions[i] = theta[i];
    }

    // Important : do not send velocity as cubic interpolation is done !
    // It should be better to handle that by proper velocity command (force velocity ramp on xbox controller)
    // point.velocities = q_meas_.velocity;
    new_jt_traj.points.push_back(point);
    command_pub_.publish(new_jt_traj);
  };

  void sendInitCommand()
  {
    trajectory_msgs::JointTrajectory new_jt_traj;

    new_jt_traj.header.stamp = ros::Time::now();
    ROS_INFO_STREAM("new_jt_traj=" << new_jt_traj);
    new_jt_traj.joint_names = q_meas_.name;
    ROS_INFO_STREAM("new_jt_traj=" << new_jt_traj);
    trajectory_msgs::JointTrajectoryPoint point;
    point.time_from_start = ros::Duration(INIT_DURATION);
    point.positions = q_meas_.position;
    for (int i = 0; i < 6; i++)
    {
      point.positions[i] = 0;
    }

    // Important : do not send velocity as cubic interpolation is done !
    // It should be better to handle that by proper velocity command (force velocity ramp on xbox controller)
    // point.velocities = q_meas_.velocity;
    new_jt_traj.points.push_back(point);
    ROS_INFO_STREAM("new_jt_traj=" << new_jt_traj);
    command_pub_.publish(new_jt_traj);
  };

  void sendOpenGripperCommand()
  {
    ROS_INFO("Open gripper");
    niryo_one_msgs::ToolCommand tcmd;
    niryo_one_msgs::RobotMoveActionGoal action;
    tcmd.cmd_type = 1;
    tcmd.gripper_open_speed = 100;
    tcmd.tool_id = TOOL_ID;
    action.goal.cmd.cmd_type = 6;
    action.goal.cmd.tool_cmd = tcmd;
    ac.sendGoal(action.goal);
    ac.waitForResult(ros::Duration(10.0));
  };

  void sendCloseGripperCommand()
  {
    ROS_INFO("Close gripper");
    niryo_one_msgs::ToolCommand tcmd;
    niryo_one_msgs::RobotMoveActionGoal action;
    tcmd.cmd_type = 2;
    tcmd.gripper_open_speed = 100;
    tcmd.tool_id = TOOL_ID;
    action.goal.cmd.cmd_type = 6;
    action.goal.cmd.tool_cmd = tcmd;
    ac.sendGoal(action.goal);
    ac.waitForResult(ros::Duration(10.0));
  };

  void sendGripperCommand()
  {
    if (prev_gripper_des_ != gripper_des_.data)
    {
      if (!gripper_des_.data)
      {
        sendOpenGripperCommand();
      }
      else
      {
        sendCloseGripperCommand();
      }
    }
    prev_gripper_des_ = gripper_des_.data;
  };

private:
  // Convert incoming joy commands to TwistStamped commands for jogging.
  // The TwistStamped component goes to jogging, while buttons 0 & 1 control
  // joints directly.
  void jointStatesCB(const sensor_msgs::JointStateConstPtr& msg)
  {
    //     pthread_mutex_lock(&q_meas_mutex_);
    q_meas_ = *msg;
    //     pthread_mutex_unlock(&q_meas_mutex_);
    ROS_DEBUG_STREAM("q_meas_: \n" << q_meas_ << "\n");
  }

  void dxDesCB(const geometry_msgs::TwistStampedPtr& msg)
  {
    if(enable_joy_)
    {
      dx_des_ = *msg;
      ROS_DEBUG_STREAM("dx_des_: \n" << dx_des_ << "\n");
    }
  }

  void gripperDesCB(const std_msgs::BoolPtr& msg)
  {
    if(enable_joy_)
    {
      gripper_des_ = *msg;
      ROS_DEBUG_STREAM("gripper_des_: \n" << dx_des_ << "\n");
    }
  }

  bool joystickEnableCB(niryo_one_msgs::SetInt::Request &req, niryo_one_msgs::SetInt::Response &res)
  {
    std::string result_message = "";
    int result = 0;
    if(req.value == 1)
    {
      if(can_enable())
      {
          enable_joy();
          result = 200;
          result_message = "Joystick has been enabled";
      }
      else
      {
        result = 400;
        result_message = "Wait for the end of command to enable Joystick";
      }
    }
    else
    {
      disable_joy();
      result = 200;
      result_message = "Joystick has been disabled";
    }
    
    res.status = result;
    res.message = result_message;
    return true;
  };

  bool can_enable()
  {
    ros::service::waitForService("/niryo_one/commander/is_active");
    niryo_one_msgs::GetInt is_active;      
    ros::ServiceClient commander_active_client = n_.serviceClient<niryo_one_msgs::GetInt>("/niryo_one/commander/is_active");
    commander_active_client.call(is_active);
    return (is_active.response.value == 0);    
  };
  
  void enable_joy()
  {
    enable_joy_ = true;
  };
  void disable_joy()
  {
    // TODO handle current motion : wait the arm reach a stable state
    enable_joy_ = false;
  };
  
  // Scale the incoming desired velocity
  Eigen::VectorXd scaleCartesianCommand(const geometry_msgs::TwistStamped& command) const
  {
    Eigen::VectorXd result(6);

    result[0] = LINEAR_SCALE * command.twist.linear.x;
    result[1] = LINEAR_SCALE * command.twist.linear.y;
    result[2] = LINEAR_SCALE * command.twist.linear.z;
    result[3] = ROTATIONAL_SCALE * command.twist.angular.x;
    result[4] = ROTATIONAL_SCALE * command.twist.angular.y;
    result[5] = ROTATIONAL_SCALE * command.twist.angular.z;

    return result;
  }

  ros::NodeHandle n_;
  ros::Publisher command_pub_;
  ros::Publisher debug_pub_;
  ros::Subscriber joints_sub_;
  ros::Subscriber dx_des_sub_;
  ros::Subscriber gripper_des_sub_;

  ros::ServiceClient changeToolClient_;
  ros::ServiceClient openGripperClient_;
  ros::ServiceClient closeGripperClient_;

  NiryoClient ac;
  bool enable_joy_ = false;
  sensor_msgs::JointState q_meas_;
  geometry_msgs::TwistStamped dx_des_;
  std_msgs::Bool gripper_des_;
  bool prev_gripper_des_;

  bool start_flag = true;
  geometry_msgs::Pose initial_pose;

  qpOASES::SQProblem IK;

  double theta[6];

  double alpha_1, alpha_2, alpha_3, alpha_4, alpha_5, alpha_6;
  double beta_1, beta_2, beta_3, beta_4, beta_5, beta_6;
  double gamma_1, gamma_2, gamma_3, gamma_4, gamma_5, gamma_6;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joystick_cartesian_interface");

  JoystickCartesianInterface joystickcartesianinterface;

  return 0;
}
