#include "geometry_msgs/TwistStamped.h"
//#include "control_msgs/JointJog.h"
#include "ros/ros.h"
#include <ros/callback_queue.h>

   
#include "sensor_msgs/Joy.h"
#include <control_msgs/JointJog.h>
#include <sensor_msgs/JointState.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>


namespace planar_robot_play
{
static const int NUM_SPINNERS = 2;
static const int JOINT_SUB_QUEUE_LENGTH = 1;
static const int X_DOT_DES_QUEUE_LENGTH = 1;
static const double LINEAR_SCALE = 1;
static const double ROTATIONAL_SCALE = 1;
static const double CALC_PERIOD = 0.1;

class Jacobian2Dof
{
public:
  Jacobian2Dof() : spinner_(NUM_SPINNERS)
  {
    pthread_mutex_init(&q_meas_mutex_, nullptr);
    pthread_mutex_init(&x_dot_des_mutex_, nullptr);

    q_dot_des_pub_ = n_.advertise<control_msgs::JointJog>("q_dot_des", JOINT_SUB_QUEUE_LENGTH);
    joints_sub_ = n_.subscribe("joint_states", JOINT_SUB_QUEUE_LENGTH, &Jacobian2Dof::jointsCB, this);
    x_det_des_sub_ = n_.subscribe("x_dot_des", X_DOT_DES_QUEUE_LENGTH, &Jacobian2Dof::xDotDesCallback, this);
    
    spinner_.start();
      
    ros::Duration(10).sleep();

    // Wait for initial messages
    ROS_INFO("Jacobian2Dof: Waiting for first joint msg.");
    ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states");
    ROS_INFO("Jacobian2Dof: Received first joint msg.");
    
    // Wait for initial messages
    ROS_INFO("Jacobian2Dof: Waiting for velocity command msg.");
    ros::topic::waitForMessage<geometry_msgs::TwistStamped>("x_dot_des");
    ROS_INFO("Jacobian2Dof: Received velocity command msg.");
    
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
    
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm_group");

    
    ros::Rate loop_rate(1./CALC_PERIOD);

    ROS_INFO("Let's go !");

      // Now do jogging calcs
    while (ros::ok())
    {     
      // Update kinematic state according to current joint position
      pthread_mutex_lock(&q_meas_mutex_);
      kinematic_state->setVariableValues(q_meas_);
      pthread_mutex_unlock(&q_meas_mutex_);
      
      kinematic_state->setVariableValues(q_meas_);

      const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
      std::vector<double> joint_values;
      kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
      for (std::size_t i = 0; i < joint_names.size(); ++i)
      {
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
      }

      Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
      Eigen::MatrixXd jacobian;
      kinematic_state->getJacobian(joint_model_group,
                              kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                              reference_point_position, jacobian);
      jacobian(5, 0) = 0.0;
      jacobian(5, 1) = 0.0;
      ROS_INFO_STREAM("Jacobian: \n" << jacobian << "\n");
      
      // Get the cartesian desired velocity
      pthread_mutex_lock(&x_dot_des_mutex_);
      const Eigen::VectorXd x_dot_des_vect = scaleCartesianCommand(x_dot_des_);
      pthread_mutex_unlock(&x_dot_des_mutex_);

      ROS_INFO_STREAM("x_dot_des_vect: \n" << x_dot_des_vect << "\n");

      svd_ = Eigen::JacobiSVD<Eigen::MatrixXd>(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
      matrix_s_ = svd_.singularValues().asDiagonal();
      pseudo_inverse_ = svd_.matrixV() * matrix_s_.inverse() * svd_.matrixU().transpose();
      q_dot_des_calc = pseudo_inverse_ * x_dot_des_vect;
      ROS_INFO_STREAM("Pseudo inverse: \n" << pseudo_inverse_ << "\n");
      ROS_INFO_STREAM("q_dot_des_calc: \n" << q_dot_des_calc << "\n");

      control_msgs::JointJog q_dot_des_command;
      q_dot_des_command.joint_names = joint_names;
      std::vector<double> q_dot_des;
      q_dot_des.resize(q_dot_des_calc.size());
      Eigen::VectorXd::Map(&q_dot_des[0], q_dot_des_calc.size()) = q_dot_des_calc;
      
      q_dot_des_command.velocities = q_dot_des;
      q_dot_des_command.header.stamp = ros::Time::now();
    
    
      q_dot_des_pub_.publish(q_dot_des_command);
      
      loop_rate.sleep();
    }
    
    ros::waitForShutdown();
  };

private:
  // Convert incoming joy commands to TwistStamped commands for jogging.
  // The TwistStamped component goes to jogging, while buttons 0 & 1 control
  // joints directly.
  void jointsCB(const sensor_msgs::JointStateConstPtr& msg)
  {
    pthread_mutex_lock(&q_meas_mutex_);
    q_meas_ = *msg;
    pthread_mutex_unlock(&q_meas_mutex_);
    ROS_DEBUG_STREAM("q_meas_: \n" << q_meas_ << "\n");
  }
  
  void xDotDesCallback(const geometry_msgs::TwistStampedPtr& msg)
  {
    pthread_mutex_lock(&x_dot_des_mutex_);
    x_dot_des_ = *msg; // why use pointer here ? Could be dangerous ?
    pthread_mutex_unlock(&x_dot_des_mutex_);
    ROS_DEBUG_STREAM("x_dot_des_: \n" << x_dot_des_ << "\n");
  }
  
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
  ros::Publisher q_dot_des_pub_;
  ros::Subscriber joints_sub_;
  ros::Subscriber x_det_des_sub_;
  ros::AsyncSpinner spinner_;
  
  sensor_msgs::JointState q_meas_;
  pthread_mutex_t q_meas_mutex_;
    
  geometry_msgs::TwistStamped x_dot_des_;
  pthread_mutex_t x_dot_des_mutex_;

  geometry_msgs::TwistStamped twist_cmd_;

      
  // For jacobian calculations
  Eigen::MatrixXd pseudo_inverse_, matrix_s_;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd_;
  Eigen::VectorXd q_dot_des_calc;
  
};
}  // namespace planar_robot_play

int main(int argc, char** argv)
{
  ros::init(argc, argv, "jacobian2dof");

  planar_robot_play::Jacobian2Dof jacobian2dof;

  return 0;
}
