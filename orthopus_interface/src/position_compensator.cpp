// TODO copyright
#include "ros/ros.h"

#include "orthopus_interface/position_compensator.h"

#include <eigen_conversions/eigen_msg.h>

// Eigen3
#include <Eigen/Dense>

// TF
#include <tf/tf.h>

namespace cartesian_controller
{
PositionCompensator::PositionCompensator()
{
  ROS_DEBUG_STREAM("PositionCompensator constructor");
 
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  kinematic_model = robot_model_loader.getModel();
  kinematic_state = std::make_shared<robot_state::RobotState>(kinematic_model);
  kinematic_state->setToDefaultValues();
  joint_model_group = kinematic_model->getJointModelGroup("arm");
  sampling_freq_ = 0;
  sampling_period_ = 0; 
  pi_max_ = 0.0;
  pi_min_ = 0.0;
  for (int i = 0; i < 6; i++)
  {
    current_xyz_rpy_[i] = 0.0;
    desired_xyz_rpy_[i] = 0.0;
    free_position_[i] = true;
    free_position_prev_[i] = false;
    proportional_gain_[i] = 10.0;
    integral_gain_[i] = 0.5;
  }
}

void PositionCompensator::init(int sampling_freq,
                               ros::Publisher& debug_pose_current,
                               ros::Publisher& debug_pose_desired,
                               ros::Publisher& debug_pose_meas,
                               double max_vel)
{
  debug_pose_current_ = debug_pose_current;
  debug_pose_desired_ = debug_pose_desired;  
  debug_pose_meas_ = debug_pose_meas;
  sampling_freq_ = sampling_freq;
  pi_max_ = max_vel;
  pi_min_ = -max_vel;
  
  if(sampling_freq == 0)
  {
    ROS_ERROR("Invalid sampling frequency. Cannot be zero !");
    sampling_period_ = -1.0;
    return;
  }
  sampling_period_ = 1.0 / sampling_freq_;
}

void PositionCompensator::reset()
{
  for (int i = 0; i < 6; i++)
  {
    current_xyz_rpy_[i] = 0.0;
    desired_xyz_rpy_[i] = 0.0;
    free_position_[i] = true;
    free_position_prev_[i] = false;
    proportional_gain_[i] = 10.0;
    integral_gain_[i] = 0.5;
  }
}

void PositionCompensator::run(double (&cartesian_velocity_compensated)[6], 
                              const double (&cartesian_velocity_desired)[6], 
                              sensor_msgs::JointState& current_joint_state)
{
  current_joint_state_ = current_joint_state;
  geometry_msgs::Pose current_pose;
  
  /* Set kinemtic state of the robot to the previous joint positions computed */
  kinematic_state->setVariableValues(current_joint_state_);
  kinematic_state->updateLinkTransforms();
  
  /* Get the cartesian state of the tool_link frame */
  const Eigen::Affine3d& end_effector_state = 
  kinematic_state->getGlobalLinkTransform(kinematic_state->getLinkModel("tool_link"));  // TODO Add configuration parameter  
  
  /* Convert cartesian state to goemetry_msgs::Pose */
  tf::poseEigenToMsg(end_effector_state, current_pose);

  /* Convert quaternion pose in RPY */
  tf::Quaternion q(current_pose.orientation.x, 
                   current_pose.orientation.y, 
                   current_pose.orientation.z, 
                   current_pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  
  if(std::abs(roll) > M_PI/2)
  {
    pitch = -pitch;
    yaw = -yaw;
  }
  if(std::abs(pitch) > M_PI/2)
  {
    roll = -roll;
    yaw = -yaw;
  }
  if(std::abs(yaw) > M_PI/2)
  {
    roll = -roll;
    pitch = -pitch;
  }
  
  /* Store RPY pose in x_cmd_prev vector */
  current_xyz_rpy_[0] = current_pose.position.x;
  current_xyz_rpy_[1] = current_pose.position.y;
  current_xyz_rpy_[2] = current_pose.position.z;
  current_xyz_rpy_[3] = roll;
  current_xyz_rpy_[4] = pitch;
  current_xyz_rpy_[5] = yaw;

  /* Look for euler jump in RPY and update constraints if needed */
  for(int i=3; i<6; i++)
  {
    if(std::abs(current_xyz_rpy_[i] - prev_xyz_rpy_[i]) >= M_PI/2)
    {
      ROS_ERROR_STREAM("==============> Euler jump detected on axis " << i << " !!!! delta=" << current_xyz_rpy_[i] - prev_xyz_rpy_[i] <<"");
      if(current_xyz_rpy_[i] - prev_xyz_rpy_[i] >= M_PI/2)
      {
        desired_xyz_rpy_[i] = desired_xyz_rpy_[i] + M_PI*2;
      }
      else if(current_xyz_rpy_[i] - prev_xyz_rpy_[i] <= M_PI/2)
      {
        desired_xyz_rpy_[i] = desired_xyz_rpy_[i] - M_PI*2;
      }
    }
  }
  
  detectFreePosition(cartesian_velocity_desired);
  updateDesiredPosition();
  
  for (int i = 0; i < 6; i++)
  {
    if(free_position_[i] == true)
    {
      bool int_ok = true;
      double error = (desired_xyz_rpy_[i] - current_xyz_rpy_[i]);
      /* Proportional term */
      double proportional_out = proportional_gain_[i] * error;
      /* Integral term */
      double integral_sum_temp_ = integral_sum_[i] + (error * sampling_period_);
      double integral_out = integral_gain_[i] * integral_sum_temp_;
      /* PI */
      double pi_out =  proportional_out + integral_out;
      
      // Restrict to max/min
      if( pi_out > pi_max_ )
      {
        pi_out = pi_max_;
        
        /* Error is the same sign? Inhibit integration. */
        if(error > 0)
        {
          int_ok = false;
        }
      }
      else if( pi_out < pi_min_ )
      {
        pi_out = pi_min_;
        
        /* Error is the same sign? Inhibit integration. */
        if(error < 0)
        {
          int_ok = false;
        }
      }
      
      /* Update the integrator if allowed. */
      if (int_ok)
      {
        integral_sum_[i] = integral_sum_temp_;
      }
      
      cartesian_velocity_compensated[i] = pi_out;
    }
    else
    {
      /* Keep the value set by the user */
      cartesian_velocity_compensated[i] = cartesian_velocity_desired[i];
    }
    free_position_prev_[i] = free_position_[i];
    prev_xyz_rpy_[i] = current_xyz_rpy_[i];
  }
  
  
  /* Publish debug topics */
  geometry_msgs::Pose current_pose_debug;
  current_pose_debug.position.x =  current_xyz_rpy_[0]; 
  current_pose_debug.position.y = current_xyz_rpy_[1]; 
  current_pose_debug.position.z = current_xyz_rpy_[2]; 
  current_pose_debug.orientation.x = current_xyz_rpy_[3]; 
  current_pose_debug.orientation.y = current_xyz_rpy_[4];  
  current_pose_debug.orientation.z = current_xyz_rpy_[5];  
  debug_pose_current_.publish(current_pose_debug);
  
  geometry_msgs::Pose desired_pose;
  desired_pose.position.x = desired_xyz_rpy_[0]; 
  desired_pose.position.y = desired_xyz_rpy_[1]; 
  desired_pose.position.z = desired_xyz_rpy_[2]; 
  desired_pose.orientation.x = desired_xyz_rpy_[3]; 
  desired_pose.orientation.y = desired_xyz_rpy_[4];  
  desired_pose.orientation.z = desired_xyz_rpy_[5]; 
  debug_pose_desired_.publish(desired_pose);  
  
  geometry_msgs::Pose meas_pose;
  meas_pose.position.x = free_position_[0]?1.0:0.0; 
  meas_pose.position.y = free_position_[1]?1.0:0.0; 
  meas_pose.position.z = free_position_[2]?1.0:0.0; 
  meas_pose.orientation.x = free_position_[3]?1.0:0.0; 
  meas_pose.orientation.y = free_position_[4]?1.0:0.0; 
  meas_pose.orientation.z = free_position_[5]?1.0:0.0; 
  debug_pose_meas_.publish(meas_pose);  
}

void PositionCompensator::detectFreePosition(const double (&cartesian_velocity_desired)[6])
{
  for (int i = 0; i < 6; i++)
  {
    if(cartesian_velocity_desired[i] == 0.0)
    {
      free_position_[i] = true;
    }
    else
    {
      free_position_[i] = false;
    }
  }
}

void PositionCompensator::updateDesiredPosition()
{
  for (int i = 0; i < 6; i++)
  {   
    /* Detect newly freed position */
    if(free_position_[i] == true && free_position_prev_[i] == false)
    {
      desired_xyz_rpy_[i] = current_xyz_rpy_[i];
    }
  }
}

}
