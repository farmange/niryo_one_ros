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
    prev_xyz_rpy_[i] = 0.0;
    desired_xyz_rpy_[i] = 0.0;
    constraint_axis_[i] = true;
    constraint_axis_prev_[i] = false;


    
    velocity_command_[i] = 0.0;
    traj_desired_pose_[i] = 0.0;
  }
  traj_position_tolerance_ = 0.01;   // 10 mm
  traj_orientation_tolerance_ = 0.01;  // 0.01 rad = 0.573 deg
  
  trajectory_ctrl_p_gain_ = 0.0;
  trajectory_ctrl_i_gain_ = 0.0;
  constraint_ctrl_p_gain_ = 0.0;
  constraint_ctrl_i_gain_ = 0.0;
}

void PositionCompensator::init(int sampling_freq, ros::Publisher& debug_pose_current,
                               ros::Publisher& debug_pose_desired, ros::Publisher& debug_pose_meas, double max_vel)
{
  debug_pose_current_ = debug_pose_current;
  debug_pose_desired_ = debug_pose_desired;
  debug_pose_meas_ = debug_pose_meas;
  sampling_freq_ = sampling_freq;
  
  ros::param::get("~cartesian_max_vel", cartesian_max_vel_);
  pi_max_ = cartesian_max_vel_;
  pi_min_ = -cartesian_max_vel_;

  if (sampling_freq == 0)
  {
    ROS_ERROR("Invalid sampling frequency. Cannot be zero !");
    sampling_period_ = -1.0;
    return;
  }
  sampling_period_ = 1.0 / sampling_freq_;
  traj_requested_ = false;
}

void PositionCompensator::reset()
{
  ros::param::get("~trajectory_ctrl_p_gain", trajectory_ctrl_p_gain_);
  ros::param::get("~trajectory_ctrl_i_gain", trajectory_ctrl_i_gain_);
  ros::param::get("~constraint_ctrl_p_gain", constraint_ctrl_p_gain_);
  ros::param::get("~constraint_ctrl_i_gain", constraint_ctrl_i_gain_);
  
  for (int i = 0; i < 6; i++)
  {
    current_xyz_rpy_[i] = 0.0;
    prev_xyz_rpy_[i] = 0.0;
    desired_xyz_rpy_[i] = 0.0;
    constraint_axis_[i] = true;
    constraint_axis_prev_[i] = false;
        
    constraint_ctrl_i_sum_[i] = 0.0;
    trajectory_ctrl_i_sum_[i] = 0.0;

    euler_factor_[i] = 1.0;
    
    velocity_command_[i] = 0.0;
    traj_desired_pose_[i] = 0.0;
    
  }
  traj_requested_ = false;
  
}

void PositionCompensator::setJointState(sensor_msgs::JointState& current_joint_state)
{
  current_joint_state_ = current_joint_state;
  geometry_msgs::Pose current_pose;

  /* Set kinemtic state of the robot to the previous joint positions computed */
  kinematic_state->setVariableValues(current_joint_state_);
  kinematic_state->updateLinkTransforms();

  /* Get the cartesian state of the tool_link frame */
  const Eigen::Affine3d& end_effector_state = kinematic_state->getGlobalLinkTransform(
      kinematic_state->getLinkModel("tool_link"));  // TODO Add configuration parameter

  /* Convert cartesian state to goemetry_msgs::Pose */
  tf::poseEigenToMsg(end_effector_state, current_pose);

  /* Convert quaternion pose in RPY */
  tf::Quaternion q(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z,
                   current_pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  euler_factor_[3] = 1.0;
  euler_factor_[4] = 1.0;
  euler_factor_[5] = 1.0;
  if (std::abs(roll) > M_PI / 2)
  {
    //     pitch = -pitch;
    //     yaw = -yaw;
    euler_factor_[4] = -1.0;
    euler_factor_[5] = -1.0;
  }
  if (std::abs(pitch) > M_PI / 2)
  {
    //     roll = -roll;
    //     yaw = -yaw;
    euler_factor_[3] = -1.0;
    euler_factor_[5] = -1.0;
  }
  if (std::abs(yaw) > M_PI / 2)
  {
    //     roll = -roll;
    //     pitch = -pitch;
    euler_factor_[3] = -1.0;
    euler_factor_[4] = -1.0;
  }

  /* Store previous cartesion position */
  for (int i = 0; i < 6; i++)
  {
    prev_xyz_rpy_[i] = current_xyz_rpy_[i];
  }
  
  /* Store RPY pose in x_cmd_prev vector */
  current_xyz_rpy_[0] = current_pose.position.x;
  current_xyz_rpy_[1] = current_pose.position.y;
  current_xyz_rpy_[2] = current_pose.position.z;
  current_xyz_rpy_[3] = roll;
  current_xyz_rpy_[4] = pitch;
  current_xyz_rpy_[5] = yaw;
}

void PositionCompensator::setVelocitiesCommand(const double (&cartesian_velocity_desired)[6])
{
  for (int i = 0; i < 6; i++)
  {
    velocity_command_[i] = cartesian_velocity_desired[i];
  }
}

void PositionCompensator::setTrajectoryPose(const geometry_msgs::Pose& traj_des_pose)
{
  traj_requested_ = true;
  
  /* Here I suppose that user is passing RPY command in pose */  
  traj_desired_pose_[0] = traj_des_pose.position.x;
  traj_desired_pose_[1] = traj_des_pose.position.y;
  traj_desired_pose_[2] = traj_des_pose.position.z;
  traj_desired_pose_[3] = traj_des_pose.orientation.x;
  traj_desired_pose_[4] = traj_des_pose.orientation.y;
  traj_desired_pose_[5] = traj_des_pose.orientation.z;
}

void PositionCompensator::activeConstraints(double (&dx_response)[6], const double (&dx_request)[6], const double (&x_request)[6], bool force_xyz_constraints)
{
  updateContraints(dx_request);
  updateDesiredPosition();
  
  for (int i = 0; i < 6; i++)
  {
    if (constraint_axis_[i] == true)
    {
      bool int_ok = true;
      double error = (desired_xyz_rpy_[i] - current_xyz_rpy_[i]) * euler_factor_[i];
      /* Proportional term */
      double proportional_out = constraint_ctrl_p_gain_ * error;
      /* Integral term */
      double constraint_ctrl_i_sum_temp_ = constraint_ctrl_i_sum_[i] + (error * sampling_period_);
      double integral_out = constraint_ctrl_i_gain_ * constraint_ctrl_i_sum_temp_;
      /* PI */
      double pi_out = proportional_out + integral_out;

      // Restrict to max/min
      if (pi_out > pi_max_)
      {
        pi_out = pi_max_;

        /* Error is the same sign? Inhibit integration. */
        if (error > 0)
        {
          int_ok = false;
        }
      }
      else if (pi_out < pi_min_)
      {
        pi_out = pi_min_;

        /* Error is the same sign? Inhibit integration. */
        if (error < 0)
        {
          int_ok = false;
        }
      }

      /* Update the integrator if allowed. */
      if (int_ok)
      {
        constraint_ctrl_i_sum_[i] = constraint_ctrl_i_sum_temp_;
      }

      dx_response[i] = pi_out;
    }
    else
    {
      /* Keep the value set by the user */
      dx_response[i] = dx_request[i];
      constraint_ctrl_i_sum_[i] = 0.0;
    }
  }

  /* Publish debug topics */
  geometry_msgs::Pose current_pose_debug;
  current_pose_debug.position.x = current_xyz_rpy_[0];
  current_pose_debug.position.y = current_xyz_rpy_[1];
  current_pose_debug.position.z = current_xyz_rpy_[2];
  current_pose_debug.orientation.x = current_xyz_rpy_[3];
  current_pose_debug.orientation.y = current_xyz_rpy_[4];
  current_pose_debug.orientation.z = current_xyz_rpy_[5];
  debug_pose_current_.publish(current_pose_debug);

  geometry_msgs::Pose desired_pose;
  desired_pose.position.x = traj_desired_pose_[0];
  desired_pose.position.y = traj_desired_pose_[1];
  desired_pose.position.z = traj_desired_pose_[2];
  desired_pose.orientation.x = traj_desired_pose_[3];
  desired_pose.orientation.y = traj_desired_pose_[4];
  desired_pose.orientation.z = traj_desired_pose_[5];
  debug_pose_desired_.publish(desired_pose);

   geometry_msgs::Pose meas_pose;
   meas_pose.position.x = constraint_axis_[0];
   meas_pose.position.y = constraint_axis_[1];
   meas_pose.position.z = constraint_axis_[2];
   meas_pose.orientation.x = constraint_axis_[3];
   meas_pose.orientation.y = constraint_axis_[4];
   meas_pose.orientation.z = constraint_axis_[5];
   debug_pose_meas_.publish(meas_pose);
} 

void PositionCompensator::executeTrajectory(double (&dx_response)[6], const double (&x_request)[6])
{  
  for (int i = 0; i < 6; i++)
  {
    bool int_ok = true;
    double error = (traj_desired_pose_[i] - current_xyz_rpy_[i]) * euler_factor_[i];
    /* Proportional term */
    double proportional_out = trajectory_ctrl_p_gain_ * error;
    /* Integral term */
    double constraint_ctrl_i_sum_temp_ = trajectory_ctrl_i_sum_[i] + (error * sampling_period_);
    double integral_out = trajectory_ctrl_i_gain_ * constraint_ctrl_i_sum_temp_;
    /* PI */
    double pi_out = proportional_out + integral_out;

    // Restrict to max/min
    if (pi_out > pi_max_)
    {
      pi_out = pi_max_;

      /* Error is the same sign? Inhibit integration. */
      if (error > 0)
      {
        int_ok = false;
      }
    }
    else if (pi_out < pi_min_)
    {
      pi_out = pi_min_;

      /* Error is the same sign? Inhibit integration. */
      if (error < 0)
      {
        int_ok = false;
      }
    }

    /* Update the integrator if allowed. */
    if (int_ok)
    {
      trajectory_ctrl_i_sum_[i] = constraint_ctrl_i_sum_temp_;
    }
    else
    {
      ROS_ERROR("ANTi WINDUP !!!!!" );
    }
    ROS_ERROR("sum = %5f, pi_out = %5f proportional_out = %5f, integral_out = %5f ", trajectory_ctrl_i_sum_[i], pi_out, proportional_out, integral_out);
    
    dx_response[i] = pi_out;
  }
  
//   /* Publish debug topics */
//   geometry_msgs::Pose current_pose_debug;
//   current_pose_debug.position.x = current_xyz_rpy_[0];
//   current_pose_debug.position.y = current_xyz_rpy_[1];
//   current_pose_debug.position.z = current_xyz_rpy_[2];
//   current_pose_debug.orientation.x = current_xyz_rpy_[3];
//   current_pose_debug.orientation.y = current_xyz_rpy_[4];
//   current_pose_debug.orientation.z = current_xyz_rpy_[5];
//   debug_pose_current_.publish(current_pose_debug);
//   
//   geometry_msgs::Pose desired_pose;
//   desired_pose.position.x = traj_desired_pose_[0];
//   desired_pose.position.y = traj_desired_pose_[1];
//   desired_pose.position.z = traj_desired_pose_[2];
//   desired_pose.orientation.x = traj_desired_pose_[3];
//   desired_pose.orientation.y = traj_desired_pose_[4];
//   desired_pose.orientation.z = traj_desired_pose_[5];
//   debug_pose_desired_.publish(desired_pose);
//   
  geometry_msgs::Pose meas_pose;
  meas_pose.position.x = dx_response[0];
  meas_pose.position.y = dx_response[1];
  meas_pose.position.z = dx_response[2];
  meas_pose.orientation.x = dx_response[3];
  meas_pose.orientation.y = dx_response[4];
  meas_pose.orientation.z = dx_response[5];
  debug_pose_meas_.publish(meas_pose);
}

void PositionCompensator::run(double (&dx_response)[6])
{
  rolloverHandling();
  if(isTrajectoryRequested())
  {
    ROS_ERROR("A trajectory is requested...");
    executeTrajectory(dx_trajectory_, traj_desired_pose_);
    activeConstraints(dx_response, dx_trajectory_, desired_xyz_rpy_, true);
  }
  else
  {  
    activeConstraints(dx_response, velocity_command_, desired_xyz_rpy_, false);
  }
}

bool PositionCompensator::isTrajectoryRequested()
{
  if (traj_requested_)
  {
    if (trajectoryIsAchieved())
    {
      traj_requested_ = false;
    }
  }
  return traj_requested_;
}

bool PositionCompensator::isTrajectoryCompleted()
{
  return !traj_requested_;
}

bool PositionCompensator::trajectoryIsAchieved()
{
  bool is_achieve = true;
  for (int i = 0; i < 3; i++)
  {
    if (std::abs(traj_desired_pose_[i] - current_xyz_rpy_[i]) > traj_position_tolerance_)
    {
      is_achieve = false;
      ROS_ERROR("delta of %5f on %d axis", traj_desired_pose_[i] - current_xyz_rpy_[i], i);
    }
    
  }
  for (int i = 3; i < 6; i++)
  {
    if (std::abs(traj_desired_pose_[i] - current_xyz_rpy_[i]) > traj_orientation_tolerance_)
    {
      is_achieve = false;
      ROS_ERROR("delta of %5f on %d axis", traj_desired_pose_[i] - current_xyz_rpy_[i], i);
    }
  }
  return is_achieve;
}

void PositionCompensator::updateContraints(const double (&dx_des)[6])
{
  for (int i = 0; i < 6; i++)
  {    
    if (dx_des[i] == 0.0)
    {
      constraint_axis_[i] = true;
    }
    else
    {
      constraint_axis_[i] = false;
    }
  }  
}

void PositionCompensator::rolloverHandling()
{
  /* As we only work in the same side (negative yaw), we ensure no flipping happened */
  if(current_xyz_rpy_[YAW_AXIS] > M_PI / 2)
  {
    current_xyz_rpy_[YAW_AXIS] -= 2 * M_PI;
  }    
}
void PositionCompensator::updateDesiredPosition()
{
  /* In case no command is set on an axis, we store the current position as a new setpoint (no motion wanted) */
  for (int i = 0; i < 6; i++)
  {
    /* Detect newly freed position */
    if (constraint_axis_[i] == true && constraint_axis_prev_[i] == false)
    {
      desired_xyz_rpy_[i] = current_xyz_rpy_[i];
    }
    constraint_axis_prev_[i] = constraint_axis_[i];
  }
}

}
