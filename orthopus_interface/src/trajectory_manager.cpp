// TODO copyright
#include "ros/ros.h"

#include "orthopus_interface/trajectory_manager.h"

#include <eigen_conversions/eigen_msg.h>

// Eigen3
#include <Eigen/Dense>

// TF
#include <tf/tf.h>

namespace cartesian_controller
{
TrajectoryManager::TrajectoryManager()
{
  ROS_DEBUG_STREAM("TrajectoryManager constructor");
  /*
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
   }*/
}
}
