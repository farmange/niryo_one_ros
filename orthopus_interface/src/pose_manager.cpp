// TODO copyright
#include "ros/ros.h"

#include "orthopus_interface/pose_manager.h"

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <eigen_conversions/eigen_msg.h>

namespace cartesian_controller
{
PoseManager::PoseManager() 
{
  std::vector<double> pose;
  ros::param::get("~home_position", pose);
  position_map_["Home"] = pose;
  ros::param::get("~rest_position", pose);
  position_map_["Rest"] = pose;
  ros::param::get("~drink_position", pose);
  position_map_["Drink"] = pose;
  
}

const std::vector<double> PoseManager::getJoints(const std::string position_name)
{
  return position_map_[position_name];
}

const geometry_msgs::Pose PoseManager::getPose(const std::string position_name)
{
  robot_model::RobotModelPtr kinematic_model;
  robot_state::RobotStatePtr kinematic_state;
  geometry_msgs::Pose current_pose;
  
  sensor_msgs::JointState local_joint_state;
  local_joint_state.name.resize(6);
  local_joint_state.name[0] = "joint_1";
  local_joint_state.name[1] = "joint_2";
  local_joint_state.name[2] = "joint_3";
  local_joint_state.name[3] = "joint_4";
  local_joint_state.name[4] = "joint_5";
  local_joint_state.name[5] = "joint_6";
  
  local_joint_state.position.resize(6);
  local_joint_state.position[0] = position_map_[position_name][0];
  local_joint_state.position[1] = position_map_[position_name][1];
  local_joint_state.position[2] = position_map_[position_name][2];
  local_joint_state.position[3] = position_map_[position_name][3];
  local_joint_state.position[4] = position_map_[position_name][4];
  local_joint_state.position[5] = position_map_[position_name][5];
  

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  kinematic_model = robot_model_loader.getModel();
  kinematic_state = std::make_shared<robot_state::RobotState>(kinematic_model);
  
  kinematic_state->setVariableValues(local_joint_state);
  
  const Eigen::Affine3d& end_effector_state =
  kinematic_state->getGlobalLinkTransform("tool_link");  // TODO Add configuration parameter
  tf::poseEigenToMsg(end_effector_state, current_pose);
  return current_pose;
}

void PoseManager::setJoints(const std::string position_name, const std::vector<double> joint_values)
{
  position_map_[position_name] = joint_values;
}

bool PoseManager::callbackManagePose(niryo_one_msgs::ManagePosition::Request& req, niryo_one_msgs::ManagePosition::Response& res)
{
  if(req.cmd_type == 0)
  {
    /* Set position */
    if(req.position.joints.size() != 6)
    {
      res.message = "Error, could not set joint position (size != 6)";
      ROS_ERROR_STREAM("Error, could not set joint position (size != 6) ");
      return false;
    }
    res.message = "Set position " + req.position_name;
    position_map_[req.position_name] =  req.position.joints;
  }
  else if(req.cmd_type == 1)
  {
    std::vector<double> pose;
    pose.resize(6);
    // TODO get the current joint state
    position_map_[req.position_name] = pose;
  }
    
  return true;  
}

}
