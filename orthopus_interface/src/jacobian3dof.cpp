#include "geometry_msgs/TwistStamped.h"
//#include "control_msgs/JointJog.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include <sensor_msgs/JointState.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

namespace planar_robot_play
{
static const int NUM_SPINNERS = 1;
static const int JOINT_SUB_QUEUE_LENGTH = 1;

class Jacobian3Dof
{
public:
  Jacobian3Dof() : spinner_(NUM_SPINNERS)
  {
    joints_sub_ = n_.subscribe("joint_states", JOINT_SUB_QUEUE_LENGTH, &Jacobian3Dof::jointsCB, this);

    spinner_.start();

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm_group");

    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

    Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
    Eigen::MatrixXd jacobian;
    kinematic_state->getJacobian(joint_model_group,
                                 kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                                 reference_point_position, jacobian);
    ROS_INFO_STREAM("Jacobian: \n" << jacobian << "\n");

    ros::waitForShutdown();
  };

private:
  // Convert incoming joy commands to TwistStamped commands for jogging.
  // The TwistStamped component goes to jogging, while buttons 0 & 1 control
  // joints directly.
  void jointsCB(const sensor_msgs::JointStateConstPtr& msg)
  {
    joints = *msg;
  }

  ros::NodeHandle n_;
  ros::Subscriber joints_sub_;
  ros::AsyncSpinner spinner_;
  sensor_msgs::JointState joints;
};
}  // namespace planar_robot_play

int main(int argc, char** argv)
{
  ros::init(argc, argv, "jacobian2dof");

  planar_robot_play::Jacobian3Dof jacobian3dof;

  return 0;
}
