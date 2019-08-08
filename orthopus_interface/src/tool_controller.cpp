// TODO copyright
#include "ros/ros.h"

#include "geometry_msgs/TwistStamped.h"
#include "niryo_one_msgs/SetInt.h"
#include "std_msgs/Bool.h"

#include "orthopus_interface/tool_controller.h"
namespace cartesian_controller
{
ToolController::ToolController() : ac_("/niryo_one/commander/robot_action/", true)
{
    ROS_INFO("Waiting for action server to start.");

    tool_id_ = 0;
    
    // wait for the action server to start
    bool connection_success = false;
    while (!connection_success)
    {
      connection_success = ac_.waitForServer(ros::Duration(3.0));
      if (connection_success)
      {
        ROS_INFO("  Robot Connection established");
      }
      else
      {
        ROS_WARN("  Error connecting to Robot. Trying again");
      }
    }
}
void ToolController::setToolId(int id_)
{
  tool_id_ = id_;
}

void ToolController::sendOpenGripperCommand()
{
  ROS_INFO("Open gripper");
  niryo_one_msgs::ToolCommand tcmd;
  niryo_one_msgs::RobotMoveActionGoal action;
  tcmd.cmd_type = 1;
  tcmd.gripper_open_speed = 100;
  tcmd.tool_id = tool_id_;
  action.goal.cmd.cmd_type = 6;
  action.goal.cmd.tool_cmd = tcmd;
  ac_.sendGoal(action.goal);
  ac_.waitForResult(ros::Duration(10.0));
}

void ToolController::sendCloseGripperCommand()
{
  ROS_INFO("Close gripper");
  niryo_one_msgs::ToolCommand tcmd;
  niryo_one_msgs::RobotMoveActionGoal action;
  tcmd.cmd_type = 2;
  tcmd.gripper_open_speed = 100;
  tcmd.tool_id = tool_id_;
  action.goal.cmd.cmd_type = 6;
  action.goal.cmd.tool_cmd = tcmd;
  ac_.sendGoal(action.goal);
  ac_.waitForResult(ros::Duration(10.0));
}

void ToolController::sendGripperCommand(bool new_state)
{
  if (tool_state_ != new_state)
  {
    if (!new_state)
    {
      sendOpenGripperCommand();
    }
    else
    {
      sendCloseGripperCommand();
    }
  }
  tool_state_ = new_state;
}


}
