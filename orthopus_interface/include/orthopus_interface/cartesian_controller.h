/*
  TODO copyright
*/

#ifndef CARTESIAN_CONTROLLER_CARTESIAN_CONTROLLER_H
#define CARTESIAN_CONTROLLER_CARTESIAN_CONTROLLER_H

#include <ros/ros.h>

// Messages
#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"

#include <niryo_one_msgs/SetInt.h>

#include "orthopus_interface/inverse_kinematic.h"
#include "orthopus_interface/tool_controller.h"

namespace cartesian_controller
{
class CartesianController
{
public:
  CartesianController();

protected:
private:
  void send6DofCommand();
  void updateFsm();

  // Callbacks
  void jointStatesCB(const sensor_msgs::JointStateConstPtr& msg);
  void dxDesCB(const geometry_msgs::TwistStampedPtr& msg);
  void gripperCB(const std_msgs::BoolPtr& msg);
  void learningModeCB(const std_msgs::BoolPtr& msg);
  bool enableCB(niryo_one_msgs::SetInt::Request& req, niryo_one_msgs::SetInt::Response& res);
  bool actionCB(niryo_one_msgs::SetInt::Request& req, niryo_one_msgs::SetInt::Response& res);
  bool can_enable();
  void enable_joy();
  void disable_joy();

  Vector6d scaleCartesianCommand(const geometry_msgs::TwistStamped& command) const;

  bool enable_joy_;

  ros::NodeHandle n_;
  ros::Publisher command_pub_;
  ros::Publisher joystick_enabled_pub_;
  ros::Publisher debug_pub_;
  ros::Publisher debug_des_pub_;
  ros::Subscriber joints_sub_;
  ros::Subscriber dx_des_sub_;
  ros::Subscriber gripper_sub_;
  ros::Subscriber learning_mode_sub_;

  InverseKinematic ik_;
  ToolController tool_controller_;

  double joint_position_cmd[6];
  //   sensor_msgs::JointState q_meas_forced, q_meas_;
  geometry_msgs::TwistStamped dx_des_;

  sensor_msgs::JointState current_joint_state;
  double cartesian_velocity_desired[7];
  double cartesian_velocity_desired_prev[7];
  bool gripper_state_;

  class FsmState
  {
  public:
    enum FsmStateEnum
    {
      Disable = 0,
      GotoHome,
      GotoRest,
      CartesianMode,
      Idle
    };

    FsmState() = default;
    const std::string ToString() const
    {
      std::string msg;
      switch (state)
      {
        case CartesianMode:
          msg = "CartesianMode";
          break;
        case GotoHome:
          msg = "GotoHome";
          break;
        case GotoRest:
          msg = "GotoRest";
          break;
        case Disable:
          msg = "Disable";
          break;
        case Idle:
          msg = "Idle";
          break;
        default:
          msg = "NaN";
          break;
      }
      return msg;
    }

    FsmState& operator=(const FsmStateEnum s)
    {
      state = s;
    };
    friend bool operator==(const FsmState& s, const FsmStateEnum& e)
    {
      return s.state == e;
    };
    friend bool operator!=(const FsmState& s, const FsmStateEnum& e)
    {
      return s.state != e;
    };
    friend bool operator==(const FsmState& s1, const FsmState& s2)
    {
      return s1.state == s2.state;
    };
    friend bool operator!=(const FsmState& s1, const FsmState& s2)
    {
      return s1.state != s2.state;
    };

  private:
    FsmStateEnum state;
  };

  class FsmAction
  {
  public:
    enum FsmActionEnum
    {
      None = 0,
      Cartesian,
      GotoHome,
      GotoRest
    };

    FsmAction() = default;
    const std::string ToString() const
    {
      std::string msg;
      switch (action)
      {
        case None:
          msg = "None";
          break;
        case Cartesian:
          msg = "Cartesian";
          break;
        case GotoHome:
          msg = "GotoHome";
          break;
        case GotoRest:
          msg = "GotoRest";
          break;
        default:
          msg = "NaN";
          break;
      }
      return msg;
    }
    FsmAction& operator=(const FsmActionEnum a)
    {
      action = a;
    }
    friend bool operator==(const FsmAction& a, const FsmActionEnum& e)
    {
      return a.action == e;
    };
    friend bool operator!=(const FsmAction& a, const FsmActionEnum& e)
    {
      return a.action == e;
    };

  private:
    FsmActionEnum action;
  };

  FsmAction action_requested;
  FsmState fsm_state;
  FsmState fsm_prev_state;

  int learning_mode;

  void printFsm();
  void runFsm();
  void cartesianState();
  void gotoHomeState();
  void gotoRestState();
  void gotoPosition(const double (&positionsDesired)[6]);
  double computeDuration(const double (&positionsDesired)[6]);
};
}
#endif
