/*
  TODO copyright
*/

#ifndef CARTESIAN_CONTROLLER_CARTESIAN_CONTROLLER_H
#define CARTESIAN_CONTROLLER_CARTESIAN_CONTROLLER_H

#include <ros/ros.h>

// Messages
#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"

#include <niryo_one_msgs/SetInt.h>

#include "orthopus_interface/inverse_kinematic.h"
#include "orthopus_interface/pose_manager.h"
#include "orthopus_interface/position_compensator.h"
#include "orthopus_interface/trajectory_manager.h"

namespace cartesian_controller
{
class CartesianController
{
public:
  CartesianController();
  void init(int sampling_freq, PoseManager& pose_manager, ros::Publisher& command_pub,
            ros::Publisher& debug_pose_current, ros::Publisher& debug_pose_desired, ros::Publisher& debug_pose_meas,
            ros::Publisher& debug_joint_desired, ros::Publisher& debug_joint_min_limit,
            ros::Publisher& debug_joint_max_limit);
  void run();
  bool cartesianIsEnable();

  // Callbacks
  bool callbackCartesianEnable(niryo_one_msgs::SetInt::Request& req, niryo_one_msgs::SetInt::Response& res);
  bool callbackAction(niryo_one_msgs::SetInt::Request& req, niryo_one_msgs::SetInt::Response& res);
  void callbackJointState(const sensor_msgs::JointStateConstPtr& msg);
  void callbackMoveGroupState(const std_msgs::Int32Ptr& msg);
  void callbackVelocitiesDesired(const geometry_msgs::TwistStampedPtr& msg);
  void callbackLearningMode(const std_msgs::BoolPtr& msg);

protected:
private:
  void sendJointsCommand();
  void updateFsm();

  bool can_enable();
  void enable_joy();
  void disable_joy();

  bool enable_joy_;

  ros::NodeHandle n_;
  ros::Publisher command_pub_;
  ros::Publisher joystick_enabled_pub_;
  ros::Publisher debug_pose_current_;
  ros::Publisher debug_pose_desired_;
  ros::Publisher debug_pose_meas_;
  ros::Publisher debug_joint_desired_;
  ros::Publisher debug_joint_min_limit_;
  ros::Publisher debug_joint_max_limit_;

  ros::Subscriber joints_sub_;
  ros::Subscriber dx_des_sub_;
  ros::Subscriber learning_mode_sub_;

  InverseKinematic ik_;
  PoseManager pose_manager_;
  PositionCompensator position_compensator_;
  TrajectoryManager trajectory_manager_;

  int sampling_freq_;

  int move_group_state_;
  bool planning_pending_;
  double joint_position_cmd[6];
  //   sensor_msgs::JointState q_meas_forced, q_meas_;
  geometry_msgs::TwistStamped dx_des_;

  sensor_msgs::JointState current_joint_state;
  double cartesian_velocity_desired[6];
  double cartesian_velocity_traj[6];
  double cartesian_velocity_compensated[6];
  double cartesian_velocity_desired_prev[6];

  double pose_goal_joints_tolerance_;
  class FsmState
  {
  public:
    enum FsmStateEnum
    {
      Disable = 0,
      GotoHome,
      GotoRest,
      GotoDrink,
      GotoStandGlass,
      FlipPinch,
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
        case GotoDrink:
          msg = "GotoDrink";
          break;
        case GotoStandGlass:
          msg = "GotoStandGlass";
          break;
        case FlipPinch:
          msg = "FlipPinch";
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
      GotoRest,
      GotoDrink,
      GotoStandGlass,
      FlipPinch
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
        case GotoDrink:
          msg = "GotoDrink";
          break;
        case GotoStandGlass:
          msg = "GotoStandGlass";
          break;
        case FlipPinch:
          msg = "FlipPinch";
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
  void gotoDrinkState();
  void gotoStandGlassState();
  void flipPinchState();

  void gotoPosition(const std::vector<double> position);
  double computeDuration(const std::vector<double> position);
  bool isPositionCompleted(const std::vector<double> position);
};
}
#endif
