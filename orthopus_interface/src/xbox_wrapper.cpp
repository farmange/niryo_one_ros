// TODO copyright
#include "ros/ros.h"

#include "orthopus_interface/xbox_wrapper.h"

// http://wiki.ros.org/joy
// Microsoft Xbox 360 Wired Controller for Linux
//
// Table of index number of /joy.buttons:
// 0 : A
// 1 : B
// 2 : X
// 3 : Y
// 4 : LB
// 5 : RB
// 6 : back
// 7 : start
// 8 : power
// 9 : Button stick left
// 10 : Button stick right
//
// Table of index number of /joy.axes:
// 0 : Left/Right Axis stick left
// 1 : Up/Down Axis stick left
// 2 : LT
// 3 : Left/Right Axis stick right
// 4 : Up/Down Axis stick right
// 5 : RT
// 6 : cross key left/right
// 7 : cross key up/down

#define JOY_BUTTON_A 0
#define JOY_BUTTON_B 1
#define JOY_BUTTON_X 2
#define JOY_BUTTON_Y 3
#define JOY_BUTTON_LB 4
#define JOY_BUTTON_RB 5
#define JOY_BUTTON_BACK 6
#define JOY_BUTTON_START 7
#define JOY_BUTTON_POWER 8
#define JOY_BUTTON_STICK_LEFT 9
#define JOY_BUTTON_STICK_RIGHT 10

#define JOY_AXIS_HORIZONTAL_LEFT 0
#define JOY_AXIS_VERTICAL_LEFT 1
#define JOY_LT 2
#define JOY_AXIS_HORIZONTAL_RIGHT 3
#define JOY_AXIS_VERTICAL_RIGHT 4
#define JOY_RT 5
#define JOY_CROSS_HORIZONTAL 6
#define JOY_CROSS_VERTICAL 7

#define JOY_DEBOUNCE_BUTTON_TIME 0.4
#define JOY_VELOCITY_FACTOR_INC 0.02

namespace cartesian_controller
{
XboxWrapper::XboxWrapper()
{
  device_sub_ = n_.subscribe("joy", 1, &XboxWrapper::joyCallback, this);

  debounce_button_a_ = ros::Time::now();
  debounce_button_start_ = ros::Time::now();
  debounce_button_lb_ = ros::Time::now();
  debounce_button_rb_ = ros::Time::now();
  debounce_button_b_ = ros::Time::now();

  button_a_ = 0;
  button_start_ = 0;
  button_lb_ = 0;
  button_rb_ = 0;
  button_b_ = 0;

  learning_mode_ = 0;
  cartesian_mode_ = YZ;
  velocity_factor_ = 0.0;
  ros::spin();
}
// TODO Put publish action on parent class
// Convert incoming xbox (joy) commands in require topics
void XboxWrapper::joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  processButtons(msg);
  updateGripperCmd();
  updateVelocityFactor();
  updateLearningMode();
  updateCartesianMode();

  // Cartesian control with the axes
  geometry_msgs::TwistStamped cartesian_vel;
  cartesian_vel.header.stamp = ros::Time::now();
  // Up/Down Axis of the right stick
  cartesian_vel.twist.linear.x = velocity_factor_ * msg->axes[JOY_AXIS_VERTICAL_RIGHT];
  // Left/Right Axis of the left stick
  cartesian_vel.twist.linear.y = velocity_factor_ * msg->axes[JOY_AXIS_HORIZONTAL_LEFT];
  // Up/Down Axis of the left stick
  cartesian_vel.twist.linear.z = velocity_factor_ * msg->axes[JOY_AXIS_VERTICAL_LEFT];

  std_msgs::Int8 cartesian_mode;
  cartesian_mode.data = cartesian_mode_;
  cartesian_cmd_pub_.publish(cartesian_vel);
  gripper_cmd_pub_.publish(gripper_cmd_);
  cartesian_mode_pub_.publish(cartesian_mode);
}

void XboxWrapper::processButtons(const sensor_msgs::Joy::ConstPtr& msg)
{
  button_a_ = 0;
  button_start_ = 0;
  button_lb_ = 0;
  button_rb_ = 0;
  button_b_ = 0;

  debounceButtons(msg, JOY_BUTTON_A, debounce_button_a_, button_a_);
  debounceButtons(msg, JOY_BUTTON_START, debounce_button_start_, button_start_);
  debounceButtons(msg, JOY_BUTTON_LB, debounce_button_lb_, button_lb_);
  debounceButtons(msg, JOY_BUTTON_RB, debounce_button_rb_, button_rb_);
  debounceButtons(msg, JOY_BUTTON_B, debounce_button_b_, button_b_);
}

void XboxWrapper::debounceButtons(const sensor_msgs::Joy::ConstPtr& msg, const int button_id,
                                  ros::Time& debounce_timer_ptr, int& button_value_ptr)
{
  if (msg->buttons[button_id])
  {
    if (ros::Time::now() > debounce_timer_ptr)
    {
      debounce_timer_ptr = ros::Time::now() + ros::Duration(JOY_DEBOUNCE_BUTTON_TIME);
      button_value_ptr = msg->buttons[button_id];
    }
  }
}

void XboxWrapper::updateCartesianMode()
{
  // Use B to change cartesian mode
  if (button_b_ == 1 && cartesian_mode_ == XY)
  {
    cartesian_mode_ = YZ;
  }
  else if (button_b_ == 1 && cartesian_mode_ == YZ)
  {
    cartesian_mode_ = XZ;
  }
  else if (button_b_ == 1 && cartesian_mode_ == XZ)
  {
    cartesian_mode_ = XY;
  }
}

void XboxWrapper::updateVelocityFactor()
{
  if (button_rb_)
  {
    velocity_factor_ += JOY_VELOCITY_FACTOR_INC;
  }
  if (button_lb_)
  {
    velocity_factor_ -= JOY_VELOCITY_FACTOR_INC;
  }
  velocity_factor_ = std::min(velocity_factor_, 1.0);
  velocity_factor_ = std::max(velocity_factor_, 0.0);
}

void XboxWrapper::updateGripperCmd()
{
  // Use A to toggle gripper state (open/close)
  if (button_a_ == 1 && gripper_cmd_.data == false)
  {
    gripper_cmd_.data = true;
  }
  else if (button_a_ == 1 && gripper_cmd_.data == true)
  {
    gripper_cmd_.data = false;
  }
}

void XboxWrapper::updateLearningMode()
{
  // Use START to toggle learning mode state (open/close)
  if (button_start_ == 1 && learning_mode_ == 0)
  {
    learning_mode_ = 1;
    requestLearningMode(learning_mode_);
  }
  else if (button_start_ == 1 && learning_mode_ == 1)
  {
    learning_mode_ = 0;
    requestLearningMode(learning_mode_);
  }
}
}

using namespace cartesian_controller;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "xbox_wrapper");

  XboxWrapper xbox_wrapper;

  return 0;
}
