// TODO copyright
#include "ros/ros.h"

#include "orthopus_interface/spacenav_wrapper.h"

#define JOY_BUTTON_LEFT 0
#define JOY_BUTTON_RIGHT 1

#define JOY_DEBOUNCE_BUTTON_TIME 0.4
#define JOY_VELOCITY_FACTOR_INC 0.02

namespace cartesian_controller
{
SpacenavWrapper::SpacenavWrapper()
{
  device_sub_ = n_.subscribe("spacenav/joy", 1, &SpacenavWrapper::joyCallback, this);

  debounce_button_left_ = ros::Time::now();
  debounce_button_right_ = ros::Time::now();

  button_left_ = 0;
  button_right_ = 0;

  learning_mode_ = 0;
  cartesian_mode_ = YZ;
  velocity_factor_ = 0.0;
  ros::spin();
}
// TODO Put publish action on parent class
// Convert incoming xbox (joy) commands in require topics
void SpacenavWrapper::joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  processButtons(msg);
  updateGripperCmd();
  //   updateVelocityFactor();
  //   updateLearningMode();
  //   updateCartesianMode();

  // Cartesian control with the axes
  geometry_msgs::TwistStamped cartesian_vel;
  cartesian_vel.header.stamp = ros::Time::now();
  velocity_factor_ = 0.5;
  cartesian_vel.twist.linear.x = velocity_factor_ * msg->axes[0];
  cartesian_vel.twist.linear.y = velocity_factor_ * msg->axes[1];
  cartesian_vel.twist.linear.z = velocity_factor_ * msg->axes[2];

  //   cartesian_vel.twist.angular.x = velocity_factor_ * msg->axes[3];
  //   cartesian_vel.twist.angular.y = velocity_factor_ * msg->axes[4];
  //   cartesian_vel.twist.angular.z = velocity_factor_ * msg->axes[5];

  std_msgs::Int8 cartesian_mode;
  cartesian_mode.data = cartesian_mode_;
  cartesian_cmd_pub_.publish(cartesian_vel);
  gripper_cmd_pub_.publish(gripper_cmd_);
  cartesian_mode_pub_.publish(cartesian_mode);
}

void SpacenavWrapper::processButtons(const sensor_msgs::Joy::ConstPtr& msg)
{
  button_left_ = 0;
  button_right_ = 0;

  debounceButtons(msg, JOY_BUTTON_LEFT, debounce_button_left_, button_left_);
  debounceButtons(msg, JOY_BUTTON_RIGHT, debounce_button_right_, button_right_);
}

void SpacenavWrapper::debounceButtons(const sensor_msgs::Joy::ConstPtr& msg, const int button_id,
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

void SpacenavWrapper::updateGripperCmd()
{
  // Use A to toggle gripper state (open/close)
  if (button_left_ == 1 && gripper_cmd_.data == false)
  {
    gripper_cmd_.data = true;
  }
  else if (button_left_ == 1 && gripper_cmd_.data == true)
  {
    gripper_cmd_.data = false;
  }
}
}

using namespace cartesian_controller;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "xbox_wrapper");

  SpacenavWrapper xbox_wrapper;

  return 0;
}
