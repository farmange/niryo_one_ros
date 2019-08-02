#include "geometry_msgs/TwistStamped.h"
//#include "control_msgs/JointJog.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Bool.h"

static const int QUEUE_LENGTH = 1;

#define BUTTON_A 0
#define BUTTON_B 1

class XboxToTwist
{
public:
  XboxToTwist() 
  {
    joy_sub_ = n_.subscribe("joy", QUEUE_LENGTH, &XboxToTwist::joyCallback, this);
    twist_pub_ = n_.advertise<geometry_msgs::TwistStamped>("dx_des", QUEUE_LENGTH);
    gripper_pub_ = n_.advertise<std_msgs::Bool>("gripper_des", QUEUE_LENGTH);
    
    debounce_button_a_ = ros::Time::now();
    gripper_cmd.data = false;
    ros::spin();
  };

private:
  // Convert incoming joy commands to TwistStamped commands for jogging.
  // The TwistStamped component goes to jogging, while buttons 0 & 1 control
  // joints directly.
  void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
  {
    // Cartesian jogging with the axes
    geometry_msgs::TwistStamped twist;
    twist.header.stamp = ros::Time::now();
    processButton(msg);
    
    // Use A to toggle gripper state (open/close)
    if(button_a == 1 && gripper_cmd.data == false)
    {
      gripper_cmd.data = true;
    }
    else if(button_a == 1 && gripper_cmd.data == true)
    {
      gripper_cmd.data = false;
    }
      
    // Left/Right Axis of the left stick
    twist.twist.linear.y = msg->axes[0];
    // Up/Down Axis of the left stick
    twist.twist.linear.z = msg->axes[1];  
    // Up/Down Axis of the right stick
    twist.twist.linear.x = msg->axes[4];

    twist_pub_.publish(twist);
    gripper_pub_.publish(gripper_cmd);
  }

  void processButton(const sensor_msgs::Joy::ConstPtr& msg)
  {
    button_a = 0;

    if(msg->buttons[BUTTON_A])
    {
      if(ros::Time::now() > debounce_button_a_)
      {
          debounce_button_a_ = ros::Time::now() + ros::Duration(0.1);
          button_a = msg->buttons[BUTTON_A];
      }
    }

  }
                
  ros::NodeHandle n_;
  ros::Subscriber joy_sub_;
  ros::Publisher twist_pub_;
  ros::Publisher gripper_pub_;
  
  ros::Time debounce_button_a_;
  std_msgs::Bool gripper_cmd;

  int button_a;
  
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "xbox_to_twist");

  XboxToTwist to_twist;

  return 0;
}
