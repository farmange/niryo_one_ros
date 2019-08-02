// #include "geometry_msgs/TwistStamped.h"
#include "ros/ros.h"
// #include "sensor_msgs/Joy.h"
// #include <sensor_msgs/JointState.h>
// #include <control_msgs/JointJog.h>
// 
// // MoveIt!
// #include <moveit/robot_model_loader/robot_model_loader.h>
// #include <moveit/robot_model/robot_model.h>
// #include <moveit/robot_state/robot_state.h>


// namespace planar_robot_play
// {
// static const int NUM_SPINNERS_VELTOPOS = 1;
// static const int Q_DOT_DES_QUEUE_LENGTH = 1;
// static const int Q_DES_QUEUE_LENGTH = 1;
// static const int JOINT_SUB_QUEUE_LENGTH = 1;
// static const double CALC_PERIOD = 0.1;

class VelToPos
{
public:
    VelToPos()
    {
    }
//    VelToPos() : spinner_(NUM_SPINNERS_VELTOPOS)
//   {
//     pthread_mutex_init(&q_meas_mutex_, nullptr);
//     pthread_mutex_init(&q_dot_des_mutex_, nullptr);
// 
//     q_dot_des_sub_ = n_.subscribe("q_dot_des", Q_DOT_DES_QUEUE_LENGTH, &VelToPos::qDotDesCallback, this);
//     joints_sub_ = n_.subscribe("joint_states", JOINT_SUB_QUEUE_LENGTH, &VelToPos::jointsCB, this);
//     joints_pub_ = n_.advertise<sensor_msgs::JointState>("joint_states", Q_DES_QUEUE_LENGTH);
//     
//     spinner_.start();
// 
//     sensor_msgs::JointState q_command;
//     q_command.header.stamp = ros::Time::now();
//     q_command.name.push_back("joint_1");
//     q_command.name.push_back("joint_2");     q_command.position.push_back(40.*3.141589/180.);     q_command.position.push_back(-1.127936);
//     
//     joints_pub_.publish(q_command);
//     ros::Rate loop_rate(1./CALC_PERIOD);
// 
//     int iteration = 0;
//     while (iteration < 100)
//     {
//       joints_pub_.publish(q_command);
//       iteration++;
//       loop_rate.sleep();
//     }
// //     // Wait for initial messages
// //     ROS_INFO("VelToPos: Waiting for first joint msg.");
// //     ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states");
// //     ROS_INFO("VelToPos: Received first joint msg.");
//     
// //     ROS_INFO("VelToPos: Waiting for first desired velocity msg.");
// //     ros::topic::waitForMessage<control_msgs::JointJog>("q_dot_des");
// //     ROS_INFO("VelToPosReceived first desired velocity msg.");
//     
//     
// 
// 
//     // Now do jogging calcs
//     while (ros::ok())
//     {
//       ROS_INFO(">>>> VelToPos Update ");
// 
// 
//       pthread_mutex_lock(&q_meas_mutex_);
//       sensor_msgs::JointState q_command = q_meas_;
//       pthread_mutex_unlock(&q_meas_mutex_);
// 
//       pthread_mutex_lock(&q_dot_des_mutex_);
//       for(int i = 0; i < q_dot_des_.joint_names.size(); i++)
//       {
//         q_command.position[i] = q_meas_.position[i] + (q_dot_des_.velocities[i] * CALC_PERIOD * 0.1);
//       }
//       pthread_mutex_unlock(&q_dot_des_mutex_);  
//       ROS_DEBUG_STREAM("q_command: \n" << q_command << "\n");
// 
//       joints_pub_.publish(q_command);
// 
//       loop_rate.sleep();
//     }
//   };

private:
//   void jointsCB(const sensor_msgs::JointStateConstPtr& msg)
//   {
//     pthread_mutex_lock(&q_meas_mutex_);
//     q_meas_ = *msg;
//     pthread_mutex_unlock(&q_meas_mutex_);
//     ROS_DEBUG_STREAM("q_meas_: \n" << q_meas_ << "\n");
//   }
//   
//   void qDotDesCallback(const control_msgs::JointJogPtr& msg)
//   {
//     pthread_mutex_lock(&q_dot_des_mutex_);
//     q_dot_des_ = *msg;
//     pthread_mutex_unlock(&q_dot_des_mutex_);
//     ROS_DEBUG_STREAM("q_dot_des_: \n" << q_dot_des_ << "\n");
//   }
//   
  ros::NodeHandle n_;
//   ros::Subscriber q_dot_des_sub_;
//   ros::Subscriber joints_sub_;
//   ros::Publisher joints_pub_;
//   ros::AsyncSpinner spinner_;

//   sensor_msgs::JointState q_meas_;
//   pthread_mutex_t q_meas_mutex_;
//   
//   control_msgs::JointJog q_dot_des_;
//   pthread_mutex_t q_dot_des_mutex_;
// 
//   sensor_msgs::JointState joints;
//   geometry_msgs::TwistStamped twist_cmd_;
  
};
// }  // namespace planar_robot_play

int main(int argc, char** argv)
{
  ros::init(argc, argv, "veltopos");

  VelToPos veltopos;

  return 0;
}
