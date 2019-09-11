 
 // TODO copyright
 #include "ros/ros.h"

 #include "orthopus_interface/move_group_node.h"

 #include <moveit/robot_state/robot_state.h>
 
 #define RATE 10
 
 namespace move_group_node
 {
   MoveGroupNode::MoveGroupNode() : spinner_(4), move_group_("arm")
   {
     ROS_DEBUG_STREAM("MoveGroupNode constructor");
     initializeSubscribers();
     initializePublishers();
     initializeServices();
     initializeActions();
     
     state_ = 0;

     // Wait for initial messages
     ROS_INFO("Waiting for first joint msg.");
     ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states");
     ROS_INFO("Received first joint msg.");
     
     spinner_.start();
     
     joint_model_group_ = move_group_.getCurrentState()->getJointModelGroup("arm");
     
     while(ros::ok())
     {
      /* snapshot protected members */
      state_mutex_.lock();
      int state_cpy = state_;
      state_mutex_.unlock();
      
      std_msgs::Int32 state_msg;
      state_msg.data = state_cpy;
      state_pub_.publish(state_msg);
      
      if(state_cpy == 1)
      {
        move_group_.move();
        
        state_cpy = 0;
      }
      
      /* update protected members */
      state_mutex_.lock();
      state_ = state_cpy;
      state_mutex_.unlock();

//       std_msgs::Int32 state_msg;
//       state_msg.data = state_cpy;
//       state_pub_.publish(state_msg);
      
      ros::Duration(0.1).sleep();
    }
     
     ROS_ERROR_STREAM("Wait for shutdown....");
     ros::waitForShutdown();
     ROS_ERROR_STREAM("MoveGroupNode kill !!!");
     
   }
   
   void MoveGroupNode::initializeSubscribers()
   {
      /* No subscriber */
   }
   void MoveGroupNode::initializePublishers()
   {
     ROS_DEBUG_STREAM("MoveGroupNode initializePublishers");
     
     state_pub_ = n_.advertise<std_msgs::Int32>("/orthopus_interface/move_groupe_node/state", 1);
   }
   void MoveGroupNode::initializeServices()
   {
     ROS_DEBUG_STREAM("MoveGroupNode initializeServices");
     
     move_service_ = n_.advertiseService("/orthopus_interface/move_groupe_node/move",
                                         &MoveGroupNode::callbackMove, this);
     get_state_service_ = n_.advertiseService("/orthopus_interface/move_groupe_node/get_state",
                                              &MoveGroupNode::callbackGetState, this);
   }
   void MoveGroupNode::initializeActions()
   {
      /* No action */
   }
   bool MoveGroupNode::callbackMove(niryo_one_msgs::RobotMove::Request& req, niryo_one_msgs::RobotMove::Response& res)
   {
     robot_state::RobotState start_state(*move_group_.getCurrentState());
     geometry_msgs::Pose current_pose = move_group_.getCurrentPose().pose;
     if(req.cmd.cmd_type == 666)
     {
       move_group_.stop();
     }
     else if(req.cmd.cmd_type == 2)
     {
       move_group_.setJointValueTarget(req.cmd.joints);
     }
     else
     {
      geometry_msgs::Pose target_pose1;
      target_pose1.position.x = req.cmd.pose_quat.position.x;
      target_pose1.position.y = req.cmd.pose_quat.position.y;
      target_pose1.position.z = req.cmd.pose_quat.position.z;

      target_pose1.orientation.x = req.cmd.pose_quat.orientation.x;
      target_pose1.orientation.y = req.cmd.pose_quat.orientation.y;
      target_pose1.orientation.z = req.cmd.pose_quat.orientation.z;
      target_pose1.orientation.w = req.cmd.pose_quat.orientation.w;
      
      ROS_ERROR_STREAM("=================current_pose================");
      ROS_ERROR_STREAM(current_pose);
      ROS_ERROR_STREAM("=================target_pose1================");
      ROS_ERROR_STREAM(target_pose1);
      
      shape_msgs::SolidPrimitive primitive;
      primitive.type = primitive.BOX;
      primitive.dimensions.resize(3);
      
      double constrain_box_scale = 1.0;
      primitive.dimensions[0] = constrain_box_scale * std::abs(target_pose1.position.x - current_pose.position.x);
      primitive.dimensions[1] = constrain_box_scale * 2.0 * std::abs(target_pose1.position.y - current_pose.position.y);
      primitive.dimensions[2] = constrain_box_scale * 2.0 * std::abs(target_pose1.position.z - current_pose.position.z);
      
      /* A pose for the box (specified relative to frame_id) */
      geometry_msgs::Pose box_pose;
      box_pose.orientation.w = 1.0;
      // place between start point and goal point.
      box_pose.position.x = (target_pose1.position.x + current_pose.position.x)/2.0;
      box_pose.position.y = (target_pose1.position.y + current_pose.position.y)/2.0;
      box_pose.position.z = (target_pose1.position.z);
      
      moveit_msgs::PositionConstraint pcm;
      pcm.link_name = "tool_link";
      pcm.header.frame_id = "ground_link";

      pcm.constraint_region.primitives.push_back(primitive);
      pcm.constraint_region.primitive_poses.push_back(box_pose);
      pcm.weight = 1.0;
      
      moveit_msgs::OrientationConstraint ocm;
      ocm.link_name = "tool_link";
      ocm.header.frame_id = "ground_link";
      ocm.orientation.w = 1.0;
      ocm.absolute_x_axis_tolerance = 0.15;
      ocm.absolute_y_axis_tolerance = 0.15;
      ocm.absolute_z_axis_tolerance = 5.0;
      ocm.weight = 1.0;
      
      
      moveit_msgs::Constraints test_constraints;
      test_constraints.orientation_constraints.push_back(ocm);
      test_constraints.position_constraints.push_back(pcm);
      if(req.cmd.cmd_type == 1)
      {
        move_group_.setPathConstraints(test_constraints);
        start_state.setFromIK(joint_model_group_, target_pose1);
        move_group_.setStartState(start_state);
        
      }
      

      
      move_group_.setPoseTarget(target_pose1);
     }
     move_group_.setPlanningTime(20.0);
     moveit::planning_interface::MoveGroupInterface::Plan my_plan;
     moveit::planning_interface::MoveItErrorCode result = move_group_.plan(my_plan);
     ROS_INFO_STREAM("End finding planning solution with code : " << result);
     
     if (result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
     {
       state_mutex_.lock();
       state_ = 1;
       state_mutex_.unlock();
       res.status = 8000;
     }
     else
     {
      res.status = 0;
     }
    return true;
   }
   
   bool MoveGroupNode::callbackGetState(niryo_one_msgs::GetInt::Request& req, niryo_one_msgs::GetInt::Response& res)
   {
     state_mutex_.lock();
     res.value = state_;
     state_mutex_.unlock();
     
     return true;
   }
   
 }
 
 using namespace move_group_node;
 int main(int argc, char** argv)
 {
   ros::init(argc, argv, "move_group_node");
   
   MoveGroupNode move_group_node;
   
   return 0;
 }
 
