#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <string.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose.h"
#include <std_msgs/Float64.h>
#include "mirzaagha_rl/WakeUpKuka.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include "mirzaagha_rl/markerKukaAction.h"

#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <tf/transform_datatypes.h>
#include <cmath>


typedef actionlib::SimpleActionClient<mirzaagha_rl::markerKukaAction> KukaIiwa;

bool  final_pose(moveit::planning_interface::MoveGroupInterface &move_group, const robot_state::JointModelGroup* joint_model_group, KukaIiwa &kuka_client ) {
  ros::NodeHandle NdHandle;
  KDL::Tree tree;
  KDL::Chain kinematic_chain;
  KDL::Frame init_frame;
  KDL::ChainFkSolverPos_recursive *fk_solver;
  KDL::JntArray *q_init;
  KDL::Rotation R_op_l7;

  R_op_l7 = KDL::Rotation::RPY(0, 0, -90*3.14159265359/180);//rotazion matrix between optical link and link7
  KDL::Vector p_op_l7(0, 0, 0.135);//distance between optical link and link 7, lo prendo sempre da urdf
  std::string robot_desc;
  NdHandle.param("/robot_description", robot_desc, std::string());

  //we put into the string robot_desc all the content of the urdf model and then we call the function treeFromString
  if(!kdl_parser::treeFromString(robot_desc, tree))//I call treeFromString and I put all robot_desc in tree
    {//The tree contains all the joints of the robot
        ROS_ERROR("Failed to construct KDL tree of the robot");
        return false;
    }
    //The kinematic chain represents the real kinematic structure of a robot and we can use solvers to compute both forward and inverse kinematics
    if(!tree.getChain("iiwa_link_0", "iiwa_link_7", kinematic_chain))//I take the kinematic chain from link 0 to link 7
    {
        ROS_ERROR("Unable to construct chain of the robot from the link names provided! ");
        return false;
    }

  fk_solver = new KDL::ChainFkSolverPos_recursive(kinematic_chain);//Forward position solver -> I'm initializing the solver
  //I send the results to the markerKukaAction
  mirzaagha_rl::markerKukaResultConstPtr result = 0;
  result = kuka_client.getResult(); 
  q_init = new KDL::JntArray(kinematic_chain.getNrOfJoints());// Request the total number of joints in the chain.

  moveit::core::RobotStatePtr current_state;   
  std::vector<double> joint_group_positions;
  //Now, we call the planner to compute the plan and visualize it.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  current_state = move_group.getCurrentState();
	//For a given group, copy the position values of the variables that make up the group into another location, in the order that the variables are found in the group.
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    //
  for(int i = 0; i < 7; i++)
  {
      q_init->data[i] = joint_group_positions[i];
  }

  fk_solver->JntToCart(*q_init, init_frame);//init_frame=base frame of the Kuka ; Calculate forward position kinematics for a KDL::Chain, from joint coordinates to cartesian pose.
  // I save the coordinate of the center
  KDL::Vector center_pos(result->found_marker[0], result->found_marker[1], result->found_marker[2]);

  //We must refer to the base frame of the Kuka   
  KDL::Vector end_pos= init_frame.M * (R_op_l7 * center_pos + p_op_l7) + init_frame.p;
  //we refer the final position of the marker in the optical frame to the link 7 using:  (R_op_l7 * center_pos + p_op_l7)
  //then I refer it to the base frame of the robot using the init_frame

  end_pos.z(1);//I have decided an arbitrary value for the z coordinate
  ROS_INFO("Finals positions:\n x= %f ", end_pos(0));//result->found_marker[0]
  ROS_INFO("y= %f", end_pos(1));
  ROS_INFO("z= %f", end_pos(2));


  bool final_success= false;

   //We can plan a motion for this group to a desired pose for the end-effector.
  geometry_msgs::Pose target_pose2;

  target_pose2.position.x = end_pos[0];
  target_pose2.position.y = end_pos[1];
  target_pose2.position.z = end_pos[2];

  tf::Quaternion q;//the end effector must look down, we need a quaternion to define the orientation of the end effector 
  
  // Create this quaternion from roll/pitch/yaw (in radians)
	q.setRPY(-3.14,0,0);//Set the quaternion using fixed axis RPY. Angle around X .
  	    
  target_pose2.orientation.x = q.x();
	target_pose2.orientation.y = q.y();
	target_pose2.orientation.z = q.z();
	target_pose2.orientation.w = q.w();
  	    
  move_group.setPoseTarget(target_pose2);//it goes on the center
     
  final_success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  //ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", final_success ? "" : "FAILED");
  move_group.move();
  return true;
}



bool service_callback( mirzaagha_rl::WakeUpKuka::Request &req, mirzaagha_rl::WakeUpKuka::Response &res) {

  //frequency
	ros::Rate r(20);
  //
  KukaIiwa kuka_client("marker_kuka",true);

  // ^^^^^
  //
  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
  // are used interchangably.

  static const std::string PLANNING_GROUP = "kuka_iiwa";

  // The :move_group_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =
  move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP); 
  
  //Now, we call the planner to compute the plan and visualize it.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = false;
  //We can plan a motion for this group to a desired pose for the end-effector.
  geometry_msgs::Pose target_pose1;

  // Planning to a joint-space goal
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Let's set a joint space goal and move towards it.  This will replace the
  // pose target we set above.
  //
  // To start, we'll create an pointer that references the current robot's state.
  // RobotState is the object that contains all the current position/velocity/acceleration data.
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  
  // Next get the current set of joint values for the group.
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  //Desired values for the joints
  joint_group_positions[0] = 0.0;  // radians
  joint_group_positions[1] = 0.0;  // radians
  joint_group_positions[2] = 0.0;  // radians
  joint_group_positions[3] = 1.0;  // radians
  joint_group_positions[4] = 0.0;  // radians
  joint_group_positions[5] = -1.6; // radians
  joint_group_positions[6] = 0.0;  // radians
  //Set the JointValueTarget and use it for future planning requests.
  move_group.setJointValueTarget(joint_group_positions);
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	move_group.move();	 
	// wait until finish markerKukaAction
  kuka_client.waitForServer();
  
  mirzaagha_rl::markerKukaGoal goal;
  
  goal.find_marker = 900; 
  kuka_client.sendGoal(goal); 
  // Next get the current set of joint values for the group.
  current_state = move_group.getCurrentState();
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions); 
  
  //Desired final value of the joints, we want to explore all the floor around the Kuka, the first joint can do a rotation of 360°
  joint_group_positions[0] = 6.27;  // radians
  joint_group_positions[1] = 0.0;  // radians
  joint_group_positions[2] = 0.0;  // radians
  joint_group_positions[3] = 1.0;  // radians
  joint_group_positions[4] = 0.0;  // radians
  joint_group_positions[5] = -1.6;  // radians
  joint_group_positions[6] = 0.0;  // radians
  
  //Set the JointValueTarget and use it for future planning requests.
  move_group.setJointValueTarget(joint_group_positions);
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  //ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  ros::Time end_time = ros::Time::now() + ros::Duration(30.0);
  
  while(end_time > ros::Time::now())
  {	
    	//Plan and execute a trajectory that takes the group of joints declared in the constructor to the specified target
	    move_group.asyncMove();
	 		
    	if(kuka_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){//if the desired marker is found
 	      move_group.stop();
        if(final_pose( move_group, joint_model_group ,kuka_client))//if the end-effector reaches the center of the marker
        {
          //I comunicate through ros service
          res.out="Task completed successfully!";
          return true;
        }else{  //if the end-effector DOESN'T reach the center of the marker
          res.out="The robot is unable to reach the center of the marker";
          return true;}
        
      }
        r.sleep();
  }
  //if we are here, there isn't the desired marker
  kuka_client.cancelGoal();
  return false; 
	
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_kuka_");
  ros::NodeHandle node_handle;
  ros::MultiThreadedSpinner spinner(2);
  ////service server WakeUpKuka
  ros::ServiceServer kuka_wake =node_handle.advertiseService("move_kuka", &service_callback);
  ROS_INFO("Kuka started<< I'm waiting to be contacted>>.\n");
  spinner.spin();
  return 0;
}