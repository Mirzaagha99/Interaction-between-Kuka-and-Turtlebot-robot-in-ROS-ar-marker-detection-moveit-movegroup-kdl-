#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "geometry_msgs/Twist.h"
#include <mirzaagha_rl/markerTurtlebotAction.h>
#include "mirzaagha_rl/WakeUpKuka.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MBClient;
typedef actionlib::SimpleActionClient<mirzaagha_rl::markerTurtlebotAction> MarkClient;

bool find_marker(MBClient &ac , MarkClient & marker_action, ros::Publisher &publisher, int search){
	
	move_base_msgs::MoveBaseGoal goal;

	//coordinates of the center of the 3 different rooms
	float goals[3][3] =  { {24.82, 13.68, 1.0},{14.19, 13.68, 1.0}, {3.02, 13.68,  1.0}};

	//Looking for the desired marker (search) in the 3 different rooms 
	for (int i =0; i< 3; i++){
		goal.target_pose.pose.position.x = goals[i][0];
		goal.target_pose.pose.position.y = goals[i][1];
		goal.target_pose.pose.orientation.w = goals[i][2];
		goal.target_pose.header.frame_id = "map";
		
		// sending the goal to the MoveBaseAction
		ac.sendGoal(goal);
		ac.waitForResult();

	 	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    	{
    		ROS_INFO("Tutlebot:\n<<Room %d reached!>>\n", i+1);
   		}
    	else
    	{
        	ROS_INFO("Tutlebot:\n<<Unable to reach the room %d>>\n", i+1);
			//this cancels the goal (MoveBaseAction)  
        	ac.cancelGoal();
        	return false;
    	}
		// wait for the action server (markerTurtlebotAction) to start
		marker_action.waitForServer();
		// sending the goal to the markerTurtlebotAction
		mirzaagha_rl::markerTurtlebotGoal goal;
		//assign the tool as a goal
    	goal.find_marker = search;
    	//	sends the goal to markerTurtlebotAction
    	marker_action.sendGoal(goal);
		//if the desired marker isn’t found, 
		//the client cancels the goal after 13 seconds.
    	ros::Time end_time = ros::Time::now() + ros::Duration(14.0);
    	bool found = false;
    	
    	ROS_INFO("Tutlebot:\n<<I'm currently searching the marker in the room %d. I'm rotating>>\n", i+1 );
    	
		while(!found && end_time > ros::Time::now())
    	{
        	geometry_msgs::Twist velocity;
			//the robot rotates to see the room
        	velocity.angular.z = 0.5;
        	publisher.publish(velocity);
			//if the turtlebot finds the right tool
        	if(marker_action.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
          			return true;
    	}
		
		ROS_INFO("Tutlebot:\n<<I have seen everywhere, there isn't the desired marker in this room>>");
    	geometry_msgs::Twist velocity;
    	velocity.angular.z = 0.0;
    	publisher.publish(velocity);	
		//this cancels the goal (markerTurtlebotAction)   		
		marker_action.cancelGoal();
		
    }
	//If we are here, it means that we have finished to explore the 3 rooms, but we haven't found the desired marker.
	return false;
}
//This function is used to allow the turtlebot to reach the Kuka
bool reach_kuka(MBClient &ac , ros::Publisher &publisher){
		//This package contains the messages used to communicate with the move_base node. 
		move_base_msgs::MoveBaseGoal final_destination;
    	final_destination.target_pose.pose.position.x = 20;
    	final_destination.target_pose.pose.position.y = -5.5;
    	final_destination.target_pose.pose.orientation.w = 1.0;
    	final_destination.target_pose.header.frame_id = "map"; 
    	
		// sending the goal	
    	ac.sendGoal(final_destination);

    	ac.waitForResult();
    		
    	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    	{
        	ROS_INFO("Tutlebot:\n<<Hi Kuka, here I'm!>>");
			ros::Time time(ros::Time::now());
    		while(ros::Time::now() <= time + ros::Duration(1.0))
    		{
        		geometry_msgs::Twist velocity;
       			velocity.angular.z = 0.0;
        		publisher.publish(velocity);
    		}
			return true;
    	}
    	else
    	{
			//this cancels the goal (MoveBaseAction) 
        	ac.cancelGoal();
        	return false;
    	}
    		
}

int main(int argc, char** argv){
	
	ros::init(argc, argv, "move_turtlebot");
	MBClient ac("turtlebot3_burger/move_base", true);
	while(!ac.waitForServer (ros::Duration(5.0))){
		ROS_INFO("Tutlebot:\n<<I am waiting for the move_base action server to come up>>");
	}
		
	ros::NodeHandle node_handle;
	ros::Publisher publisher = node_handle.advertise<geometry_msgs::Twist>("turtlebot3_burger/cmd_vel", 1);
	MarkClient marker_action("marker_turtlebot", true);	
	//service client WakeUpKuka
	ros::ServiceClient client = node_handle.serviceClient<mirzaagha_rl::WakeUpKuka>("move_kuka");
	
	char search;	
	//Ask the desired tool to the operator
	do {
    	std::cout<<"Which tool do you want the turtlebot to pick up?"<<std::endl;
    	std::cout<<"Please, press [h] for the hammer"<<std::endl;
    	std::cout<<"Please, press [t] for the toolbox"<<std::endl;
    	std::cout<<"Please, press [w] for the washer"<<std::endl;
		std::cin>>search;
	} while (search!='h' && search !='t' && search!='w');
	if(search == 'h')
        search = 1;
    else if(search == 't')
        search = 2;
    else
        search = 3;
		
	if( find_marker(ac , marker_action, publisher, search) ) {

		ROS_INFO("Tutlebot:\n<<Desired marker found! I'm steel for 1 second!>>");
		ros::Time now(ros::Time::now());
    	while(ros::Time::now() <= now + ros::Duration(1.0))
    	{
        	geometry_msgs::Twist velocity;
       		velocity.angular.z = 0.0;
        	publisher.publish(velocity);
    	}		
		//if the turtlebot has reached the room of the Kuka
		if(reach_kuka(ac, publisher)){
			//this function returns true if the service is up and available, false otherwise
			//It is trying to comunicate with service server
			if(!client.exists()){
				ROS_INFO("Tutlebot:\n<<The service isn't available>>");
				return 0;}

			mirzaagha_rl::WakeUpKuka srv;

    		if(client.call(srv))
       		{
				ROS_INFO("Kuka has been awakened and it has found a marker!");

				
				ROS_WARN("Server says:\n<<%s>>",srv.response.out.c_str());
			}
    		else
			//kuka did not find the ar marker
       		{
				ROS_ERROR("Problems incurred with Kuka: there isn't the desired marker!"); 
			}
		//if the turtlebot has NOT reached the room of the Kuka
		}else {
			ROS_ERROR("Turtlebot:\n<<I am unable to reach Kuka>>");	
			return 0;}
    				
	} else{ 
		//if the desired marker has NOT been found by the turtlebot
		ROS_ERROR("Turtlebot:\n<<I am unable to find the desired marker>>");
		return 0;
	}

	return 0;

	
}
