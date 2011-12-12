#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "ros/ros.h"
#include "body_msgs/Skeletons.h"
#include "body_msgs/SkeletonJoint.h"
#include "body_msgs/Skeleton.h"
#include "body_msgs/Hands.h"
#include "body_msgs/Hand.h"

double lastLeftHand_x = 0.0;

double leftHand_x = 0.0;
double leftHand_y = 0.0;
double leftHand_z = 0.0;

bool spinRobot_start = false;
bool spinRobot_inc = false;

void spinRobot(bool clockwise){
	ROS_INFO("Robot is spinning.");
}

void gestureRecog(double curLeftHand_x){
//SPIN ROBOT CLOCKWISE	
	//if left hand between -0.4 and -0.5, hand is in starting position
	if(curLeftHand_x <= -0.4 && curLeftHand_x >= -0.5){
		spinRobot_start = true;
		lastLeftHand_x = curLeftHand_x;
		ROS_INFO("starting.");
	}
	
	//if left hand is increasing, then set second condition to true
	else if(spinRobot_start && curLeftHand_x > lastLeftHand_x && curLeftHand_x < 0.1){
		spinRobot_inc = true;
		ROS_INFO("increasing");
	}
	
	//if left hand reaches 0.1, gesture complete, send command
	else if(spinRobot_start && spinRobot_inc && leftHand_x >= 0.1){
		spinRobot(true);
		
		//reset conditions
		spinRobot_start = false;
		spinRobot_inc = false;
	}
}

//get the values from the topics and store them for use
void skeletonCallback(const body_msgs::SkeletonsConstPtr &msg){
	//update hand positions
	leftHand_x = msg->skeletons[0].left_hand.position.x;
	leftHand_y = msg->skeletons[0].left_hand.position.y;
	leftHand_z = msg->skeletons[0].left_hand.position.z;
	
//	ROS_INFO("leftHand_x: %lf",leftHand_x);
	//ROS_INFO("leftHand_y: %lf",leftHand_y);
	//ROS_INFO("leftHand_z: %lf",leftHand_z);
	//ROS_INFO(" ");
	//ROS_INFO(" ");
	gestureRecog(leftHand_x);
}

int main(int argc, char**argv){
	
	//initialize ROS
	ros::init(argc, argv, "main");
	
	//create a nodehandle
	ros::NodeHandle nh;
	
	//use handle to subscribe to messages (name, buffer, callback pointer)
	ros::Subscriber skeleton_sub = nh.subscribe("/skeletons", 1, skeletonCallback);
	
	//give callback time to respond
	ros::spin();

}


