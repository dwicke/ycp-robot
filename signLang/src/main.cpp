#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "ros/ros.h"
#include "body_msgs/Skeletons.h"
#include "body_msgs/SkeletonJoint.h"
#include "body_msgs/Skeleton.h"
#include "body_msgs/Hands.h"
#include "body_msgs/Hand.h"

#include "MotorControlMsg/MotorCommand.h"

#include "robot_msgs/MotorData.h"

double lastLeftHand_x = 0.0;

double leftHand_x = 0.0;
double leftHand_y = 0.0;
double leftHand_z = 0.0;

bool spinRobot_start = false;
bool spinRobot_inc = false;
bool sendSpin = false;

ros::Publisher gesture_pub;

void spinRobot(bool clockwise){
	/*MotorControlMsg::MotorCommand GestureMsg;
		
		GestureMsg.precedence = -1;
		GestureMsg.isLeftRightVel = true;
		GestureMsg.linear_velocity = 0.25;
		GestureMsg.angular_velocity = -0.25;
		
		gesture_pub.publish(GestureMsg);
		sendSpin = false;
		ROS_INFO("SUCCESS!!! Motor Command sent.");*/
		
	robot_msgs::MotorData GestureMsg;
	
	GestureMsg.motor_left_velocity = 25;
	GestureMsg.motor_left_time = 100;
	GestureMsg.motor_right_velocity = -25;
	GestureMsg.motor_right_time = 100;
	
	sendSpin = false;
	ROS_INFO("SUCCESS!!! Motor Command sent.");
}

void gestureRecog(double curLeftHand_x){
//SPIN ROBOT CLOCKWISE	
	ROS_INFO("Current hand position: %lf", curLeftHand_x);

	//if left hand between -0.4 and -0.5, hand is in starting position
	//can't do specific values because x values subjective to Kinect position
	//if(curLeftHand_x <= -0.4 && curLeftHand_x >= -0.5){
	if(curLeftHand_x <= -0.2){
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
		//spinRobot(true);
		sendSpin = true;
		//ROS_INFO("Robot SHOULD be spinning, but it's not.");
		
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
	
	//if this doesn't work, send it to the Converter without Braitenburg
	//gesture_pub = nh.advertise<MotorControlMsg::MotorCommand>("Motor_Command", 1000);
	gesture_pub = nh.advertise<robot_msgs::MotorData>("motordata",100);
	
	ros::Rate loop_rate(10);
	while(ros::ok()){
		//ROS_INFO("I'm at the start of the loop.");
		if(sendSpin){
			/*ROS_INFO("Robot is spinning.");
			MotorControlMsg::MotorCommand GestureMsg;
		
			GestureMsg.precedence = -1;
			GestureMsg.isLeftRightVel = true;
			GestureMsg.linear_velocity = 0.25;
			GestureMsg.angular_velocity = -0.25;
		
			gesture_pub.publish(GestureMsg);
			sendSpin = false;*/
			robot_msgs::MotorData GestureMsg;
	
			GestureMsg.motor_left_velocity = 25;
			GestureMsg.motor_left_time = 100;
			GestureMsg.motor_right_velocity = -25;
			GestureMsg.motor_right_time = 100;
			
			//publish message 5 times because of fix in Converter for Braitenburg
			for(int i=0; i<5; i++){
				gesture_pub.publish(GestureMsg);
			}
			
			//send zeroes to stop the spinning
			GestureMsg.motor_left_velocity = 0;
			GestureMsg.motor_left_time = 100;
			GestureMsg.motor_right_velocity = 0;
			GestureMsg.motor_right_time = 100;
			
			//publish message 5 times because of fix in Converter for Braitenburg
			for(int i=0; i<5; i++){
				gesture_pub.publish(GestureMsg);
			}
			
			sendSpin = false;
			ROS_INFO("SUCCESS!!! Motor Command sent.");
		}
	
		//give callback time to respond
		ros::spinOnce();
		
		loop_rate.sleep();
		ROS_INFO("End of loop reached.");
	}
}


