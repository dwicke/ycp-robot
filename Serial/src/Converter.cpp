/*******************************************************************************
Converter.cpp
This is a program that will initialize the serial library, and then convert the data into
the proper format for ROS.  Subscribes to the motor data, publishes the sensor data

JMC
9/29/11
********************************************************************************/


#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "ros/ros.h"
#include "robot_msgs/SensorData.h"
#include "robot_msgs/MotorData.h"
#include "DrRobotMotionSensorDriver.hpp"

//serial parameters
//TODO Check to make sure this is the right port on the beagleboard
#define PORT "/dev/ttyUSB0"
#define BAUD 115200



using namespace DrRobot_MotionSensorDriver;

DrRobotMotionSensorDriver *driver; //serial driver needs global access


//callback for the motor Data
//if there is new information, it needs to be converted and sent to the robot
//motors are controlled by taking in a velocity value which is then set for the specific motor channel
void motorCallback(const robot_msgs::MotorData::ConstPtr& msg){
	ROS_INFO("Motor data received: \n");
	
	//send left/right values to the robot
	int motorLPASS = driver->sendMotorCtrlCmd(Velocity, 0, msg->motor_left_velocity, msg->motor_left_time);
	if(motorLPASS < 0) ROS_INFO("Left motor value was not sent to the robot!");
	int motorRPASS = driver->sendMotorCtrlCmd(Velocity, 1, msg->motor_right_velocity*-1, msg->motor_right_time);
	if(motorRPASS < 0) ROS_INFO("Right motor value was not sent to the robot!");

}


int IR_Convert(int value){
	return 739.38*pow(value,-.8105);
}

void serialInit(){
	//initialize internal vars to default - constructor
	driver = new DrRobotMotionSensorDriver();
	//set id and ports
	DrRobotMotionConfig *config = new DrRobotMotionConfig();
	driver->setDrRobotMotionDriverConfig(config);
	//open the serial port and start the driver
	driver->openSerial(PORT, BAUD); //port,baudrate
}



int main(int argc, char**argv){
	
	//initialize the serial port, linux ports are "/dev/ttyUSB0" for default for usb port
	serialInit();
	
	//initialize ROS
	ros::init(argc, argv, "Converter");

	//create a handle to the process' node
	ros::NodeHandle n;

	//create a handle to subscribe to messages, name, buffer, callback function pointer
	ros::Subscriber motordata_sub = n.subscribe("motordata", 100, motorCallback);

	//create a handle to publish messages, buffer name and size for args
	ros::Publisher sensordata_pub = n.advertise<robot_msgs::SensorData>("sensordata", 20);

	//check to see if the node handle returned an empty ros::Publisher
	if(!sensordata_pub) ROS_INFO("Failed to create a publisher node!");
	
	ros::Rate loop_rate(10); //10 Hz publish time
			
	int count = 0; //count of how many messages were sent
	
	//make a range sensor data object and a standard sensor object
	RangeSensorData *rangeSensorData = new RangeSensorData();
	StandardSensorData *standardSensorData = new StandardSensorData();
	while(ros::ok())
	{
		//This is a message object, it is stuffed with the datas and then can be published
		robot_msgs::SensorData SensorMsg;
		
		//get the sensor data, return values are 6 Ultrasonic sensors and 10 IRs, X80SVP does not require all of these.
		driver->readRangeSensorData(rangeSensorData);
		driver->readStandardSensorData(standardSensorData);

		//set the sensor data

		//Ultrasonic: data is 8 bit unsigned, in cm's
		SensorMsg.ultrasonic_frontLeft_distance = rangeSensorData->usRangeSensor[0];
		SensorMsg.ultrasonic_frontCenter_distance = rangeSensorData->usRangeSensor[1];
		SensorMsg.ultrasonic_frontRight_distance = rangeSensorData->usRangeSensor[2];
		SensorMsg.ultrasonic_rearRight_distance = rangeSensorData->usRangeSensor[3];
		SensorMsg.ultrasonic_rearCenter_distance = rangeSensorData->usRangeSensor[4];
		SensorMsg.ultrasonic_rearLeft_distance = rangeSensorData->usRangeSensor[5];
		
		//Infrared: data is floats
		SensorMsg.infrared_frontLeftLeft_distance = IR_Convert(rangeSensorData->irRangeSensor[0]);
		SensorMsg.infrared_frontLeftCenter_distance = IR_Convert(rangeSensorData->irRangeSensor[1]);
		SensorMsg.infrared_frontRightCenter_distance = IR_Convert(rangeSensorData->irRangeSensor[2]);
		SensorMsg.infrared_frontRightRight_distance = IR_Convert(rangeSensorData->irRangeSensor[3]);
		SensorMsg.infrared_right_distance = IR_Convert(rangeSensorData->irRangeSensor[4]);
		SensorMsg.infrared_rear_distance = IR_Convert(rangeSensorData->irRangeSensor[5]);
		SensorMsg.infrared_left_distance = IR_Convert(rangeSensorData->irRangeSensor[6]);

		//human sensor, returned from robot in a different message
		SensorMsg.human_left_motion = standardSensorData->humanSensorData[0];
		SensorMsg.human_left_presence = standardSensorData->humanSensorData[1];
		SensorMsg.human_right_motion = standardSensorData->humanSensorData[2];
		SensorMsg.human_right_presence = standardSensorData->humanSensorData[3];

		ROS_INFO("Human left presence = %d", standardSensorData->humanSensorData[1]);
		ROS_INFO("Human right presence = %d", standardSensorData->humanSensorData[3]);
		
			
/*
		//print out the sensor data messages for testing purposes
 		ROS_INFO("Infrasonic range sensor 1 = %d\n", IR_Convert(rangeSensorData->irRangeSensor[0]));
		ROS_INFO("Infrasonic range sensor 2 = %d\n", IR_Convert(rangeSensorData->irRangeSensor[1]));
		ROS_INFO("Infrasonic range sensor 3 = %d\n", IR_Convert(rangeSensorData->irRangeSensor[2]));
		ROS_INFO("Infrasonic range sensor 4 = %d\n", IR_Convert(rangeSensorData->irRangeSensor[3]));
		ROS_INFO("Infrasonic range sensor 5 = %d\n", IR_Convert(rangeSensorData->irRangeSensor[4]));
		ROS_INFO("Infrasonic range sensor 6 = %d\n", IR_Convert(rangeSensorData->irRangeSensor[5]));
		ROS_INFO("Infrasonic range sensor 7 = %d\n", IR_Convert(rangeSensorData->irRangeSensor[6]));
*/
		//now publish the sensor data
		sensordata_pub.publish(SensorMsg);
	
		//spinOnce is required so the callback finishes
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	
	}
	
	return 0;
}

