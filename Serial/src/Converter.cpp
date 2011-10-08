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
	int motorRPASS = driver->sendMotorCtrlCmd(Velocity, 1, msg->motor_right_velocity, msg->motor_right_time);
	if(motorRPASS < 0) ROS_INFO("Right motor value was not sent to the robot!");

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
	
	//make a sensor data struct
	RangeSensorData *rangeSensorData = new RangeSensorData();
	
	while(ros::ok())
	{
		//This is a message object, it is stuffed with the datas and then can be published
		robot_msgs::SensorData SensorMsg;
		
		//get the sensor data, return values are 6 Ultrasonic sensors and 10 IRs, X80SVP does not require all of these.
		driver->readRangeSensorData(rangeSensorData);
		
		//set the sensor data
		//data is 8 bit unsigned, in cm's
		SensorMsg.ultrasonic_frontLeft_distance = rangeSensorData->usRangeSensor[0];
		SensorMsg.ultrasonic_frontCenter_distance = rangeSensorData->usRangeSensor[1];
		SensorMsg.ultrasonic_frontRight_distance = rangeSensorData->usRangeSensor[2];
		SensorMsg.ultrasonic_rearRight_distance = rangeSensorData->usRangeSensor[3];
		SensorMsg.ultrasonic_rearCenter_distance = rangeSensorData->usRangeSensor[4];
		SensorMsg.ultrasonic_rearLeft_distance = rangeSensorData->usRangeSensor[5];		
			

		//print out the sensor data messages for testing purposes
 		ROS_INFO("Ultrasonic range sensor frontLeft = %d\n", rangeSensorData->usRangeSensor[0]);
		ROS_INFO("Ultrasonic range sensor frontCenter = %d\n", rangeSensorData->usRangeSensor[1]);
		ROS_INFO("Ultrasonic range sensor frontRight = %d\n", rangeSensorData->usRangeSensor[2]);
		ROS_INFO("Ultrasonic range sensor rearRight = %d\n", rangeSensorData->usRangeSensor[3]);
		ROS_INFO("Ultrasonic range sensor rearCenter = %d\n", rangeSensorData->usRangeSensor[4]);
		ROS_INFO("Ultrasonic range sensor rearLeft = %d\n", rangeSensorData->usRangeSensor[5]);

		//now publish the sensor data
		sensordata_pub.publish(SensorMsg);
	
		//spinOnce is required so the callback finishes
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	
	}
	

	//ports are configure, robot is ready to receieve packets
	//note:sensor update rate is around 10 Hz (firmware specified) so don't go faster than that
	
	
	//MotorSensorData *motorSensorData = new MotorSensorData();
	//driver->getDrRobotMotionDriverConfig(config);
	
	//for(;;){
		
		//Use the subscribed data to write the correct values to the motors
		//move forward
		//int error = driver->sendMotorCtrlAllCmd(PWM, 32767-5000, 5000,16384,16384,16384,16384,100);
		//if(error = -1) fprintf(stderr, "An error has occured while trying to move");
		


		//read some sensors	
		//driver->readStandardSensorData(sensorData);
		//driver->readMotorSensorData(motorSensorData);
		//driver->getDrRobotMotionDriverConfig(config);
		
		//print out some values
		//fprintf(stdout, "Human Sensor Data 1: %d\n", sensorData->humanSensorData[1]);
		//fprintf(stdout, "Human Sensor Data 2: %d\n", sensorData->humanSensorData[1]);
		//fprintf(stdout, "Human Sensor Data 3: %d\n", sensorData->humanSensorData[3]);	
		//fprintf(stdout, "Human Sensor Data 4: %d\n", sensorData->humanSensorData[3]);
		
		//save the sensordata into the sensorData struct to be published
		
	//}
	
	return 0;
}

