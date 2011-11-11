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

#define RIGHT_MOTOR_OFFSET 18

//prototypes
int IR_Convert(int value);
double Motor_Convert(double motor_val);
long microtime();
void print_infrared();
void print_ultrasonic();

using namespace DrRobot_MotionSensorDriver;

//globals
DrRobotMotionSensorDriver *driver; //serial driver needs global access

//make a range sensor data object and a standard sensor object
RangeSensorData *rangeSensorData = new RangeSensorData();
StandardSensorData *standardSensorData = new StandardSensorData();

long motor_time=0;

//callback for the motor Data
//if there is new information, it needs to be converted and sent to the robot
//motors are controlled by taking in a velocity value which is then set for the specific motor channel
void motorcmd(float left,float right,int time)
{
	int ret=driver->sendMotorCtrlAllCmd(Velocity,Motor_Convert(left),Motor_Convert(-right/4.0*12.0),0,0,0,0,0);//last param is time, seems to be broken
	if(ret<0)
	{
		ROS_INFO("motor value was not sent to the robot!");
		exit(1);
	}
	motor_time=microtime()+time;
}

void motorCallback(const robot_msgs::MotorData::ConstPtr& msg){
	//ROS_INFO("Motor data received: \n");
	ROS_INFO("Motor data received: [%.0f,%.0f]",msg->motor_left_velocity,msg->motor_right_velocity);
	
	motorcmd(msg->motor_left_velocity,msg->motor_right_velocity,msg->motor_left_time);
	
	return;
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
	ros::Subscriber motordata_sub = n.subscribe("motordata", 1, motorCallback);

	//create a handle to publish messages, buffer name and size for args
	ros::Publisher sensordata_pub = n.advertise<robot_msgs::SensorData>("sensordata", 1);

	//check to see if the node handle returned an empty ros::Publisher
	if(!sensordata_pub) ROS_INFO("Failed to create a publisher node!MotorCtrl");
	
	ros::Rate loop_rate(10); //10 Hz publish time
			
	int count = 0; //count of how many messages were sent
	
	while(ros::ok())
	{
		//This is a message object, it is stuffed with the datas and then can be published
		robot_msgs::SensorData SensorMsg;
		
		//get the sensor data, return values are 6 Ultrasonic sensors and 10 IRs, X80SVP does not require all of these.
		if( driver->readRangeSensorData(rangeSensorData)!=0 ||
			driver->readStandardSensorData(standardSensorData)!=0)
		{
			ROS_INFO("Error reading sensor data!");
			exit(1);
		}

		//set the sensor data
		//Ultrasonic: data is 8 bit unsigned, in cm's
		SensorMsg.ultrasonic_frontLeft_distance = rangeSensorData->usRangeSensor[5];
		SensorMsg.ultrasonic_frontCenter_distance = rangeSensorData->usRangeSensor[4];
		SensorMsg.ultrasonic_frontRight_distance = rangeSensorData->usRangeSensor[3];

		/*
		SensorMsg.ultrasonic_frontLeft_distance = rangeSensorData->usRangeSensor[0];
		SensorMsg.ultrasonic_frontCenter_distance = rangeSensorData->usRangeSensor[1];
		SensorMsg.ultrasonic_frontRight_distance = rangeSensorData->usRangeSensor[2];
		SensorMsg.ultrasonic_rearRight_distance = rangeSensorData->usRangeSensor[3];
		SensorMsg.ultrasonic_rearCenter_distance = rangeSensorData->usRangeSensor[4];
		SensorMsg.ultrasonic_rearLeft_distance = rangeSensorData->usRangeSensor[5];
		*/
		
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
		
		//now publish the sensor data
		sensordata_pub.publish(SensorMsg);
		
		/*
		printf("%d,%d,%d\n",
		standardSensorData->boardPowerVol,
		standardSensorData->motorPowerVol,
		standardSensorData->refVol);*/
		
		//Check motor timeouts
		if(motor_time&&motor_time<microtime())
		{
			ROS_INFO("Motor command expired.");
			motorcmd(0,0,0);
			motor_time=0;
		}

		//print_infrared();
		print_ultrasonic();

		//spinOnce is required so the callback finishes
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	
	}
	
	return 0;
}
void print_infrared(){
	//print out the sensor data messages for testing purposes
	ROS_INFO("Infrared range sensor 1 = %d", IR_Convert(rangeSensorData->irRangeSensor[0]));
	ROS_INFO("Infrared range sensor 2 = %d", IR_Convert(rangeSensorData->irRangeSensor[1]));
	ROS_INFO("Infrared range sensor 3 = %d", IR_Convert(rangeSensorData->irRangeSensor[2]));
	ROS_INFO("Infrared range sensor 4 = %d", IR_Convert(rangeSensorData->irRangeSensor[3]));
	ROS_INFO("Infrared range sensor 5 = %d", IR_Convert(rangeSensorData->irRangeSensor[4]));
	ROS_INFO("Infrared range sensor 6 = %d", IR_Convert(rangeSensorData->irRangeSensor[5]));
	ROS_INFO("Infrared range sensor 7 = %d", IR_Convert(rangeSensorData->irRangeSensor[6]));
}
void print_ultrasonic(){
	//print out the sensor data messages for testing purposes
	//ROS_INFO("Ultrasonic range sensor 1 = %d", (rangeSensorData->usRangeSensor[0]));
	//ROS_INFO("Ultrasonic range sensor 2 = %d", (rangeSensorData->usRangeSensor[1]));
	//ROS_INFO("Ultrasonic range sensor 3 = %d", (rangeSensorData->usRangeSensor[2]));
	ROS_INFO("Ultrasonic range sensor 4 = %d", (rangeSensorData->usRangeSensor[3]));
	ROS_INFO("Ultrasonic range sensor 5 = %d", (rangeSensorData->usRangeSensor[4]));
	ROS_INFO("Ultrasonic range sensor 6 = %d", (rangeSensorData->usRangeSensor[5]));
}

/*Convert the IR Sensor to a value in cm.  Note that this is currently not too accurate,
it is just an approximation for the voltage slope*/
int IR_Convert(int value){
	return 739.38*pow(value,-.8105);
}
/*Convert the Motor encoder values into cm/s values.
Diameter is 18 cm so circumference is about 56.5 cm. 800 ticks on each encoder
Motor_num is 0 for left and 1 for right on the X80SVP*/
double Motor_Convert(double motor_val){
	return (800/56.5) * motor_val;
}

long microtime()
{
	struct timeval t;
	gettimeofday(&t,0);
	return t.tv_sec*1000+t.tv_usec/1000;
}


