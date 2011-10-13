//This just communicates dummy sensor data to the nav/other control program.
//Might be useful as an example for how to do the actual sensor lib.

#include "ros/ros.h"

#include "robot_msgs/SensorData.h"
#include "robot_msgs/MotorData.h"

void motordataCallback(const robot_msgs::MotorData::ConstPtr& msg)
{
	//Got motor packet
	ROS_INFO("Got motor packet: %.1f:%d | %.1f:%d", msg->motor_left_velocity,msg->motor_left_time,msg->motor_right_velocity,msg->motor_right_time);

	//Do something with it
}

void frontObstacle(ros::Publisher sensor_pub)
{

	//Sensor data message
	robot_msgs::SensorData msg;
	//Populate with values
	msg.ultrasonic_frontLeft_distance=9;
	msg.ultrasonic_frontCenter_distance=8;
	msg.ultrasonic_frontRight_distance=9;
	msg.ultrasonic_rearRight_distance=104;
	msg.ultrasonic_rearCenter_distance=105;
	msg.ultrasonic_rearLeft_distance=106;

	msg.infrared_frontLeftLeft_distance=9;
	msg.infrared_frontLeftCenter_distance=8;
	msg.infrared_frontRightCenter_distance=9;
	msg.infrared_frontRightRight_distance=9;
	msg.infrared_right_distance=75;
	msg.infrared_rear_distance=75;
	msg.infrared_left_distance=75;
	sensor_pub.publish(msg);
	ROS_INFO("Sent front obstacle sensor packet.");

}

void freeSpace(ros::Publisher sensor_pub)
{
	//Sensor data message
	robot_msgs::SensorData msg;

	//Populate with values
	msg.ultrasonic_frontLeft_distance=255;
	msg.ultrasonic_frontCenter_distance=255;
	msg.ultrasonic_frontRight_distance=255;
	msg.ultrasonic_rearRight_distance=255;
	msg.ultrasonic_rearCenter_distance=255;
	msg.ultrasonic_rearLeft_distance=255;

	msg.infrared_frontLeftLeft_distance=80;
	msg.infrared_frontLeftCenter_distance=80;
	msg.infrared_frontRightCenter_distance=80;
	msg.infrared_frontRightRight_distance=80;
	msg.infrared_right_distance=80;
	msg.infrared_rear_distance=80;
	msg.infrared_left_distance=80;

	msg.human_left_motion=301;
	msg.human_left_presence=302;
	msg.human_right_motion=303;
	msg.human_right_presence=304;

	//Send sensor packet
	sensor_pub.publish(msg);
	ROS_INFO("Sent dummy sensor packet.");
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "simtest");//name of the program as seen by ROS
	ros::NodeHandle n;

	//Advertise this program provides sensor data
	ros::Publisher sensor_pub = n.advertise<robot_msgs::SensorData>("sensordata", 10);

	//Subscribe this program to motor data
	ros::Subscriber motordata_sub = n.subscribe("motordata", 10, motordataCallback);

	//Send message every 10hz (note: sim.cpp shows how to do this reading on demand/from a blocking device)
	ros::Rate loop_rate(5);

	//ros:ok() will return false if the ROS core node is shut down
	while (ros::ok())
	{

		//frontObstacle(sensor_pub);
		freeSpace(sensor_pub);
		/*
		//Sensor data message
		robot_msgs::SensorData msg;

		//Populate with values
		msg.ultrasonic_frontLeft_distance=101;
		msg.ultrasonic_frontCenter_distance=102;
		msg.ultrasonic_frontRight_distance=103;
		msg.ultrasonic_rearRight_distance=104;
		msg.ultrasonic_rearCenter_distance=105;
		msg.ultrasonic_rearLeft_distance=106;

		msg.infrared_frontLeftLeft_distance=75;
		msg.infrared_frontLeftCenter_distance=75;
		msg.infrared_frontRightCenter_distance=75;
		msg.infrared_frontRightRight_distance=75;
		msg.infrared_right_distance=75;
		msg.infrared_rear_distance=75;
		msg.infrared_left_distance=75;

		msg.human_left_motion=301;
		msg.human_left_presence=302;
		msg.human_right_motion=303;
		msg.human_right_presence=304;

		//Send sensor packet
		sensor_pub.publish(msg);
		ROS_INFO("Sent dummy sensor packet.");
*/
		//Run pending motor callbacks
		ros::spinOnce();

		loop_rate.sleep();
	}

	ROS_INFO("ROS has shut down");

	return 0;
}
