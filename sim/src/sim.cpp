//This relays the ROS messages across a set of named pipes to/from the GL simulator. Nessecery because it won't build in the ROS environment.

#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/stat.h>

#include "ros/ros.h"

#include "robot_msgs/SensorData.h"
#include "robot_msgs/MotorData.h"


//This is duplicated in roscomm.h, should be made a common include.//
#define SENSORDATA_PIPE_NAME	"/tmp/sensordata"
#define MOTORDATA_PIPE_NAME		"/tmp/motordata"


struct MotorData{
	float motor_left_velocity;
	int motor_left_time;
	float motor_right_velocity;
	int motor_right_time;
};

struct SensorData{
	int ultrasonic_frontLeft_distance;
	int ultrasonic_frontCenter_distance;
	int ultrasonic_frontRight_distance;
	int ultrasonic_rearRight_distance;
	int ultrasonic_rearCenter_distance;
	int ultrasonic_rearLeft_distance;

	float infrared_frontLeftLeft_distance;
	float infrared_frontLeftCenter_distance;
	float infrared_frontRightCenter_distance;
	float infrared_frontRightRight_distance;
	float infrared_right_distance;
	float infrared_rear_distance;
	float infrared_left_distance;

	int human_left_motion;
	int human_left_presence;
	int human_right_motion;
	int human_right_presence;
};
/////////////////////////////////////////////////////////////////////


int motordata_pipe;

void motordataCallback(const robot_msgs::MotorData::ConstPtr& msg)
{
//	ROS_INFO("Got motor packet: %.1f:%d | %.1f:%d", msg->motor_left_velocity,msg->motor_left_time,msg->motor_right_velocity,msg->motor_right_time);
	
	//Populate named pipe struct with the values from the ROS message
	MotorData motordata;
	motordata.motor_left_velocity	=msg->motor_left_velocity;
	motordata.motor_left_time		=msg->motor_left_time;
	motordata.motor_right_velocity	=msg->motor_right_velocity;
	motordata.motor_right_time		=msg->motor_right_time;
	
	//Pass data on to sim via named pipe
	int ret=write(motordata_pipe,&motordata,sizeof(motordata));
	if(ret!=sizeof(motordata))
	{
		printf("Write error, closing.");
		exit(1);
	}
}

int main(int argc, char **argv)
{
	//Launch GL simulator
	int pid = fork();
	if (pid == -1)
	{
		printf("fork fail");
		return 1;
	}
	else if (pid == 0)
	{//child
		execl("/bin/bash","/bin/bash","_sim_launch.sh",(const char *)0);
		printf("execl fail\n");
		return 1;
	}
	
	//Create pipes- Execution will hang here till the GL sim opens its pipe
	ROS_INFO("Waiting for sim to start.");
	int sensordata_pipe, ret_val;

	ret_val = mkfifo(SENSORDATA_PIPE_NAME, 0666);
	if ((ret_val == -1) && (errno != EEXIST)) {
		perror("Error creating the named pipe");
		exit (1);
	}
	ret_val = mkfifo(MOTORDATA_PIPE_NAME, 0666);
	if ((ret_val == -1) && (errno != EEXIST)) {
		perror("Error creating the named pipe");
		exit (1);
	}
	sensordata_pipe = open(SENSORDATA_PIPE_NAME, O_RDONLY);
	motordata_pipe = open(MOTORDATA_PIPE_NAME, O_WRONLY);
	
	//Init ROS stuff
	ros::init(argc, argv, "sim");//name of the program as seen by ROS
	ros::NodeHandle n;
	
	//Advertise this program provides sensor data
	ros::Publisher sensor_pub = n.advertise<robot_msgs::SensorData>("sensordata", 10);
	
	//Subscribe this program to motor data
	ros::Subscriber motordata_sub = n.subscribe("motordata", 10, motordataCallback);
	
	
	ROS_INFO("Started sim comm.");
	
	//ros:ok() will return false if the ROS core node is shut down
	while (ros::ok())
	{
		robot_msgs::SensorData msg;
		
		//Retrieve sensordata from GL sim- this will block till it is actually available.
		SensorData sensordata;
		ret_val=read(sensordata_pipe, &sensordata, sizeof(sensordata));
		if(ret_val!=sizeof(sensordata))
		{
			ROS_INFO("Read error, closing.");
			exit(1);
		}
		
		//Populate the ROS message with the values from the named pipe struct
		msg.ultrasonic_frontLeft_distance		=sensordata.ultrasonic_frontLeft_distance;
		msg.ultrasonic_frontCenter_distance		=sensordata.ultrasonic_frontCenter_distance;
		msg.ultrasonic_frontRight_distance		=sensordata.ultrasonic_frontRight_distance;
		msg.ultrasonic_rearRight_distance		=sensordata.ultrasonic_rearRight_distance;
		msg.ultrasonic_rearCenter_distance		=sensordata.ultrasonic_rearCenter_distance;
		msg.ultrasonic_rearLeft_distance		=sensordata.ultrasonic_rearLeft_distance;

		msg.infrared_frontLeftLeft_distance		=sensordata.infrared_frontLeftLeft_distance;
		msg.infrared_frontLeftCenter_distance	=sensordata.infrared_frontLeftCenter_distance;
		msg.infrared_frontRightCenter_distance	=sensordata.infrared_frontRightCenter_distance;
		msg.infrared_frontRightRight_distance	=sensordata.infrared_frontRightRight_distance;
		msg.infrared_right_distance				=sensordata.infrared_right_distance;
		msg.infrared_rear_distance				=sensordata.infrared_rear_distance;
		msg.infrared_left_distance				=sensordata.infrared_left_distance;
		
		msg.human_left_motion					=sensordata.human_left_motion;
		msg.human_left_presence					=sensordata.human_left_presence;
		msg.human_right_motion					=sensordata.human_right_motion;
		msg.human_right_presence				=sensordata.human_right_presence;
		
		//Publish the sensordata message to ROS
		sensor_pub.publish(msg);
		//ROS_INFO("Sent sensor packet (read %d bytes).",ret_val);
		
		ros::spinOnce();
	}
	
	ROS_INFO("ROS has shut down");
	
	return 0;
}