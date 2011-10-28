/* (Try to) follow heat.
 * The robot should turn towards the heat source and remain pointing at it.
 * This doesn't work very well currently- see human.h.
 */

//Top speed, in cm/sec
#define SPEED		5

//Time interval the motor commands should be valid for
#define TIME		200

//Threshold to "see" heat, in arbitrary units (0-1000 or so)
#define thresh	200.0

//Comment out to disable sending of actual motor commands
#define GO		1




//////////


#include "ros/ros.h"

#include "robot_msgs/SensorData.h"
#include "robot_msgs/MotorData.h"

#include "human.h"

ros::Publisher motordata_pub;


#define LEFT	-1
#define RIGHT	+1

int dir=0;

void sensordataCallback(const robot_msgs::SensorData::ConstPtr& msg)
{
	printf(	"left[%d,%d]|right[%d,%d]\n",
		msg->human_left_motion,
		msg->human_left_presence,
		msg->human_right_motion,
		msg->human_right_presence);
	
	//Process and print human data
	float ldelta,rdelta;
	humanmotion(msg->human_left_presence,msg->human_right_presence,ldelta,rdelta);
	hmdebug();
	
	//Values below thresh are clipped to zero, as these are most likely low-level noise.
	float left=ldelta-thresh,right=rdelta-thresh;
	if(left<0)left=0;
	if(right<0)right=0;
	
	printf("[%c,%c] ",
		left>0?'#':' ',
		right>0?'#':' '
	);
	
	robot_msgs::MotorData motordata_msg;
	
	if(left>right)
	{//turn left if there is heat to the left
		printf("< <");
		
		motordata_msg.motor_left_velocity	=-SPEED;
		motordata_msg.motor_left_time		=TIME;
		motordata_msg.motor_right_velocity	=+SPEED;
		motordata_msg.motor_right_time		=TIME;
	}
	else
	{//turn right otherwise
		printf("> >");
		
		motordata_msg.motor_left_velocity	=+SPEED;
		motordata_msg.motor_left_time		=TIME;
		motordata_msg.motor_right_velocity	=-SPEED;
		motordata_msg.motor_right_time		=TIME;
	}
	
#ifdef GO
	motordata_pub.publish(motordata_msg);
#endif
	printf("\n");

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "nav");
	ros::NodeHandle n;
	
	//topic, buffer
	motordata_pub = n.advertise<robot_msgs::MotorData>("motordata", 1);
	
	//topic, buffer, callback
	ros::Subscriber sensordata_sub = n.subscribe("sensordata", 1, sensordataCallback);
	
	ROS_INFO("Follow started.");
	
	ros::spin();
	
	return 0;
}
