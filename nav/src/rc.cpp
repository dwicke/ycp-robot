/* Remote control.
 * Use arrow keys, Q exits.
 */

//Top speed, in cm/sec
#define SPEED		15

//Time interval the motor commands should be valid for
#define TIME		200




//////////



#include "ros/ros.h"

#include "robot_msgs/SensorData.h"
#include "robot_msgs/MotorData.h"


#include <stdio.h>
#include <stdlib.h>
#include <termios.h>

static struct termios oldt;

void restore_terminal_settings(void)
{
    tcsetattr(0, TCSANOW, &oldt);  /* Apply saved settings */
}

void disable_waiting_for_enter(void)
{
    struct termios newt;

    /* Make terminal read 1 char at a time */
    tcgetattr(0, &oldt);  /* Save terminal settings */
    newt = oldt;  /* Init new settings */
    newt.c_lflag &= ~(ICANON | ECHO);  /* Change settings */
    tcsetattr(0, TCSANOW, &newt);  /* Apply settings */
    atexit(restore_terminal_settings); /* Make sure settings will be restored when program ends  */
}


ros::Publisher motordata_pub;


int main(int argc, char **argv)
{
	ros::init(argc, argv, "nav");
	ros::NodeHandle n;
	
	//topic, buffer
	motordata_pub = n.advertise<robot_msgs::MotorData>("motordata", 1);
	
	//topic, buffer, callback
	//ros::Subscriber sensordata_sub = n.subscribe("sensordata", 1, sensordataCallback);
	
	ROS_INFO("RC started.\n\nQ exits.\n\n");
	
	int l,r;
	
	disable_waiting_for_enter();
	while(1)
	{
		switch(getchar())
		{
			case 65:l=+1;r=+1;break;
			case 66:l=-1;r=-1;break;
			case 68:l=-1;r=+1;break;
			case 67:l=+1;r=-1;break;
			case 'q':exit(1);
		}
		
		robot_msgs::MotorData motordata_msg;
		motordata_msg.motor_left_velocity	=l*SPEED;
		motordata_msg.motor_left_time		=TIME;
		motordata_msg.motor_right_velocity	=r*SPEED;
		motordata_msg.motor_right_time		=TIME;
		motordata_pub.publish(motordata_msg);
	}
	return 0;
}
