/* Bare minimum obstacle avoidence.
 * This uses only the frontLeftLeft and frontRightRight infrared sensors to avoid crashing while driving around aimlessly.
 */

///Parameters

//Threshold, in cm, that an object is "seen"
#define SEE_THRESH	150

//Threshold, in cm, that an object is too close (movement will be stopped to prevent collision)
#define ERR_THRESH	10

//Top speed, in cm/sec
#define SPEED		3

//Time interval the motor commands should be valid for
#define TIME		200

//If robot encounters a corner, it will get stuck- this controls how likely it is to turn around to get out of that situation (lower=more likely, higher=less likely)
#define stuck	100


//////////////////////////////////////////////////

#include "ros/ros.h"

#include "robot_msgs/SensorData.h"
#include "robot_msgs/MotorData.h"

ros::Publisher motordata_pub;


///Navi.bs2
//stuck  CON 100  '


int st=0;

//IR left and right "clear" flags
void nav(int il,int ir,int &sl,int &sr)
{
	st=st+(4-((il+ir)*2))*2;
	
	if( st>=stuck)// THEN
	{
		//FREQOUT spkrP,beep2,beepN,beepN
		
		//command ~90-180deg turn
		//????????????????
		
		ROS_INFO("!!!!!!!!!");
		robot_msgs::MotorData motordata_msg;
	
		
		//
		motordata_msg.motor_left_velocity	=-SPEED;
		motordata_msg.motor_left_time		=1100;
		motordata_msg.motor_right_velocity	=+SPEED;
		motordata_msg.motor_right_time		=1100;
		motordata_pub.publish(motordata_msg);
		
		ros::spinOnce();
		sleep(1);
		
		st=0;
		return;
	}//ENDIF
	
    //'DEBUG DEC st,CR
    //IF st THEN
    //  st=st-1
    ////ENDIF
	if(st)--st;

    if( il && ir)// THEN ' All clear!
	{
		sl=+1;
		sr=+1;
	}
    else if( ir==0)// THEN  ' Somthing ahead right...
	{
		sl=-1;
		sr=+1;
	}
    else if( il==0)// THEN  ' Somthing ahead left...
    {
		sl=+1;
		sr=-1;
	}//ENDIF

}

void thresh_err(const char * sensor,int val)
{
	ROS_INFO("SENSOR OUT OF RANGE: %s=%d", sensor,val);
}
void thresh_err(const char * sensor,float val)
{
	ROS_INFO("SENSOR OUT OF RANGE: %s=%.1f", sensor,val);
}

void sensordataCallback(const robot_msgs::SensorData::ConstPtr& msg)
{
	ROS_INFO(	"Got sensor packet:\n"
				"\tultrasonic: front[%d,%d,%d]|rear[%d,%d,%d]\n"
				"\tinfrared:   front[%.1f,%.1f,%.1f,%.1f]|middle[%.1f,%.1f]|rear[%.1f]\n"
				"\thuman:      left[%d,%d]|right[%d,%d]",
		msg->ultrasonic_frontLeft_distance,
		msg->ultrasonic_frontCenter_distance,
		msg->ultrasonic_frontRight_distance,
		msg->ultrasonic_rearLeft_distance,
		msg->ultrasonic_rearCenter_distance,
		msg->ultrasonic_rearRight_distance,

		msg->infrared_frontLeftLeft_distance,
		msg->infrared_frontLeftCenter_distance,
		msg->infrared_frontRightCenter_distance,
		msg->infrared_frontRightRight_distance,
		msg->infrared_right_distance,
		msg->infrared_rear_distance,
		msg->infrared_left_distance,

		msg->human_left_motion,
		msg->human_left_presence,
		msg->human_right_motion,
		msg->human_right_presence);
	
	
	robot_msgs::MotorData motordata_msg;
	
	//If simulator is functioning properly, these should never reach ERR_THRESH
	int trip=0;
	if(msg->ultrasonic_frontLeft_distance		<ERR_THRESH){++trip;thresh_err("ultrasonic_frontLeft_distance"		,msg->ultrasonic_frontLeft_distance);}
	if(msg->ultrasonic_frontCenter_distance		<ERR_THRESH){++trip;thresh_err("ultrasonic_frontCenter_distance"	,msg->ultrasonic_frontCenter_distance);}
	if(msg->ultrasonic_frontRight_distance		<ERR_THRESH){++trip;thresh_err("ultrasonic_frontRight_distance"		,msg->ultrasonic_frontRight_distance);}
	if(msg->ultrasonic_rearRight_distance		<ERR_THRESH){++trip;thresh_err("ultrasonic_rearRight_distance"		,msg->ultrasonic_rearRight_distance);}
	if(msg->ultrasonic_rearCenter_distance		<ERR_THRESH){++trip;thresh_err("ultrasonic_rearCenter_distance"		,msg->ultrasonic_rearCenter_distance);}
	if(msg->ultrasonic_rearLeft_distance		<ERR_THRESH){++trip;thresh_err("ultrasonic_rearLeft_distance"		,msg->ultrasonic_rearLeft_distance);}

	if(msg->infrared_frontLeftLeft_distance		<ERR_THRESH){++trip;thresh_err("infrared_frontLeftLeft_distance"	,msg->infrared_frontLeftLeft_distance);}//
	if(msg->infrared_frontLeftCenter_distance	<ERR_THRESH){++trip;thresh_err("infrared_frontLeftCenter_distance"	,msg->infrared_frontLeftCenter_distance);}
	if(msg->infrared_frontRightCenter_distance	<ERR_THRESH){++trip;thresh_err("infrared_frontRightCenter_distance"	,msg->infrared_frontRightCenter_distance);}
	if(msg->infrared_frontRightRight_distance	<ERR_THRESH){++trip;thresh_err("infrared_frontRightRight_distance"	,msg->infrared_frontRightRight_distance);}//
	if(msg->infrared_right_distance				<ERR_THRESH){++trip;thresh_err("infrared_right_distance"			,msg->infrared_right_distance);}
	if(msg->infrared_rear_distance				<ERR_THRESH){++trip;thresh_err("infrared_rear_distance"				,msg->infrared_rear_distance);}
	if(msg->infrared_left_distance				<ERR_THRESH){++trip;thresh_err("infrared_left_distance"				,msg->infrared_left_distance);}
	
	if(trip)
	{
		ROS_INFO("Abort!");
		
		//halt motors
		motordata_msg.motor_left_velocity	=0;
		motordata_msg.motor_left_time		=1000;
		motordata_msg.motor_right_velocity	=0;
		motordata_msg.motor_right_time		=1000;
		motordata_pub.publish(motordata_msg);
		
		ros::spinOnce();
		exit(1);
	}
	
	//run Nav
	int il=msg->infrared_frontLeftLeft_distance>=SEE_THRESH,
		ir=msg->infrared_frontRightRight_distance>=SEE_THRESH,
		sl=0,sr=0;
	
	nav(il,ir,sl,sr);
	
	ROS_INFO("Nav: (%c,%c) -> (%c,%c)",il?' ':'#',ir?' ':'#',sl==1?'^':'V',sr==1?'^':'V');
	
	motordata_msg.motor_left_velocity	=sl*SPEED;
	motordata_msg.motor_left_time		=TIME;
	motordata_msg.motor_right_velocity	=sr*SPEED;
	motordata_msg.motor_right_time		=TIME;
	motordata_pub.publish(motordata_msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "nav");
	ros::NodeHandle n;
	
	//topic, buffer
	motordata_pub = n.advertise<robot_msgs::MotorData>("motordata", 10);
	
	//topic, buffer, callback
	ros::Subscriber sensordata_sub = n.subscribe("sensordata", 10, sensordataCallback);
	
	ROS_INFO("Nav started.");
	
	ros::spin();
	
	return 0;
}