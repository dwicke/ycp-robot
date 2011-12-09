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

//There are some hardware issues with the sensors that we encountered with our X80SVP that this flag will account for.
//For all other builds, set this to 0
#define YCP_ROBOT 1

//prototypes
int IR_Convert(int value);
double Motor_Convert(double motor_val);
void print_infrared(void);
void print_ultrasonic(void);
void get_usStatistics(void);
long microtime(void);

using namespace DrRobot_MotionSensorDriver;

//globals
DrRobotMotionSensorDriver *driver; //serial driver needs global access
int count = 0;
int countmax = 0;
long motor_time = 0;
//make a range sensor data object and a standard sensor object
RangeSensorData *rangeSensorData = new RangeSensorData();
StandardSensorData *standardSensorData = new StandardSensorData();

robot_msgs::SensorData SensorMsg;  //create a message struct for publishing

//callback for the motor Data
//if there is new information, it needs to be converted and sent to the robot
//motors are controlled by taking in a velocity value which is then set for the specific motor channel

float lleft=-100,lright=-100;
void motorcmd(float left,float right)
{
	//Sending repeat commands seems so sometimes cause the robot to stop- avoid doing that
	if((left==lleft)&&(right==lright))
		return;
	lleft=left;
	lright=right;
	
	//Send motor commands as one message
	int ret=driver->sendMotorCtrlAllCmd(Velocity,Motor_Convert(left),Motor_Convert(-right),0,0,0,0,0);//last param is time, seems to be broken
	if(ret<0)
	{
		ROS_INFO("motor value was not sent to the robot!");
		//exit(1);
	}
}

void motorCallback(const robot_msgs::MotorData::ConstPtr& msg){
	//ROS_INFO("Motor data received: \n");
	ROS_INFO("Motor data received: [%.0f,%.0f]:%d",msg->motor_left_velocity,msg->motor_right_velocity,msg->motor_left_time);
	
	motorcmd(msg->motor_left_velocity,msg->motor_right_velocity);
	motor_time=microtime()+msg->motor_left_time;
	
	return;
}

/*
void motorCallback(const robot_msgs::MotorData::ConstPtr& msg){
        ROS_INFO("Motor data received: \n");
        ROS_INFO("motor1: %lf", msg->motor_left_velocity);
        ROS_INFO("motor2: %lf", Motor_Convert(msg->motor_left_velocity));
        //send left/right values to the robot
        int motorLPASS = driver->sendMotorCtrlCmd(Velocity, 0, Motor_Convert(msg->motor_left_velocity), msg->motor_left_time);
        int motorRPASS;
        
        if(motorLPASS < 0) ROS_INFO("Left motor value was not sent to the robot!");                     
        if(msg->motor_right_velocity>0) motorRPASS = driver->sendMotorCtrlCmd(Velocity, 1, Motor_Convert((msg->motor_right_velocity)*-1-RIGHT_MOTOR_OFFSET), msg->motor_right_time); 
        else motorRPASS = driver->sendMotorCtrlCmd(Velocity, 1, Motor_Convert((msg->motor_right_velocity)*-1), msg->motor_right_time); 
        if(motorRPASS < 0) ROS_INFO("Right motor value was not sent to the robot!");

	

}
*/

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
        
	//Command a stop so the initial motor state is consistent
	motorcmd(0,0);
	
        while(ros::ok())
        {
        
                
                //get the sensor data, return values are 6 Ultrasonic sensors and 10 IRs, X80SVP does not require all of these.
                driver->readRangeSensorData(rangeSensorData);
                driver->readStandardSensorData(standardSensorData);

                //set the sensor data

                //Ultrasonic: data is 8 bit unsigned, in cm's
                //These sensors are switched around due to a control board issue on our specific 
#if YCP_ROBOT
                SensorMsg.ultrasonic_frontLeft_distance = rangeSensorData->usRangeSensor[5];
                SensorMsg.ultrasonic_frontCenter_distance = rangeSensorData->usRangeSensor[4];
                SensorMsg.ultrasonic_frontRight_distance = rangeSensorData->usRangeSensor[3];
#else
                SensorMsg.ultrasonic_frontLeft_distance = rangeSensorData->usRangeSensor[0];
                SensorMsg.ultrasonic_frontCenter_distance = rangeSensorData->usRangeSensor[1];
                SensorMsg.ultrasonic_frontRight_distance = rangeSensorData->usRangeSensor[2];
                SensorMsg.ultrasonic_rearRight_distance = rangeSensorData->usRangeSensor[3];
                SensorMsg.ultrasonic_rearCenter_distance = rangeSensorData->usRangeSensor[4];
                SensorMsg.ultrasonic_rearLeft_distance = rangeSensorData->usRangeSensor[5];
#endif
                
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
                        
                //print out sensor data
		/*#if YCP_ROBOT
		ROS_INFO("Ultrasonic range sensor 4 = %d\n", SensorMsg.ultrasonic_rearRight_distance); 
		ROS_INFO("Ultrasonic range sensor 5 = %d\n", SensorMsg.ultrasonic_rearCenter_distance);
		ROS_INFO("Ultrasonic range sensor 6 = %d\n", SensorMsg.ultrasonic_rearLeft_distance);
		#else
		ROS_INFO("Ultrasonic range sensor 1 = %d\n", SensorMsg.ultrasonic_frontLeft_distance);
		ROS_INFO("Ultrasonic range sensor 2 = %d\n", SensorMsg.ultrasonic_frontCenter_distance);
		ROS_INFO("Ultrasonic range sensor 3 = %d\n", SensorMsg.ultrasonic_frontRight_distance);
		ROS_INFO("Ultrasonic range sensor 4 = %d\n", SensorMsg.ultrasonic_rearRight_distance); 
		ROS_INFO("Ultrasonic range sensor 5 = %d\n", SensorMsg.ultrasonic_rearCenter_distance);
		ROS_INFO("Ultrasonic range sensor 6 = %d\n", SensorMsg.ultrasonic_rearLeft_distance);
        	#endif*/
			
		//print out the sensor data messages for testing purposes
		/*ROS_INFO("Infrared range sensor 1 = %d\n", IR_Convert(rangeSensorData->irRangeSensor[0]));
		ROS_INFO("Infrared range sensor 2 = %d\n", IR_Convert(rangeSensorData->irRangeSensor[1]));
		ROS_INFO("Infrared range sensor 3 = %d\n", IR_Convert(rangeSensorData->irRangeSensor[2]));
		ROS_INFO("Infrared range sensor 4 = %d\n", IR_Convert(rangeSensorData->irRangeSensor[3]));
		ROS_INFO("Infrared range sensor 5 = %d\n", IR_Convert(rangeSensorData->irRangeSensor[4]));
		ROS_INFO("Infrared range sensor 6 = %d\n", IR_Convert(rangeSensorData->irRangeSensor[5]));
		ROS_INFO("Infrared range sensor 7 = %d\n", IR_Convert(rangeSensorData->irRangeSensor[6]));
		*/
		
		 //If the motor timeouts are exceeded, stop the motors
		if(motor_time&&(motor_time<microtime()))  
		{
				ROS_INFO("Motor command expired.");
				//driver->sendMotorCtrlCmd(Velocity, 0, 0, 1);
				motorcmd(0,0);
				motor_time=0;
		}


                //now publish the sensor data
                sensordata_pub.publish(SensorMsg);

                //spinOnce is required so the callback finishes
                ros::spinOnce();
                loop_rate.sleep();
                ++count;
        
        }
        return 0;
}


void get_usStatistics(){
        //for developing the filter algorithms, we should begin by calculating how many occurances of 255 are seen
        //out of 1000 samples and how many are detections, missed detections, and false detections.
        //this will focus on using one sensor at a time
        
        //if 1000 samples are taken, calculate statistics and be done
        if(count >=1000){
                //done taking dataa
                ROS_INFO("255's seen: %d", countmax);
                ROS_INFO("samples taken: %d", count);
                ROS_INFO("percent max: %d", (countmax/count)*100);
        }
        else{
                ROS_INFO("Ultrasonic range sensor 5 = %d\n", SensorMsg.ultrasonic_rearCenter_distance); 
                count++;
                //if there is a 255 seen, increase the max count
                if(SensorMsg.ultrasonic_rearCenter_distance == 255) countmax++;
        }
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
 

