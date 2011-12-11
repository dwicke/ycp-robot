#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "ros/ros.h"
#include "robot_msgs/SensorData.h"
#include "robot_msgs/MotorData.h"
#include "DrRobotMotionSensorDriver.hpp"

//serial parameters
#define PORT "/dev/ttyUSB0"
#define BAUD 115200

//prototypes
float IR_Convert(int IRValue);
double Motor_Convert(double motor_val);
void print_ultrasonic();
void print_infrared();
long microtime();

using namespace DrRobot_MotionSensorDriver;

//globals
DrRobotMotionSensorDriver *driver; //serial driver needs global access

//make a range sensor data object and a standard sensor object
RangeSensorData *rangeSensorData = new RangeSensorData();
StandardSensorData *standardSensorData = new StandardSensorData();
CustomSensorData *customSensorData = new CustomSensorData();
long microtime(void);
int count = 0;
int countmax = 0;
long motor_time = 0;
int motor_count = 0;
int motor_threshold = 5;
float motor_left_velocity_prev = 0;
float motor_right_velocity_prev = 0;
using namespace DrRobot_MotionSensorDriver;


//callback for the motor Data
//if there is new information, it needs to be converted and sent to the robot
//motors are controlled by taking in a velocity value which is then set for the specific motor channel
void motorCallback(const robot_msgs::MotorData::ConstPtr& msg){
	//send every 10th motor value and check to see if new value is outside of threshold value range	
	if((motor_count % 10) == 0 && (msg->motor_left_velocity < motor_left_velocity_prev - motor_threshold || msg->motor_left_velocity > motor_left_velocity_prev + motor_threshold ||
		msg->motor_right_velocity < motor_right_velocity_prev - motor_threshold || msg->motor_right_velocity > motor_right_velocity_prev + motor_threshold)){ 
		ROS_INFO("Motor data received: \n");
		ROS_INFO("motor1: %lf", msg->motor_left_velocity);
		ROS_INFO("motor2: %lf", msg->motor_left_velocity);
		//send left/right values to the robot
		int motorPASS = driver->sendMotorCtrlAllCmd(Velocity, Motor_Convert(msg->motor_left_velocity), Motor_Convert(-1*msg->motor_right_velocity), 0,0,0,0,100);
	       	if(motorPASS < 0) ROS_INFO("Motor data was not sent to the Robot!");
		motor_left_velocity_prev = msg->motor_left_velocity;
		motor_right_velocity_prev = msg->motor_right_velocity;
	}
	count++;
}

//callback for the motor Data
//if there is new information, it needs to be converted and sent to the robot
//motors are controlled by taking in a velocity value which is then set for the specific motor channel

/*float lleft=-100,lright=-100;
void motorcmd(float left,float right)
{
        //Sending repeat commands seems so sometimes cause the robot to stop- avoid doing that
        if((left==lleft)&&(right==lright))
                return;
        lleft=left;count++;
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
}*/

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
        if(!sensordata_pub) ROS_INFO("Failed to create a publisher node!MotorCtrl");
        
        ros::Rate loop_rate(10); //10 Hz publish time
                        
        int count = 0; //count of how many messages were sent
        
        while(ros::ok())
        {
                //This is a message object, it is stuffed with the datas and then can be published
                robot_msgs::SensorData SensorMsg;

                //get the sensor data, return values are 6 Ultrasonic sensors and 10 IRs, X80SVP does not require all of these.
                driver->readRangeSensorData(rangeSensorData);
                driver->readStandardSensorData(standardSensorData);
		driver->readCustomSensorData(customSensorData);
                //set the sensor data

                //Ultrasonic: data is 8 bit unsigned, in cm's
                //SensorMsg.ultrasonic_frontLeft_distance = rangeSensorData->usRangeSensor[0];
                //SensorMsg.ultrasonic_frontCenter_distance = rangeSensorData->usRangeSensor[1];
                //SensorMsg.ultrasonic_frontRight_distance = rangeSensorData->usRangeSensor[2];
                SensorMsg.ultrasonic_frontRight_distance = rangeSensorData->usRangeSensor[3];
                SensorMsg.ultrasonic_frontCenter_distance = rangeSensorData->usRangeSensor[4];
                SensorMsg.ultrasonic_frontLeft_distance = rangeSensorData->usRangeSensor[5];
                
                //Infrared: data is floats
                SensorMsg.infrared_frontLeftLeft_distance = IR_Convert(rangeSensorData->irRangeSensor[0]);
                SensorMsg.infrared_frontLeftCenter_distance = IR_Convert(customSensorData->customADData[2]);
                SensorMsg.infrared_frontRightCenter_distance = IR_Convert(customSensorData->customADData[3]);
                SensorMsg.infrared_frontRightRight_distance = IR_Convert(customSensorData->customADData[4]);
                SensorMsg.infrared_right_distance = IR_Convert(customSensorData->customADData[5]);
                SensorMsg.infrared_rear_distance = IR_Convert(customSensorData->customADData[6]);
                SensorMsg.infrared_left_distance = IR_Convert(customSensorData->customADData[7]);

                //human sensor, returned from robot in a different message
                SensorMsg.human_left_motion = standardSensorData->humanSensorData[0];
                SensorMsg.human_left_presence = standardSensorData->humanSensorData[1];
                SensorMsg.human_right_motion = standardSensorData->humanSensorData[2];
                SensorMsg.human_right_presence = standardSensorData->humanSensorData[3];
                
		//print sensor data for debug        
		print_ultrasonic();
		print_infrared();

                //now publish the sensor data
                sensordata_pub.publish(SensorMsg);

                //spinOnce is required so the callback finishes
                ros::spinOnce();
                loop_rate.sleep();
                ++count;
        
        }
        
        return 0;
}
void print_infrared(){
        //print infrareds, 1 is stored in the range sensor, other is in custom data
	ROS_INFO("Infrared range sensor 1 = %f\n",  IR_Convert(rangeSensorData->irRangeSensor[0]));
	ROS_INFO("Infrared range sensor 2 = %f\n",  IR_Convert(customSensorData->customADData[2]));
     	ROS_INFO("Infrared range sensor 3 = %f\n",  IR_Convert(customSensorData->customADData[3]));
	ROS_INFO("Infrared range sensor 4 = %f\n",  IR_Convert(customSensorData->customADData[4]));
	ROS_INFO("Infrared range sensor 5 = %f\n",  IR_Convert(customSensorData->customADData[5]));
     	ROS_INFO("Infrared range sensor 6 = %f\n",  IR_Convert(customSensorData->customADData[6]));
	ROS_INFO("Infrared range sensor 7 = %f\n",  IR_Convert(customSensorData->customADData[7]));
}
void print_ultrasonic(){
        //print out the sensor data messages for testing purposes
        //ROS_INFO("Ultrasonic range sensor 1 = %d\n", (rangeSensorData->usRangeSensor[0]));
        //ROS_INFO("Ultrasonic range sensor 2 = %d\n", (rangeSensorData->usRangeSensor[1]));
        //ROS_INFO("Ultrasonic range sensor 3 = %d\n", (rangeSensorData->usRangeSensor[2]));
	
	//sensors were changed around due to bad sensor / control board issues at YCP
        ROS_INFO("Ultrasonic right = %d\n", (rangeSensorData->usRangeSensor[3]));
        ROS_INFO("Ultrasonic center = %d\n", (rangeSensorData->usRangeSensor[4]));
        ROS_INFO("Ultrasonic left = %d\n", (rangeSensorData->usRangeSensor[5]));
}

/*Convert the IR Sensor to a value in cm.  Note that this is currently not too accurate,
it is just an approximation for the voltage slope*/
float IR_Convert(int IRValue){
        double temp = 0;
    	double IRAD2Distance = 0;
    	temp = 21.6 / ((double)IRValue * 3 / 4028 - 0.17);

	// IR range 10-80cm
	if ((temp > 80) || (temp < 0))
		IRAD2Distance = 80;
	else if ((temp < 10) && (temp > 0))
		IRAD2Distance = 7.5;
	else
		IRAD2Distance = temp;
	return IRAD2Distance;
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
