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
int IR_Convert(int IRValue);
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

using namespace DrRobot_MotionSensorDriver;


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
                        
		print_ultrasonic();
		
		
		
		//print infrareds, 1 is stored in the range sensor, other is in custom data
		ROS_INFO("Infrared range sensor 1 = %d\n",  (rangeSensorData->irRangeSensor[0]));
        	ROS_INFO("Infrared range sensor 2 = %d\n",  (customSensorData->customADData[2]));
  	     	ROS_INFO("Infrared range sensor 3 = %d\n",  (customSensorData->customADData[3]));
		ROS_INFO("Infrared range sensor 4 = %d\n",  (customSensorData->customADData[4]));
        	ROS_INFO("Infrared range sensor 5 = %d\n",  (customSensorData->customADData[5]));
  	     	ROS_INFO("Infrared range sensor 6 = %d\n",  (customSensorData->customADData[6]));
		ROS_INFO("Infrared range sensor 7 = %d\n",  (customSensorData->customADData[7]));


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
        //print out the sensor data messages for testing purposes
        //ROS_INFO("Infrared range sensor 1 = %d\n",  SensorMsg.infrared_frontLeftLeft_distance);
        //ROS_INFO("Infrared range sensor 2 = %d\n", SensorMsg.infrared_frontLeftCenter_distance);
        //ROS_INFO("Infrared range sensor 3 = %d\n",  SensorMsg.infrared_frontRightCenter_distance);
        ROS_INFO("Infrared range sensor 4 = %d\n", (rangeSensorData->irRangeSensor[3]));
        ROS_INFO("Infrared range sensor 5 = %d\n", (rangeSensorData->irRangeSensor[4]));
        ROS_INFO("Infrared range sensor 6 = %d\n", (rangeSensorData->irRangeSensor[5]));
        ROS_INFO("Infrared range sensor 7 = %d\n", (rangeSensorData->irRangeSensor[6]));
}
void print_ultrasonic(){
        //print out the sensor data messages for testing purposes
        //ROS_INFO("Ultrasonic range sensor 1 = %d\n", (rangeSensorData->usRangeSensor[0]));
        //ROS_INFO("Ultrasonic range sensor 2 = %d\n", (rangeSensorData->usRangeSensor[1]));
        //ROS_INFO("Ultrasonic range sensor 3 = %d\n", (rangeSensorData->usRangeSensor[2]));
	//sensors were changed around due to bad sensor / control board issues at YCP
        ROS_INFO("Ultrasonic range sensor 1 = %d\n", (rangeSensorData->usRangeSensor[3]));
        ROS_INFO("Ultrasonic range sensor 2 = %d\n", (rangeSensorData->usRangeSensor[4]));
        ROS_INFO("Ultrasonic range sensor 3 = %d\n", (rangeSensorData->usRangeSensor[5]));
}

/*Convert the IR Sensor to a value in cm.  Note that this is currently not too accurate,
it is just an approximation for the voltage slope*/
int IR_Convert(int IRValue){
        double temp = 0;
    	double IRAD2Distance = 0;
    	temp = 21.6 / ((double)IRValue * 3 / 4028 - 0.17);

	// IR range 10-80cm
	if ((temp > 80) || (temp < 0))
		IRAD2Distance = 0.81;
	else if ((temp < 10) && (temp > 0))
		IRAD2Distance = 0.09;
	else
		IRAD2Distance = temp / 100;
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
