
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


#define HUMAN_NULL 2024

ros::Publisher sensor_pub;
ros::Subscriber motordata_sub;

void motordataCallback(const robot_msgs::MotorData::ConstPtr& motordata)
{
	//ROS_INFO("Got motor packet: %.1f:%d | %.1f:%d", motordata->motor_left_velocity,motordata->motor_left_time,motordata->motor_right_velocity,motordata->motor_right_time);
	
	int ttime=glutGet(GLUT_ELAPSED_TIME);
	
	wheel_l_velocity=motor_sim(motordata->motor_left_velocity);
	wheel_l_end		=ttime+motordata->motor_left_time;
	wheel_r_velocity=motor_sim(motordata->motor_right_velocity);
	wheel_r_end		=ttime+motordata->motor_right_time;
}

void ros_init(int &argc, char **argv)
{
	//Init ROS stuff
	ros::init(argc, argv, "sim");//name of the program as seen by ROS
	ros::NodeHandle n;
	
	//Advertise this program provides sensor data
	sensor_pub = n.advertise<robot_msgs::SensorData>("sensordata", 1);
	
	//Subscribe this program to motor data
	motordata_sub = n.subscribe("motordata", 1, motordataCallback);
	
	
	ROS_INFO("Init'd sim ROS.");
}

void send_sensordata(robot_msgs::SensorData sensordata)
{
	//ros:ok() will return false if the ROS core node is shut down
	if(!ros::ok())
	{
		lerr("ROS is not OK!\n");
	}
	
	//Publish the sensordata message to ROS
	sensor_pub.publish(sensordata);
	
	//Let the motor callback run
	ros::spinOnce();
}

/*
void ros_get_motors(int ttime)
{
	MotorData motordata;
	int ret=read(motordata_pipe, &motordata, sizeof(motordata));
	
	if(ret==-1)
	{//no data to get
		if(errno==EAGAIN)return;
	}
	
	//printf("Got motor data (%d bytes)\n",ret);
	//printf("Got motor packet: %.1f:%d | %.1f:%d\n", motordata.motor_left_velocity,motordata.motor_left_time,motordata.motor_right_velocity,motordata.motor_right_time);
	
	if(ret!=sizeof(motordata))
	{
		printf("Read error, closing (%d).\n",errno);
		exit(1);
	}
	
	wheel_l_velocity=motor_sim(motordata.motor_left_velocity);
	wheel_l_end		=ttime+motordata.motor_left_time;
	wheel_r_velocity=motor_sim(motordata.motor_right_velocity);
	wheel_r_end		=ttime+motordata.motor_right_time;
}
*/
/*
void send_sensordata(robot_msgs::SensorData sensordata)
{
	sensor_pub.publish(sensordata);
	ros::spinOnce();
	//printf("Sent sensor dummy [%.2f,%.2f]\n",sensordata.infrared_frontLeftLeft_distance,sensordata.infrared_frontRightRight_distance);
}
*/



/*
void send_sensor_dummy()
{
	SensorData sensordata;
	sensordata.ultrasonic_frontLeft_distance=101;
	sensordata.ultrasonic_frontCenter_distance=102;
	sensordata.ultrasonic_frontRight_distance=103;
	sensordata.ultrasonic_rearRight_distance=104;
	sensordata.ultrasonic_rearCenter_distance=105;
	sensordata.ultrasonic_rearLeft_distance=106;

	sensordata.infrared_frontLeftLeft_distance=107;//
	sensordata.infrared_frontLeftCenter_distance=108;
	sensordata.infrared_frontRightCenter_distance=109;
	sensordata.infrared_frontRightRight_distance=110;//
	sensordata.infrared_right_distance=111;
	sensordata.infrared_rear_distance=112;
	sensordata.infrared_left_distance=113;

	sensordata.human_left_motion=131;
	sensordata.human_left_presence=132;
	sensordata.human_right_motion=133;
	sensordata.human_right_presence=134;
	
	
	float depth=0,data;
	glReadPixels(winx*.05,winy/2,1,1,GL_DEPTH_COMPONENT,GL_FLOAT,&data);
	depth=-1/log((data))/11.0;
	sensordata.infrared_frontLeftLeft_distance=depth*100;//
	
	glReadPixels(winx*.95,winy/2,1,1,GL_DEPTH_COMPONENT,GL_FLOAT,&data);
	depth=-1/log((data))/11.0;
	sensordata.infrared_frontRightRight_distance=depth*100;//
		
	int ret=write(sensordata_pipe,&sensordata,sizeof(sensordata));
	if(ret!=sizeof(sensordata))
	{
		printf("Write error, closing.");
		exit(1);
	}
	//printf("Sent sensor dummy [%.2f,%.2f]\n",sensordata.infrared_frontLeftLeft_distance,sensordata.infrared_frontRightRight_distance);
}*/
/*
void ros_do_callbacks()
{
	ros::spinOnce();
}

void motordataCallback(const robot_msgs::MotorData::ConstPtr& msg)
{
	ROS_INFO("Got motor packet: %.1f:%d | %.1f:%d", msg->motor_left_velocity,msg->motor_left_time,msg->motor_right_velocity,msg->motor_right_time);
	
	//update sim
}

void ros_init()
{
	ros::init(argc, argv, "talker");
	ros::NodeHandle n;
	
	sensor_pub = n.advertise<robot_msgs::SensorData>("sensordata", 10);
	
	ros::Subscriber motordata_sub = n.subscribe("motordata", 10, motordataCallback);
	
	ROS_INFO("ROS initialized.");
}


void send_sensor_dummy()
{
	if(!ros::ok())
	{
		printf("ROS is not OK!\n\n");
		exit(1);
	}
	robot_msgs::SensorData msg;
	
	//msg.ultrasonic_frontLeft_distance = 42;
	msg.ultrasonic_frontLeft_distance=101;
	msg.ultrasonic_frontCenter_distance=102;
	msg.ultrasonic_frontRight_distance=103;
	msg.ultrasonic_rearRight_distance=104;
	msg.ultrasonic_rearCenter_distance=105;
	msg.ultrasonic_rearLeft_distance=106;

	msg.infrared_frontLeftLeft_distance=107;
	msg.infrared_frontLeftCenter_distance=108;
	msg.infrared_frontRightCenter_distance=109;
	msg.infrared_frontRightRight_distance=110;
	msg.infrared_right_distance=111;
	msg.infrared_rear_distance=112;
	msg.infrared_left_distance=113;
	
	msg.human_left_motion=HUMAN_NULL;
	msg.human_left_presence=HUMAN_NULL;
	msg.human_right_motion=HUMAN_NULL;
	msg.human_right_presence=HUMAN_NULL;
	
	
	sensor_pub.publish(msg);
	ros::spinOnce();
}*/