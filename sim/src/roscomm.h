//ROS communication
// Cory Boyle 2011

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