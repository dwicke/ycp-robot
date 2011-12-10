//ROS communication
// Cory Boyle 2011

#include "robot_msgs/SensorData.h"
#include "robot_msgs/MotorData.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"

#define HUMAN_NULL 2024

ros::Publisher  sensor_pub,
				rgb_pub,
				depth_raw_pub,depth_float_pub;
ros::Subscriber motordata_sub;
ros::Publisher pointcloud_pub;

//ROS topic status
enum rostopic_mode{
	ENABLE	=0,
	DISABLE	=1,
	FORCE	=2
};

enum rostopic_mode rostopic_sensors_mode	=ENABLE;
enum rostopic_mode rostopic_video_mode		=ENABLE;
enum rostopic_mode rostopic_pointcloud_mode	=ENABLE;

int rostopic_sensors_subscribed		=false,
	rostopic_rgb_subscribed			=false,
	rostopic_depth_raw_subscribed	=false,
	rostopic_depth_float_subscribed	=false,
	rostopic_pointcloud_subscribed	=false;

#include "pointcloud.h"

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
	sensor_pub = n.advertise<robot_msgs::SensorData>("sensordata",1);
	
	
	rgb_pub			=n.advertise<sensor_msgs::Image>(ROSTOPIC_RGB_NAME			,1);
	depth_raw_pub	=n.advertise<sensor_msgs::Image>(ROSTOPIC_DEPTH_RAW_NAME	,1);
	depth_float_pub	=n.advertise<sensor_msgs::Image>(ROSTOPIC_DEPTH_FLOAT_NAME	,1);
	
	pointcloud_pub	=n.advertise<sensor_msgs::PointCloud2>(ROSTOPIC_POINTCLOUD_NAME,1);
	
	//Subscribe this program to motor data
	motordata_sub = n.subscribe("motordata", 1, motordataCallback);
	
	
	ROS_INFO("Init'd sim ROS.");
}

void ros_get_subscribers()
{//Should be possible to do this as a callback
	rostopic_sensors_subscribed		=sensor_pub.getNumSubscribers();
	rostopic_rgb_subscribed			=rgb_pub.getNumSubscribers();
	rostopic_depth_raw_subscribed	=depth_raw_pub.getNumSubscribers();
	rostopic_depth_float_subscribed	=depth_float_pub.getNumSubscribers();
	rostopic_pointcloud_subscribed	=pointcloud_pub.getNumSubscribers();
}

std::vector<uint8_t> vec_encode(unsigned char *buf,int length)
{
	std::vector<uint8_t> vec;
	vec.insert(vec.end(),buf,buf+length);
	return vec;
}

void ros_publish_rgb()
{
	sensor_msgs::Image imagedata;
	
	std::vector<uint8_t> data=vec_encode(kinect_rgb,SENSOR_WINDOW_WIDTH*SENSOR_WINDOW_HEIGHT*3);
	imagedata.height=SENSOR_WINDOW_HEIGHT;
	imagedata.width=SENSOR_WINDOW_WIDTH;
	imagedata.step=SENSOR_WINDOW_WIDTH*3;
	imagedata.encoding=sensor_msgs::image_encodings::RGB8;
	imagedata.is_bigendian=0;//doesnt matter here
	imagedata.data=data;
	
	rgb_pub.publish(imagedata);
}

void ros_publish_depth_raw()
{
	sensor_msgs::Image imagedata;
	
	std::vector<uint8_t> data=vec_encode((unsigned char*)kinect_depthmap_raw,SENSOR_WINDOW_WIDTH*SENSOR_WINDOW_HEIGHT*2);
	imagedata.height=SENSOR_WINDOW_HEIGHT;
	imagedata.width=SENSOR_WINDOW_WIDTH;
	imagedata.step=SENSOR_WINDOW_WIDTH*2;
	imagedata.encoding=sensor_msgs::image_encodings::TYPE_16UC1;
	imagedata.is_bigendian=0;//check this
	imagedata.data=data;
	
	depth_raw_pub.publish(imagedata);
}

void ros_publish_depth_float()
{
	//FIXME: Not Implemented.
	printf("ros_publish_depth_float() Not Implemented!\n");
}

/*
void send_imagedata(int bpp,const char* encoding,void *data)
{
	printf("sent rgb dummy\n");
	sensor_msgs::Image imagedata;
	
/ *
Header header
    uint32 seq
    time stamp
    string frame_id
uint32 height
uint32 width
string encoding
uint8 is_bigendian
uint32 step
uint8[] data
* /
	//char orig_data[3]{1,33,7};
	
	std::vector<uint8_t> data=vec_encode(data,SENSOR_WINDOW_WIDTH*SENSOR_WINDOW_HEIGHT*bpp);
	//std::vector<uint8_t> data;
	imagedata.height=SENSOR_WINDOW_HEIGHT;
	imagedata.width=SENSOR_WINDOW_WIDTH;
	imagedata.step=SENSOR_WINDOW_WIDTH*bpp;
	imagedata.encoding=encoding;
	imagedata.is_bigendian=0;//doesnt matter here
	imagedata.data=data;
	
	rgb_pub.publish(imagedata);

}*/


/*
void send_imagedata()
{
	printf("sent rgb dummy\n");
	sensor_msgs::Image rgbdata;
	
/ *
Header header
    uint32 seq
    time stamp
    string frame_id
uint32 height
uint32 width
string encoding
uint8 is_bigendian
uint32 step
uint8[] data
* /
	//char orig_data[3]{1,33,7};
	
	std::vector<uint8_t> data=vec_encode(kinect_rgb,SENSOR_WINDOW_WIDTH*SENSOR_WINDOW_HEIGHT*3);
	//std::vector<uint8_t> data;
	rgbdata.height=SENSOR_WINDOW_HEIGHT;
	rgbdata.width=SENSOR_WINDOW_WIDTH;
	rgbdata.step=SENSOR_WINDOW_WIDTH*3;
	rgbdata.encoding=sensor_msgs::image_encodings::RGB8;
	rgbdata.is_bigendian=0;//doesnt matter here
	rgbdata.data=data;
	
	rgb_pub.publish(rgbdata);

}*/


void ros_send_sensordata(robot_msgs::SensorData sensordata)
{
	//Publish the sensordata message to ROS
	sensor_pub.publish(sensordata);
}

void ros_do_callbacks()
{
	//ros:ok() will return false if the ROS core node is shut down
	if(!ros::ok())
	{
		lerr("ROS is not OK!\n");
	}
	
	//Let the motor callback run
	ros::spinOnce();
}