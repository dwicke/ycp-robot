//Kinect navigation
// Cory Boyle 2011

//This just uses the depthmap to attempt to drive toward open space.
//No filtering is done yet, so this probably won't work on actual hardware- this is mainly to test the simulator.


//Size of the depthmap
#define MAP_WIDTH			640
#define MAP_HEIGHT			480

//Size of the depthmap area to use as a window for range-finding.
#define WINDOW_TOP			MAP_HEIGHT/2-80		//Starting row for windows (px)
#define WINDOW_HEIGHT		40					//Number of rows for windows (px)

//X-Blur factor for window
#define BOX_BLUR			100		//px

#define MOTOR_SPEED			75		//cm/s
#define MOTOR_TIMEOUT		200		//ms

//Turning range in mm
#define TURN_RANGE_FAR		3000
#define TURN_RANGE_CLOSE	1500

//FPS target, frames won't get shown and ROS data wont get processed faster than this
#define	FPS	60



#ifdef OSX
	#include <GLUT/glut.h>
#else
//	#include <GL/glew.h>
	#include <GL/glut.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <ctype.h>
#include <stdio.h>


#include <SOIL/SOIL.h>


#include "ros/ros.h"

#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/image_encodings.h"


#include "robot_msgs/MotorData.h"

ros::Publisher motordata_pub;


#define	TIMER_ID	1


#define MAX_LEN	512


//for va_list
#include <stdarg.h>

//this is what causes ROS to explode
#include "glx.h"

int winx=0,winy=0;

void reshape(int w, int h)
{
	// Set new screen extents
	glViewport(0,0,w,h);

	//aspect=((GLfloat)w)/((GLfloat)h);
	winx=w;
	winy=h;
	
	//redraw
	glutPostRedisplay();
}

//orthographic, pixel coordinates
void raster()
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0,winx,winy,0,-1,1);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void raster_flipped()
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0,winx,0,winy,-1,1);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

unsigned char *image=0;
int image_x=0,image_y=0;
GLenum image_format=0,image_type=0;
//char rgb[IMAGE_HEIGHT*IMAGE_WIDTH*3];
void draw_rgb()
{
	//glBindTexture(GL_TEXTURE_2D,0);
	
	if(!image)
		return;
	
	glPixelZoom((float)winx/(float)image_x,-(float)winy/(float)image_y);
	
	raster_flipped();
	//raster();
	
	glRasterPos2f(0,winy);
	
	glDrawPixels(image_x,image_y,image_format,image_type,image);

}


int count=0;
void display()
{
	glClear(GL_COLOR_BUFFER_BIT);
	
	draw_rgb();
	
	raster();
	glColor3f(0,1,0);
	glxPrintf(4,24,"%d",count);
	count++;
	
	glFlush();
	glutSwapBuffers();
}

void timer(int id)
{
	glutTimerFunc(1000/FPS,timer,TIMER_ID);
	//glutPostRedisplay();
	ros::spinOnce();
}


ros::Subscriber rgbdata_sub;
ros::Subscriber pointcloud_sub;

size_t lsize=-1;
void sizebuf(size_t size)
{
	if(size!=lsize)
	{//(re)allocate buffer (should only ever happen once)
		printf("Allocating buffer of size %d.\n",size);
		if(image)
			free(image);
		image=(unsigned char*)malloc(size);
		lsize=size;
	}
}


void rgbdataCallback(const sensor_msgs::Image::ConstPtr& im)
{
	std::vector<uint8_t> data=im->data;
	size_t size=data.size();
	
	image_x=im->width;
	image_y=im->height;
	
	if(lsize==-1)
	{
		printf("Format: %s %dx%d (%d bpp + %d, %s-endian)\n",im->encoding.c_str(),image_x,image_y,(int)size/(image_x*image_y),(int)size%(image_x*image_y),
			im->is_bigendian==1?"big":"little"
		);
	}
	

	
	const unsigned char *buf=data.data();
	
	if(!(im->encoding==sensor_msgs::image_encodings::TYPE_16SC1||im->encoding==sensor_msgs::image_encodings::TYPE_16UC1))
	{
		printf("Bad encoding (wrong topic?)\n");
		exit(1);
	}
	if((int)size!=MAP_WIDTH*MAP_HEIGHT*2)
	{
		printf("Invalid buffer size: %d (wrong topic or image size)\n",(int)size);
		exit(1);
	}
	
	sizebuf(size);
	memcpy(image,buf,size);
	
	uint16_t *imbuf=(uint16_t*)image;
	
	//Collapse the window area to one line
	uint32_t line[MAP_WIDTH];
	memset(line,0,MAP_WIDTH*sizeof(uint32_t));
	for(int y=0;y<WINDOW_HEIGHT;y++)
	{
		for(int x=0;x<MAP_WIDTH;x++)
		{
			line[x]+=imbuf[(y+WINDOW_TOP)*MAP_WIDTH+x];
		}
	}
	
	//Box blur and normalize
	uint16_t window[MAP_WIDTH];
	
	//(this should probably use Gaussian blur not box blur)
	for(int i=0;i<MAP_WIDTH;i++)
	{
		int start=i-BOX_BLUR/2,
			end=i+BOX_BLUR/2-1;
		
		if(start<0)start=0;
		if(end>MAP_WIDTH)end=MAP_WIDTH;
		
		uint32_t avg=0,div=0,
		div2=end-start;
		
		for(int j=start;j<end;j++)
		{
			avg+=line[j];
			div++;
		}
		if(div!=div2)
			printf("%d,%d\n",div,div2);
		window[i]=avg/(div*WINDOW_HEIGHT);
	}
	
	//Write back to image for debug
	for(int y=0;y<WINDOW_HEIGHT;y++)
	{
		for(int x=0;x<MAP_WIDTH;x++)
		{
			imbuf[(y+(MAP_HEIGHT-WINDOW_HEIGHT))*MAP_WIDTH+x]=window[x];
		}
	}
	
	float center_heading=0;
	int hmax=0;
	
	
	//find heading
	for(int i=0;i<MAP_WIDTH;i++)
	{
		int this_heading=i-MAP_WIDTH/2;
		
		int value=window[i];
		
		if(hmax<value)
		{
			hmax=value;
			center_heading=((float)this_heading)/(MAP_WIDTH/2.0);
		}
	}
	
	
	float turn_heading=0;
	if(hmax<=TURN_RANGE_FAR)
	{
		int f=(TURN_RANGE_FAR-TURN_RANGE_CLOSE) - (hmax-TURN_RANGE_CLOSE);
		turn_heading=(((float)f)/((float)(TURN_RANGE_FAR-TURN_RANGE_CLOSE)));
		if(turn_heading>1)
		{
			turn_heading=1;
		}
	}
	

	
	float heading=center_heading+turn_heading;
	
	if(heading>+1)heading=+1;
	if(heading<-1)heading=-1;
	
	
	printf("Heading: %+4.02f | Center: %+4.02f | Turn: %+4.02f\n",heading,center_heading,turn_heading);
	
	float want_heading=heading*(MAP_WIDTH/2.0);
	
	//int left =MOTOR_SPEED+want_heading*5,
	//	right=MOTOR_SPEED-want_heading*5;
	
	int left =MOTOR_SPEED+((float)MOTOR_SPEED)*(((float)want_heading)/((float)MAP_WIDTH))*2.0,
		right=MOTOR_SPEED-((float)MOTOR_SPEED)*(((float)want_heading)/((float)MAP_WIDTH))*2.0;
	
	if(left>MOTOR_SPEED)left=MOTOR_SPEED;
	if(left<-MOTOR_SPEED)left=-MOTOR_SPEED;
	if(right>MOTOR_SPEED)right=MOTOR_SPEED;
	if(right<-MOTOR_SPEED)right=-MOTOR_SPEED;
	
	//Send motor packet
	robot_msgs::MotorData motordata_msg;
	motordata_msg.motor_left_velocity	=left;
	motordata_msg.motor_left_time		=MOTOR_TIMEOUT;
	motordata_msg.motor_right_velocity	=right;
	motordata_msg.motor_right_time		=MOTOR_TIMEOUT;
	motordata_pub.publish(motordata_msg);
	
	
	
	/*
	float min=9999,max=0,total=0;
	for(int i=0;i<image_x*image_y;i++)
	{
		float x=imbuf[i];
		if(x>0&&x<min)min=x;
		if(x>max)max=x;
		total+=x;
	//	printf("%f\n",buf[i]);
		//imbuf[i]=0;
		//if(x==9757)imbuf[i]=11000;
		//if(x<10000 && x>9000)imbuf[i]=11000;
		//if(x<5050 && x>4950)imbuf[i]=11000;
		//if(x<500 && x>480)imbuf[i]=5000;
		
		//if(i<20000)imbuf[i]=11000;
	}
	if(min<gmin)gmin=min;
	printf("%f<%f,%f %f\n",min,max,total/((float)image_x*image_y),gmin);
	
	*/
	
	for(int i=0;i<image_x*image_y;i++)
	{
		imbuf[i]*=5;
	}
	
	image_format=GL_LUMINANCE;
	image_type=GL_UNSIGNED_SHORT;
		
		
	glutPostRedisplay();
}

void ros_init(int &argc, char **argv,const char *topic)
{
	//ROS topic must be unique- create one per topic
	char name[MAX_LEN];
	snprintf(name,MAX_LEN,"Knav_%s",topic);
	for(int i=0;;i++)
	{
		if(name[i]=='/')
			name[i]='_';
		else if(name[i]==0)
			break;
	}
	
	//Init ROS stuff
	ros::init(argc, argv, name);//name of the program as seen by ROS
	ros::NodeHandle n;
	
	//Subscribe this program to rgb data
	rgbdata_sub = n.subscribe(topic, 1, rgbdataCallback);
	
	motordata_pub = n.advertise<robot_msgs::MotorData>("motordata", 1);
	
	ROS_INFO("Init'd Knav ROS.");
}

int main(int argc, char *argv[])
{
	
	if(argc!=2)
	{
		printf(
			"Usage:\n"
			"\trosrun knav knav [depth_topic]\n"
			"Examples:\n"
			"\trosrun knav knav /camera/depth/image_raw\n\n");
		exit(1);
	}
	
	
	const char *topic=argv[1];
	
	
	char title[MAX_LEN];
	snprintf(title,MAX_LEN,"%s - Viewer",topic);
	
	
	ros_init(argc,argv,topic);
	
	// Initialize GLUT
	glutInit(&argc,argv);

	// Initialize the window with double buffering and RGB colors
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);// | GLUT_DEPTH);

	// Set the window size to image size
	glutInitWindowSize(640,480);
	
	// Create window
	glutCreateWindow("Knav");
	

#ifndef OSX
	// Initialize GLEW - MUST BE DONE AFTER CREATING GLUT WINDOW
	//glewInit();
#endif

	//set event handlers
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutTimerFunc(1000/FPS,timer,TIMER_ID);
	
	glDisable(GL_DEPTH_TEST);
	
	// Begin event loop
	glutMainLoop();
	return 0;
}

