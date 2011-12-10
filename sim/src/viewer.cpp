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



//#include "roscomm.h"
#include "ros/ros.h"

//#include "robot_msgs/MotorData.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/image_encodings.h"



#define	TIMER_ID	1
#define	FPS	60

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


void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& im)
{
	//std::basic_string<char> a=image->encoding;
	//const char *s=a.c_str();
	/*
	ROS_INFO("Got image packet: %s %dx%d",
		im->encoding.c_str(),
		im->width,
		im->height);
	*/
	
	//const unsigned char *a=im->data.data();
	
	std::vector<uint8_t> data=im->data;
	size_t size=data.size();
	
	image_x=im->width;
	image_y=im->height;
	
	if(lsize==-1)
	{
		printf("Format: pointcloud %dx%d (%d bpp + %d, %s-endian)\n",image_x,image_y,(int)size/(image_x*image_y),(int)size%(image_x*image_y),
			im->is_bigendian==1?"big":"little"
		);
	}
	
	
	const unsigned char *buf=data.data();
	
			//try anyway, with larger buffer
	sizebuf(size+image_x*image_y*16);
	
//	for(int i=0;i<size;i++)
//		image[i]=buf[size-i-1];
	
	const float *depthmap=(const float*)buf;
	float min=0,max=0;
	for(int i=0;i<image_x*image_y;i++)
	{
		float x=buf[i];
		if(x<min)min=x;
		if(x>max)max=x;
	//	printf("%f\n",buf[i]);
	}
	printf("%f<%f\n",min,max);
	
	
	float *out=(float*)image;
	int j=0;
	for(int i=0;i<size/4;i+=4)
	{
		//printf("%d<-%d\n",j,i);
		out[j]=depthmap[i+0]/10.0;
		//out[j]=1;
		
		j++;
	}
	
	//memcpy(image,buf,size);
	
	image_format=GL_LUMINANCE;
	image_type=GL_FLOAT;
	glutPostRedisplay();
}

float gmin=9999;
void rgbdataCallback(const sensor_msgs::Image::ConstPtr& im)
{
	//std::basic_string<char> a=image->encoding;
	//const char *s=a.c_str();
	/*
	ROS_INFO("Got image packet: %s %dx%d",
		im->encoding.c_str(),
		im->width,
		im->height);
	*/
	
	//const unsigned char *a=im->data.data();
	
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
	
	if(im->encoding==sensor_msgs::image_encodings::RGB8)
	{//RGB image
		if(size!=image_x*image_y*3)
		{
			printf("Invalid buffer size: %d\n",size);
			exit(1);
		}
		
		sizebuf(size);
		memcpy(image,buf,size);
		
		image_format=GL_RGB;
		image_type=GL_UNSIGNED_BYTE;
	}
	else if(im->encoding==sensor_msgs::image_encodings::MONO8)
	{//mono image
		if((int)size!=image_x*image_y)
		{
			printf("Invalid buffer size: %d\n",size);
			exit(1);
		}
		
		sizebuf(size);
		memcpy(image,buf,size);
		
		image_format=GL_LUMINANCE;
		image_type=GL_UNSIGNED_BYTE;
	}
	else if(im->encoding==sensor_msgs::image_encodings::TYPE_32FC1)//||im->encoding==sensor_msgs::image_encodings::TYPE_16SC1)
	{//depthmap
		
		if((int)size!=image_x*image_y*sizeof(float))
		{
			printf("Invalid buffer size: %d\n",size);
		//	exit(1);
		}
				//try anyway, with larger buffer
		sizebuf(size+image_x*image_y*8);
		
	//	for(int i=0;i<size;i++)
	//		image[i]=buf[size-i-1];
		
		const float *depthmap=(const float*)&buf;
		float min=9999,max=0,total=0;
		for(int i=0;i<image_x*image_y;i++)
		{
			float x=buf[i];
			if(x<min)min=x;
			if(x>max)max=x;
			total+=x;
		//	printf("%f\n",buf[i]);
		}
		printf("%f<%f,%f\n",min,max,total/((float)image_x*image_y));
		
		memcpy(image,buf,size);
		
		float *imbuf=(float*)image;
		
		for(int i=0;i<image_x*image_y;i++)
		{
			imbuf[i]/=10.0;
		}
		
		
		image_format=GL_LUMINANCE;
		image_type=GL_FLOAT;
	}
	else if(im->encoding==sensor_msgs::image_encodings::TYPE_16SC1||im->encoding==sensor_msgs::image_encodings::TYPE_16UC1)//This is treated as unsigned regardless, but that shouldn't matter (values are expected to be non-negitive and "small")
	{//raw depthmap
		if((int)size!=image_x*image_y*2)
		{
			printf("Invalid buffer size: %d\n",size);
			exit(1);
		}
		
		sizebuf(size);
		memcpy(image,buf,size);
		
		uint16_t *imbuf=(uint16_t*)image;
		
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
		
		for(int i=0;i<image_x*image_y;i++)
		{
			imbuf[i]*=5;
		}
		
		image_format=GL_LUMINANCE;
		image_type=GL_UNSIGNED_SHORT;
	}
	else
	{
		printf("Unknown format: %s\n",im->encoding.c_str());
		
		//try anyway, with larger buffer
		if(image)
			free(image);
		image=(unsigned char*)malloc(size+image_x*image_y);
		
		memcpy(image,buf,size);
		
		image_format=GL_LUMINANCE;
		image_type=GL_UNSIGNED_BYTE;
		//exit(1);
	}
	glutPostRedisplay();
	
	/*
	int IMAGE_WIDTH =640,
		IMAGE_HEIGHT=480;
		
	image_x=IMAGE_WIDTH;
	image_y=IMAGE_HEIGHT;
	
	if(image)
		free(image);
	
	image=(unsigned char*)malloc(IMAGE_WIDTH*IMAGE_HEIGHT*3);
	memset(image,0,IMAGE_WIDTH*IMAGE_HEIGHT*3);
	
	//int ttime=glutGet(GLUT_ELAPSED_TIME);
	for(int y=0;y<IMAGE_HEIGHT;y++)
	{
		for(int x=0;x<IMAGE_WIDTH;x++)
		{
			image[(y*IMAGE_WIDTH+x)*3]=(y/(float)IMAGE_HEIGHT)*255;
		}
	}
	*/
}

void ros_init(int &argc, char **argv,const char *topic)
{
	//ROS topic must be unique- create one per topic
	char name[MAX_LEN];
	snprintf(name,MAX_LEN,"Viewer_%s",topic);
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
	//rgbdata_sub = n.subscribe("camera/rgb/image_color", 1, rgbdataCallback);
	rgbdata_sub = n.subscribe(topic, 1, rgbdataCallback);
	//pointcloud_sub = n.subscribe(topic, 1, pointcloudCallback);
	
	ROS_INFO("Init'd viewer ROS.");
}

int main(int argc, char *argv[])
{
	
	if(argc!=2)
	{
		printf(
			"Usage:\n"
			"\trosrun sim viewer [topic]\n"
			"Examples:\n"
			"\trosrun sim viewer /camera/rgb/image_color\n"
			"\trosrun sim viewer /camera/depth/image\n\n");
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
	glutCreateWindow(title);
	

#ifndef OSX
	// Initialize GLEW - MUST BE DONE AFTER CREATING GLUT WINDOW
//	glewInit();
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

