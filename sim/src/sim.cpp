//GL Simulator for ROS
// Cory Boyle 2011

const char* HELP_STRING=
"--Keyboard--\n"
"W,A,S,D: Move\n"
"Q,E: Pan left/right\n"
"R,F: Tilt up/down\n"
"Numpad 0: Reset (Initialize Actors)\n"
"Numpad 8,2: Select actor up/down\n"
"Numpad 4,5,6: Select sensor left/first/right\n"
"Numpad /,*: Select mesh left/right\n"
"H: Toggle help\n"
"O: Toggle OSD\n"
"B: Toggle bounds\n"
"N: Toggle noise\n"
"M: Toggle motors\n"
"J,K,L: Toggle Sensor, Video, and Pointcloud topics\n"
"            between Enabled, Disabled, and Forced states.\n"
"`: Toggle fullscreen (FPS mode)\n"
"\n"
"--Mouse--\n"
"Scroll: Select actor up/down\n"
"Left click: Toggle fullscreen\n"
"Middle click: Select sensor next\n"
"Right click: Select mesh next\n";

//TODO:
/*
* Need to calibrate depth scale and PC constants in pointcloud.h. Should probably use to calibrate ROS sensors too?
* More to-scale room environment
* Compute and publish depthmap_float data
*/

//Comment out to use other names (e.g., so it doesn't conflict if kinect attached)
#define USE_KINECT_TOPIC_NAMES

#ifdef USE_KINECT_TOPIC_NAMES
	#define ROSTOPIC_RGB_NAME			"/camera/rgb/image_color"
	#define ROSTOPIC_DEPTH_RAW_NAME		"/camera/depth/image_raw"
	#define ROSTOPIC_DEPTH_FLOAT_NAME	"/camera/depth/image"
	#define ROSTOPIC_POINTCLOUD_NAME	"/camera/rgb/points"
	#define TF_POINTCLOUD_FRAME_ID		"/openni_rgb_optical_frame"
#else
	#define ROSTOPIC_RGB_NAME			"/sim/rgb"
	#define ROSTOPIC_DEPTH_RAW_NAME		"/sim/depth_raw"
	#define ROSTOPIC_DEPTH_FLOAT_NAME	"/sim/depth_float"
	#define ROSTOPIC_POINTCLOUD_NAME	"/sim/points"
	#define TF_POINTCLOUD_FRAME_ID		"/sim_rgb_optical_frame"
#endif

//Resolution to render sensors at. Also sets minimum window size and resolution for virtual Kinect.
#define SENSOR_WINDOW_WIDTH		640
#define SENSOR_WINDOW_HEIGHT	480

//Initial window size
#define INITIAL_WINDOW_WIDTH	640
#define INITIAL_WINDOW_HEIGHT	480

//Target FPS. MUST be a multiple of ROS_HZ greater than ROS_HZ*2.
//Comment out to run at max speed.
//This also sets the output rate for Kinect/video output
#define	FPS		30

//Number of times per second to process ROS messages
#define ROS_HZ	10


#define OSD_COLOR	0,1,1

//Max length of most strings- lines/words/filenames
#define MAX_LEN	1024
//Max textures to allocate. Increase if texture error occurs.
#define MAX_TEXTURES	20


//Disable display output
//#define NO_DISPLAY

//Disable lighting (will crash Intel GMA950 gfx if not set)
//This does NOT work in this revision
//#define NO_LIGHTING	1




//////////

//These should not be changed
#define	TIMER_ID	1

//Generally only defined on Windows.
#ifndef	VK_ESCAPE
#define VK_ESCAPE	27
#endif

//ROS *MUST* be included first for some reason
#include "ros/ros.h"

//needed for pointcloud.h
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/image_encodings.h"

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>


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

//for va_list
#include <stdarg.h>

//For chdir()
#ifdef _WIN32
	#include <direct.h>
#else//POSIX
	#include <unistd.h>
	#include <X11/Xlib.h>
	#include <X11/Xos.h>
#endif

#include <SOIL/SOIL.h>

#include "glx.h"
#include "loader3.h"


float kinect_depthmap_gl[SENSOR_WINDOW_WIDTH*SENSOR_WINDOW_HEIGHT];
float kinect_depthmap_view[SENSOR_WINDOW_WIDTH*SENSOR_WINDOW_HEIGHT];
float kinect_depthmap_float[SENSOR_WINDOW_WIDTH*SENSOR_WINDOW_HEIGHT];
uint16_t kinect_depthmap_raw[SENSOR_WINDOW_WIDTH*SENSOR_WINDOW_HEIGHT];

unsigned char kinect_rgb[SENSOR_WINDOW_WIDTH*SENSOR_WINDOW_HEIGHT*3];

//ROS stuff
#include "sensors.h"
#include "roscomm.h"

#include "move.h"
#include "display.h"



void load_res()
{
	//Find the source directory
	char exe[MAX_LEN];
	int len=readlink("/proc/self/exe",exe,MAX_LEN);
	int slashes=0;
	for(int i=len;i>0;i--)
	{
		if(exe[i]=='/')
			slashes++;
		if(slashes==2)
		{
			exe[i]=0;
			break;
		}
	}
	
	if(chdir(exe))
		printf("WARNING: Failed to locate sim root directory. Must do `roscd sim` manually before starting!\n");
	
	if(chdir("res"))
		lerr("Resource directory missing!\n");
	
	//Load mesh files
	mesh *actors=load_meshfile("actors.x");
	mesh *environment=load_meshfile("environment.x");
	//mesh *environment=load_meshfile("kinecttest.x");
	
	//main actors
	load_mesh(actors,"camera",camera);
	
	//assemble robot (multipart mesh)
	load_mesh(actors,"robot",robot);
	robot.child=&US_block;
	robot.sibling=&wheels;
	load_mesh(actors,"wheels",wheels);
	
	load_mesh(actors,"US_block",US_block);
	US_block.sibling=&IR_block;
	load_mesh(actors,"IR_block",IR_block);
	
	//obstacles
	load_mesh(actors,"null",nullmesh);
	nullmesh.list=0;
	load_mesh(actors,"box",box);
	load_mesh(actors,"fence",fence);
	load_mesh(actors,"table",table);
	load_mesh(actors,"suzanne",suzanne);
	
	
	load_mesh(environment,"",root);
}

int main(int argc, char *argv[])
{
	//Initialize RNG
	srand(time(NULL));
	
	// Initialize GLUT
	glutInit(&argc,argv);

	// Initialize the window with double buffering and RGB colors
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

	// Set the window size to image size
	glutInitWindowSize(INITIAL_WINDOW_WIDTH,INITIAL_WINDOW_HEIGHT);
	
	// Create window
	glutCreateWindow("Sim");

#ifndef OSX
	// Initialize GLEW - MUST BE DONE AFTER CREATING GLUT WINDOW
	//glewInit();
#endif

	//set event handlers
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutTimerFunc(0,timer,TIMER_ID);
	glutKeyboardFunc(keydown);
	glutKeyboardUpFunc(keyup);
	glutMouseFunc(mouseClick);
	
	//move.h
	init_keystate();
	init_xorg();
	init_actors();

	// Set shading model
	glShadeModel(GL_SMOOTH);

	// Enable depth test
	glEnable(GL_DEPTH_TEST);
	
	// Enable textures
	glEnable(GL_TEXTURE_2D);

	glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
	//glBlendFunc(GL_SRC_ALPHA,GL_ONE);
	
	glDisable(GL_BLEND);
	
#ifndef	NO_LIGHTING
	//NOTE: Causes an immediate system hang on GMA950 gfx
	glEnable(GL_LIGHTING);
	
	glRotatef(50,1,0,0);
	
	glEnable(GL_NORMALIZE);
	
	GLfloat v[4];
	v[0]=.8;
	v[1]=.8;
	v[2]=.8;
	v[3]=1;
	glLightfv(GL_LIGHT0,GL_AMBIENT,v);
	glEnable(GL_LIGHT0);
#else
	printf("WARNING: Lighting disabled!\n");
#endif

	load_res();
	
	ros_init(argc,argv);

	// Begin event loop
	glutMainLoop();
	return 0;
}

