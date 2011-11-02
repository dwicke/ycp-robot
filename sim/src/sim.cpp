// CS370 - Fall 2010
// Final Project

const char* HELP_STRING=
"W,A,S,D: Move\n"
"Q,E: Pan left/right\n"
"R,F: Tilt up/down\n"
"Numpad 0: Reset (Initialize Actors)\n"
"Numpad 8,2: Select actor up/down\n"
"Numpad 4,5,6: Select sensor left/first/right\n"
"H: Toggle help\n"
"O: Toggle OSD\n"
"B: Toggle bounds\n"
"N: Toggle noise\n"
"M: Toggle motors\n"
"`: Toggle fullscreen (FPS mode)\n";

/**
Controls:
WS AD	Move/strafe
QE RF	Rotate Z/X
TG D	Adjust convergence /reset
ZX C	Adjust IPD /reset
I		Cycle 3D modes
O		Toggle OSD
P		Toggle bounds

`		Fullscreen

1		Light
2		Fan
3		Door
4		Tilt blinds
5		Open blinds


Anaglyph Red/Cyan should work best with good quality filter glasses,
the other modes provide various tradeoffs in color accuracy, color fringing, and depth
when used with cheaper ones that significantly bleed the green channel.

2D
"Anaglyph Red/Blue",
"Anaglyph Red/Blue + Green",
"Anaglyph Red/Cyan",
*/

//todo
/*
avoid updating convergence every frame

should only read vals at CUPS rate,
but apply weight/recompute on every frame

CUPS //Nuimber of times/sec to refocus the 3D camera
*/

//ROS *MUST* be included first for some reason
#include "ros/ros.h"

#include "robot_msgs/SensorData.h"
#include "robot_msgs/MotorData.h"

#ifdef OSX
	#include <GLUT/glut.h>
#else
	#include <GL/glew.h>
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


//#define NO_DISPLAY


#include <SOIL/SOIL.h>

#define SENSOR_WINDOW_WIDTH		640
#define SENSOR_WINDOW_HEIGHT	480
#define INITIAL_WINDOW_WIDTH	SENSOR_WINDOW_WIDTH
#define INITIAL_WINDOW_HEIGHT	SENSOR_WINDOW_HEIGHT



//Max length of most strings- lines/words/filenames
#define MAX_LEN	512
#define MAX_TEXTURES	20

int winx,winy;

#include "glx.h"

#define	TIMER_ID	1
#define	FPS	60


#ifndef	VK_ESCAPE
#define VK_ESCAPE	27
#endif

//To prevent system from crashing on sucky Intel GMA950 gfx card
#ifdef __linux
//#define NO_LIGHTING	1
#endif

#ifndef	NO_LIGHTING
//#include "lighting.h"
#endif
#include "loader3.h"


//ROS stuff
#include "robot.h"
#include "sensors.h"

#include "roscomm.h"
#define SENSOR_DELAY	100;

#include "move.h"



#include "display.h"




void load_res()
{
	if(chdir("res"))
		lerr("Resource directory missing!\n(NOTE: Currently, you must be in the ros_workspace/sim/ directory for it to find this)\n");

	mesh * m=load_meshfile("room.x");
	//mesh * m=load_meshfile("robot.x");
	
	load_mesh(m,"robot",robot);
	robot.child=&US_block;
	robot.sibling=&wheels;
	load_mesh(m,"wheels",wheels);
	
	load_mesh(m,"US_block",US_block);
	US_block.sibling=&IR_block;
	load_mesh(m,"IR_block",IR_block);
	
	
	load_mesh(m,"null",nullmesh);
	nullmesh.list=0;
	load_mesh(m,"box",box);
	load_mesh(m,"fence",fence);
	load_mesh(m,"table",table);
	load_mesh(m,"suzanne",suzanne);
	
	load_mesh(m,"camera",camera);
	
	load_mesh(m,"",root);
	return;
}

int main(int argc, char *argv[])
{
	//Initialize RNG
	srand (time(NULL));
	
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
	glewInit();
#endif

	//set event handlers
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutTimerFunc(1000/FPS,timer,TIMER_ID);
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
	//set_light(GL_LIGHT1,&white_light);
	//set_spot(GL_LIGHT1,light1_pos,light1_dir,light_cutoff,light_exp);
	//glEnable(GL_LIGHT1);
	
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

