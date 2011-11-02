
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


#include <SOIL/SOIL.h>

#define	TIMER_ID	1
#define	FPS	60

#define MAX_LEN	512
#define CHAR_HEIGHT	20 //Doesnt seem to be a GL function to actually retrieve this
void glxPrintf(int x,int y,const char * format, ... )
{
	char buffer[MAX_LEN+1];
	va_list args;
	va_start (args, format);
	vsnprintf (buffer,MAX_LEN,format, args);
	char c;
	glRasterPos2f(x,y);
	for(int i=0;;i++)
	{
		c=buffer[i];
		if(!c)break;
		if(c=='\n')
		{
			y+=CHAR_HEIGHT;
			glRasterPos2f(x,y);
			continue;
		}
		glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24,c);//10 or 24
	}
	va_end (args);
}

Display *dpy;
Window root_window;
void init_xorg()
{
	dpy = XOpenDisplay(0);
	root_window = XRootWindow(dpy, 0);
	XSelectInput(dpy, root_window, KeyReleaseMask);
}

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


int count=0;
void display()
{
	glClear(GL_COLOR_BUFFER_BIT);
	
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
	glutPostRedisplay();
}


int main(int argc, char *argv[])
{
	
	// Initialize GLUT
	glutInit(&argc,argv);

	// Initialize the window with double buffering and RGB colors
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

	// Set the window size to image size
	glutInitWindowSize(160,80);
	
	// Create window
	glutCreateWindow("GL Test");
	
	init_xorg();

#ifndef OSX
	// Initialize GLEW - MUST BE DONE AFTER CREATING GLUT WINDOW
	glewInit();
#endif

	//set event handlers
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutTimerFunc(1000/FPS,timer,TIMER_ID);
	
	// Begin event loop
	glutMainLoop();
	return 0;
}

