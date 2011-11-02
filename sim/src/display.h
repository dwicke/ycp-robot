

GLfloat aspect=1.;
//TODO: fix
#define world 1

//fps globals
int frame=0,timebase=0,fps=0,ltime=0;



void render_scene();
void render_hud(int ttime);
void traverse(node *root);
void display();
void reshape(int w, int h);
void raster();
void raster_flipped();
void orthographic();
void perspective(Actor actor,Sensor sensor);

//global because its used to display HUD
robot_msgs::SensorData sensordata;



void render_scene()
{//getkey(VK_UP);
	
	
	glPushMatrix();

	//	glRotatef(25,1,0,0);
		//glRotatef(theta,0,0,1);
		//glRotatef(timeinfo->tm_min/60.0*360.0,0,1,0);

		//glScalef(.75,.75,.75);
		//glScalef(.5,.5,.5);
		//glScalef(2,2,2);
		

		//glRotatef(90,0,0,1);
		//glCallList(1);
		//glTranslatef(0,0,10);
		//glCallList(2);
		traverse(&root);
	glPopMatrix();
	
	Actor actor;
	for(int i=0;i<NUM_ACTORS;i++)
	{
		if(actor_active!=i||actors[actor_active].sensors[actors[actor_active].sensor_active].type==SENSOR_TYPE_3PCAM)
		{
			//printf("Drew %d\n",i);
			glPushMatrix();
				actor=actors[i];
			
				glTranslatef(actor.x,actor.y,actor.z);
				glRotatef(-actor.ztheta,0,0,1);
				if(i==0)//only for camera
					glRotatef(-actor.xtheta,1,0,0);
				
				node *n;
				n=actor.mesh[actor.mesh_active].node;
				//printf("Rendering %d -> %d(%d,%d)\n",i,n->list,n->sibling,n->child);
				
				traverse(actor.mesh[actor.mesh_active].node);
			glPopMatrix();
		}
	}
}


float one=1.0;
void render_hud(int ttime)
{
	glBindTexture(GL_TEXTURE_2D,0);
	
	switch(osd_mode)
	{
		case 1:
			break;
		case 2:
			raster();
			glColor3f(0,1,0);
			glxPrintf(4,24,
				"%02d FPS",fps);
			break;
		case 3:
			raster();
			glColor3f(0,1,0);
			glxPrintf(4,24,
				"%02d FPS\n"
				"Bounds: %s\n"
				"%s%s: %.2fx%.2f %.1f\n"
				"%s%s: %.2fx%.2f %.1f\n"
				"Sensor: %s\n"
				"Noise: %s x%.1f\n"
				"Motors: %s\n"
				

				
				,fps,
				check_bounds?"on":"off",
				actor_active==0?"*":"",actors[0].name,actors[0].x,actors[0].y,actors[0].ztheta,
				actor_active==1?"*":"",actors[1].name,actors[1].x,actors[1].y,actors[1].ztheta,
				actors[actor_active].sensors[actors[actor_active].sensor_active].name,
			 
				noise_enable?"on":"off",noise_factor,
				enable_motors?"on":"off");
			break;
		case 4:
			raster();
			glColor3f(0,1,0);
			glxPrintf(4,24,
				"%02d FPS\n"
				"Bounds: %s\n"
				"%s: %.2fx%.2f %.1f\n"
				"Sensor: %s\n"
				"Mesh: %s\n"
				"%s: %.2fx%.2f %.1f\n"
				"Noise: %s x%.1f\n"
				"Motors: %s%s%+03.0f | %s%+03.0f\n"
				"Ultrasonic: front[%03d,%03d,%03d]|rear[%03d,%03d,%03d]\n"
				"Infrared: front[%04.1f,%04.1f,%04.1f,%04.1f]|middle[%04.1f,%04.1f]|rear[%04.1f]\n"
				
				,fps,
				check_bounds?"on":"off",
				actors[actor_active].name,actors[actor_active].x,actors[actor_active].y,actors[actor_active].ztheta,
			 	actors[actor_active].sensors[actors[actor_active].sensor_active].name,
				actors[actor_active].mesh[actors[actor_active].mesh_active].name,
			 
				actors[ACTOR_ROBOT].name,actors[ACTOR_ROBOT].x,actors[ACTOR_ROBOT].y,actors[ACTOR_ROBOT].ztheta,
			 
				noise_enable?"on":"off",noise_factor,
			 
				enable_motors?"":"OFF ",
			 	wheel_l_end>ttime?"":"X",
				wheel_l_velocity*100,
				wheel_r_end>ttime?"":"X",
				wheel_r_velocity*100,
			 
				sensordata.ultrasonic_frontLeft_distance,
				sensordata.ultrasonic_frontCenter_distance,
				sensordata.ultrasonic_frontRight_distance,
				sensordata.ultrasonic_rearLeft_distance,
				sensordata.ultrasonic_rearCenter_distance,
				sensordata.ultrasonic_rearRight_distance,

				sensordata.infrared_frontLeftLeft_distance,
				sensordata.infrared_frontLeftCenter_distance,
				sensordata.infrared_frontRightCenter_distance,
				sensordata.infrared_frontRightRight_distance,
				sensordata.infrared_right_distance,
				sensordata.infrared_rear_distance,
				sensordata.infrared_left_distance);
			break;
		default:
			raster();
			glColor3f(0,1,0);
			glxPrintf(4,24,"%s",HELP_STRING);
	}
	


		


}


float getConvergence(bool &error)
{
	int ix,iy,fx,fy,sx,sy,dx,dy;
	float depth=0,data;
	
	sx=winx*(CONVERGENCE_WINDOW_SIZE_X);
	sy=winy*(CONVERGENCE_WINDOW_SIZE_Y);

	ix=winx/2.0-(sx/2.0);
	iy=winy/2.0-(sy/2.0);
	
	dx=sx/(CONVERGENCE_POINTS-1);
	dy=sy/(CONVERGENCE_POINTS-1);
	
	fx=ix+sx;
	fy=iy+sy;
	
	if(dx==0)
	{
		printf("BUG: Window not init! (getConvergence())\n");
		error=1;
		return -1;
	}
	
	for(int y=iy;y<=fy;y+=dy)
	{
		for(int x=ix;x<=fx;x+=dx)
		{
			glReadPixels(x,y,1,1,GL_DEPTH_COMPONENT,GL_FLOAT,&data);
			depth+=data;
#ifdef CONVERGENCE_DEBUG_DRAW
			raster();
			glRasterPos2f(x,y);
			float z=0;
			glDisable(GL_DEPTH_TEST);
			glDrawPixels(1,1,GL_RED,GL_FLOAT,&z);
			glEnable(GL_DEPTH_TEST);
#endif
		}
	}
	if(depth==CONVERGENCE_POINTS*CONVERGENCE_POINTS)
	{
		printf("Bad convergence reading! [@%d]\n",ltime);
		error=1;
		return -1;
	}
	depth=-1/log((depth/CONVERGENCE_POINTS/CONVERGENCE_POINTS))/11.0*100.0;
	//printf("Convergence: %f\n",depth);
	//depth=999;
	return depth;
}




void drawConvergence()
{
	glBindTexture(GL_TEXTURE_2D,0);
	int ix,iy,fx,fy,sx,sy,dx,dy;
	
	sx=winx*(CONVERGENCE_WINDOW_SIZE_X);
	sy=winy*(CONVERGENCE_WINDOW_SIZE_Y);

	ix=winx/2.0-(sx/2.0);
	iy=winy/2.0-(sy/2.0);
	
	dx=sx/(CONVERGENCE_POINTS-1);
	dy=sy/(CONVERGENCE_POINTS-1);
	
	fx=ix+sx;
	fy=iy+sy;
	
	if(dx==0)
	{
		printf("BUG: Window not init! (drawConvergence())\n");
		return;
	}
	
	for(int y=iy;y<=fy;y+=dy)
	{
		for(int x=ix;x<=fx;x+=dx)
		{
			raster();
			glRasterPos2f(x,y);
			float z=0xFF;
			glDisable(GL_DEPTH_TEST);
			glDrawPixels(1,1,GL_GREEN,GL_FLOAT,&z);
			glEnable(GL_DEPTH_TEST);
		}
	}
}

float *dmbuf=0;//allocated in reshape()
void clone_depth()
{
	if(!dmbuf)
	{
		printf("depthbuffer not init!\n");
		return;
	}
	glBindTexture(GL_TEXTURE_2D,0);
	//winx=100;winy=100;
	

//	glFlush();
//	glutSwapBuffers();

	
	glReadPixels(0,0,winx,winy,GL_DEPTH_COMPONENT,GL_FLOAT,dmbuf);
	
	int dim=winx*winy;
	
	/*
	for(int i=0;i<dim;i++)
	{
		dmbuf[i]=1.0- (-1.0/log((dmbuf[i]))/11.0/20.0);
	//	data[i]=i/(float)dim*2;
	}*/
	
	for(int i=0;i<dim;i+=4)
	{//Only computing every 4th cuz the whole thing is too slow
		dmbuf[i]=
		dmbuf[i+1]=
		dmbuf[i+2]=
		dmbuf[i+3]=1.0- (-1.0/log((dmbuf[i]))/11.0/20.0);
		
	//	data[i]=i/(float)dim*2;
	}


	
	raster_flipped();
	glRasterPos2f(0,0);
	glDisable(GL_DEPTH_TEST);
	glDrawPixels(winx,winy,GL_LUMINANCE,GL_FLOAT,dmbuf);
	glEnable(GL_DEPTH_TEST);
	
	//printf("%f\n",data);
}

// Display routine
int sensor_elapsed=0;
void display()
{
	int ttime=glutGet(GLUT_ELAPSED_TIME),
	eltime=ttime-timebase;
	frame++;
	if (eltime > 1000) {
		fps = frame*1000.0/eltime;
		timebase = ttime;
		frame = 0;
	}
	
	//move(ttime,(ttime-ltime)/1000.0);
	move(ttime,ltime);
	ltime=ttime;
	
	
	
/*
	switch(stereo_mode)
	{
	case ANA_RGB:// red/blue with green center
		//Render center
		glClear(GL_DEPTH_BUFFER_BIT);
		glColorMask(GL_FALSE,GL_TRUE,GL_FALSE,GL_FALSE);
		perspective(0,stereo_convergence);
		render_scene();
		
		glClear(GL_DEPTH_BUFFER_BIT);
		//fallthru
	case ANA_RB:// red/blue
		//Render left
		//depth clear already done
		glColorMask(GL_TRUE,GL_FALSE,GL_FALSE,GL_FALSE);
		perspective(-stereo_ipd,stereo_convergence);
		render_scene();
		
		//Render right
		glClear(GL_DEPTH_BUFFER_BIT);
		glColorMask(GL_FALSE,GL_FALSE,GL_TRUE,GL_FALSE);
		perspective(stereo_ipd,stereo_convergence);
		render_scene();
		
		//restore mask
		glColorMask(GL_TRUE,GL_TRUE,GL_TRUE,GL_TRUE);
		break;
	case ANA_RC:// red/cyan
		//Render left
		glClear(GL_DEPTH_BUFFER_BIT);
		glColorMask(GL_TRUE,GL_FALSE,GL_FALSE,GL_FALSE);
		perspective(-stereo_ipd,stereo_convergence);
		render_scene();
		
		//Render right
		glClear(GL_DEPTH_BUFFER_BIT);
		glColorMask(GL_FALSE,GL_TRUE,GL_TRUE,GL_FALSE);
		perspective(stereo_ipd,stereo_convergence);
		render_scene();
		
		//restore mask
		glColorMask(GL_TRUE,GL_TRUE,GL_TRUE,GL_TRUE);
		break;
	
	case NORMAL://2d
*/
	//glClear(GL_DEPTH_BUFFER_BIT);
	
	//glDisable(GL_DEPTH_TEST);
	
	
	
	//ROS stuff
	sensor_elapsed+=eltime;
	if(sensor_elapsed>=100)//10hz
	{
		sensor_elapsed=0;
		//send_sensor_dummy();
		Actor actor=actors[ACTOR_ROBOT];
		Sensor *sensors=actor.sensors;
		
		glColorMask(GL_FALSE,GL_FALSE,GL_FALSE,GL_FALSE);
		glDisable(GL_LIGHTING);
		glDisable(GL_TEXTURE_2D);
		glDisable(GL_NORMALIZE);
		
		glClear(GL_DEPTH_BUFFER_BIT);
		
		//Use consistent size
		int owinx=winx,owiny=winy;
		float oaspect=aspect;
		
		glViewport(0,0,SENSOR_WINDOW_WIDTH,SENSOR_WINDOW_HEIGHT);
		aspect=((GLfloat)SENSOR_WINDOW_WIDTH)/((GLfloat)SENSOR_WINDOW_HEIGHT);
		winx=SENSOR_WINDOW_WIDTH;
		winy=SENSOR_WINDOW_HEIGHT;
		
		bool data_bad=0;
		
		perspective(actor,sensors[sensor_ultrasonic_frontLeft]);
		render_scene();
		sensordata.ultrasonic_frontLeft_distance=us_sim(getConvergence(data_bad));

		perspective(actor,sensors[sensor_ultrasonic_frontCenter]);
		render_scene();
		sensordata.ultrasonic_frontCenter_distance=us_sim(getConvergence(data_bad));
		
		perspective(actor,sensors[sensor_ultrasonic_frontRight]);
		render_scene();
		sensordata.ultrasonic_frontRight_distance=us_sim(getConvergence(data_bad));
		
		perspective(actor,sensors[sensor_ultrasonic_rearRight]);
		render_scene();
		sensordata.ultrasonic_rearRight_distance=us_sim(getConvergence(data_bad));
		
		perspective(actor,sensors[sensor_ultrasonic_rearCenter]);
		render_scene();
		sensordata.ultrasonic_rearCenter_distance=us_sim(getConvergence(data_bad));
		
		perspective(actor,sensors[sensor_ultrasonic_rearLeft]);
		render_scene();
		sensordata.ultrasonic_rearLeft_distance=us_sim(getConvergence(data_bad));

		
		//IR//////////////
		perspective(actor,sensors[sensor_infrared_frontLeftLeft]);
		render_scene();
		sensordata.infrared_frontLeftLeft_distance=ir_sim(getConvergence(data_bad));
		
		perspective(actor,sensors[sensor_infrared_frontLeftCenter]);
		render_scene();
		sensordata.infrared_frontLeftCenter_distance=ir_sim(getConvergence(data_bad));
		
		perspective(actor,sensors[sensor_infrared_frontRightCenter]);
		render_scene();
		sensordata.infrared_frontRightCenter_distance=ir_sim(getConvergence(data_bad));
		
		perspective(actor,sensors[sensor_infrared_frontRightRight]);
		render_scene();
		sensordata.infrared_frontRightRight_distance=ir_sim(getConvergence(data_bad));
		
		perspective(actor,sensors[sensor_infrared_right]);
		render_scene();
		sensordata.infrared_right_distance=ir_sim(getConvergence(data_bad));
		
		perspective(actor,sensors[sensor_infrared_rear]);
		render_scene();
		sensordata.infrared_rear_distance=ir_sim(getConvergence(data_bad));
		
		perspective(actor,sensors[sensor_infrared_left]);
		render_scene();
		sensordata.infrared_left_distance=ir_sim(getConvergence(data_bad));

		sensordata.human_left_motion=HUMAN_NULL;
		sensordata.human_left_presence=HUMAN_NULL;
		sensordata.human_right_motion=HUMAN_NULL;
		sensordata.human_right_presence=HUMAN_NULL;
		
		
		if(!data_bad)
			send_sensordata(sensordata);
		
		//restore viewport
		winx=owinx;
		winy=owiny;
		aspect=oaspect;
		glViewport(0,0,winx,winy);
		
		//restore mask
		glColorMask(GL_TRUE,GL_TRUE,GL_TRUE,GL_TRUE);
		glEnable(GL_LIGHTING);
		glEnable(GL_TEXTURE_2D);
		glEnable(GL_NORMALIZE);
	
	}
	
	
	
	
	Actor actor=actors[actor_active];
	Sensor sensor=actor.sensors[actor.sensor_active];
	
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	perspective(actor,sensor);
	render_scene();
	
	
	if(sensor.type==SENSOR_TYPE_INFRARED||sensor.type==SENSOR_TYPE_ULTRASONIC)
		drawConvergence();
	else if(sensor.type==SENSOR_TYPE_KINECT)
		clone_depth();
	
	///////////
		
	
#ifdef NO_DISPLAY
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
#endif
	
	//orthographic();
	// Render h
#ifndef	NO_LIGHTING
	glDisable(GL_LIGHTING);
#endif
	glDisable(GL_DEPTH_TEST);
	render_hud(ttime);
	glEnable(GL_DEPTH_TEST);
#ifndef	NO_LIGHTING
	glEnable(GL_LIGHTING);
#endif

	// Flush buffer
	glFlush();
	
	
/*
GLenum errCode;
const GLubyte *errString;

if ((errCode = glGetError()) != GL_NO_ERROR) {
    errString = gluErrorString(errCode);
   fprintf (stderr, "OpenGL Error: %s\n", errString);
   exit(1);
}
*/
	
	// Swap buffers
	glutSwapBuffers();
	//glutPostRedisplay();
}

void timer(int id)
{
	glutTimerFunc(1000/FPS,timer,TIMER_ID);
	
	//NOTE: The redisplay handler is called directly here. This is so the simulation won't pause when the window is minimized.
	//glutPostRedisplay();
	display();
}

// Reshape callback
void reshape(int w, int h)
{
	if(w<SENSOR_WINDOW_WIDTH || h<SENSOR_WINDOW_HEIGHT)
	{
		glutReshapeWindow(SENSOR_WINDOW_WIDTH,SENSOR_WINDOW_HEIGHT);
		aspect=winx=winy=0;
		return;
	}
	
	// Set new screen extents
	glViewport(0,0,w,h);

	aspect=((GLfloat)w)/((GLfloat)h);
	winx=w;
	winy=h;
	
	//(Re)allocate depthmap copy buffer
	if(dmbuf)
		free(dmbuf);
	int dim=winx*winy;
	dmbuf=(float*)malloc(dim*sizeof(float));
	//float *data=(float*)malloc(1024*1024);
	if(dmbuf==NULL)lerr("Out of memory allocating depthmap buffer- bogus window dimensions?");
	
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

//orthographic, pixel coordinates
void raster_flipped()
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0,winx,0,winy,-1,1);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void perspective(Actor actor,Sensor sensor)
{
	// Select projection matrix
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	
	//glFrustum(-1,1,-1,1,1,8);
	//gluPerspective(90,1,1,4);
	gluPerspective(45,aspect,.1,100);//fov,aspect,nearclip,far
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	
	gluLookAt(
			0,0,0,	//camera location
			0,1,0,	//target location
			0,0,1);	//up vector
	//glLoadIdentity();
	
	//Apply camera tilt (last)
	if(sensor.type==SENSOR_TYPE_CAMERA)//only for camera
		glRotatef(actor.xtheta,1,0,0);
	
	//Apply sensor transforms (2nd)
	glRotatef(sensor.xtheta,1,0,0);
	glRotatef(sensor.ztheta,0,0,1);
	glTranslatef(-sensor.x/100.0,-sensor.y/100.0,-sensor.z/100.0);//note: conversion cm->m
	
	//Apply actor origin transforms (1st)
	//glRotatef(actor.xtheta,1,0,0);//done above, in weird order
	glRotatef(actor.ztheta,0,0,1);
	glTranslatef(-actor.x,-actor.y,-actor.z);
	
}


/*
	float transform[16];		//user transform
	float static_transform[16];	//frame transform
	int list;					//display list
	node *sibling,*child;
*/
// Tree traversal routine
void traverse(node *root)
{
	// Bottom of branch
	if (root == NULL)
	{
		return;
	}

	// Update transformation, set material, and draw object
	glPushMatrix();
	glMultMatrixf(root->static_transform);
	glMultMatrixf(root->transform);
	//root->f();
	
	if(root->transparent)
	{
		//printf("MIRROR!\n");
		glDepthMask(GL_FALSE);
		glEnable(GL_BLEND);
		glCallList(root->list);
		glDisable(GL_BLEND);
		glDepthMask(GL_TRUE);
	}
	else
	{
		glCallList(root->list);
	}
	
	// Recurse tree vertically
	if (root->child != NULL)
	{
		traverse(root->child);
	}
	glPopMatrix();

	// Recurse tree laterally
	if (root->sibling != NULL)
	{
		traverse(root->sibling);
	}
}
