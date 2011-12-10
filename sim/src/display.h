//GL display logic
// Cory Boyle 2011

//fps globals
int ltime=0,
framerate_counter=0,framerate_timebase=0,fps=0,
roshz_counter=0,roshz_timebase=0,roshz=0;



void render_scene();
void render_hud(int ttime);
void traverse(node *root);
void display();
void reshape(int w, int h);
void raster();
void raster_flipped();
void orthographic();
void perspective(Actor actor,Sensor sensor);
void perspective_flipped(Actor actor,Sensor sensor);

//global because its used to display HUD
robot_msgs::SensorData sensordata;



void render_scene()
{
	glPushMatrix();
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
				
				traverse(actor.mesh[actor.mesh_active].node);
			glPopMatrix();
		}
	}
}

void render_hud(int ttime)
{
	glBindTexture(GL_TEXTURE_2D,0);
	
	switch(osd_mode)
	{
		case 1:
			break;
		case 2:
			raster();
			glColor3f(OSD_COLOR);
			glxPrintf(4,24,
				"%02d FPS / %02d hz",fps,roshz);
			break;
		case 3:
			raster();
			glColor3f(OSD_COLOR);
			glxPrintf(4,24,
				"%02d FPS / %02d hz\n"
				"Bounds: %s\n"
				"%s%s: %.2fx%.2f %.1f\n"
				"%s%s: %.2fx%.2f %.1f\n"
				"Sensor: %s\n"
				"Noise: %s x%.1f\n"
				"Motors: %s\n"
				
				,fps,roshz,
				check_bounds?"on":"off",
				actor_active==0?"*":"",actors[0].name,actors[0].x,actors[0].y,actors[0].ztheta,
				actor_active==1?"*":"",actors[1].name,actors[1].x,actors[1].y,actors[1].ztheta,
				actors[actor_active].sensors[actors[actor_active].sensor_active].name,
			 
				noise_enable?"on":"off",noise_factor,
				enable_motors?"on":"off");
			break;
		case 4:
			const char *sensors,*points,*video;
			switch(rostopic_sensors_mode)
			{
				case ENABLE:
					sensors="E";
					break;
				case DISABLE:
					sensors="D";
					break;
				case FORCE:
					sensors="F";
			}
			switch(rostopic_pointcloud_mode)
			{
				case ENABLE:
					points="E";
					break;
				case DISABLE:
					points="D";
					break;
				case FORCE:
					points="F";
			}
			switch(rostopic_video_mode)
			{
				case ENABLE:
					video="E";
					break;
				case DISABLE:
					video="D";
					break;
				case FORCE:
					video="F";
			}
			
			raster();
			glColor3f(OSD_COLOR);
			glxPrintf(4,24,
				"%02d FPS / %02d hz\n"
				"Bounds: %s\n"
				"%s: %.2fx%.2f %.1f\n"
				"Sensor: %s\n"
				"Mesh: %s\n"
				"%s: %.2fx%.2f %.1f\n"
				"Noise: %s x%.1f\n"
				"Motors: %s%s%+03.0f | %s%+03.0f\n"
				"ROS: Sensors [%s (%d)] | Video [%s (%d,%d,%d)] | Points [%s (%d)]%c\n"	//NOTE: This uses a null charactor to conditionally terminate at this point. Bad Things will happen if the following lines are reordered!
				"Ultrasonic: front[%03d,%03d,%03d]|rear[%03d,%03d,%03d]\n"
				"Infrared: front[%04.1f,%04.1f,%04.1f,%04.1f]|middle[%04.1f,%04.1f]|rear[%04.1f]\n"
				
				,fps,roshz,
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
			 
				sensors,rostopic_sensors_subscribed,
				video,rostopic_rgb_subscribed,rostopic_depth_raw_subscribed,rostopic_depth_float_subscribed,
				points,rostopic_pointcloud_subscribed,
				
				(rostopic_sensors_mode==FORCE || (rostopic_sensors_mode==ENABLE&&rostopic_sensors_subscribed))?' ':0,
				
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
			glColor3f(OSD_COLOR);
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
			glDrawPixels(1,1,GL_BLUE,GL_FLOAT,&z);
			glEnable(GL_DEPTH_TEST);
		}
	}
}


//Read depthmap
void read_depthmap_gl()
{
	glReadPixels(0,0,SENSOR_WINDOW_WIDTH,SENSOR_WINDOW_HEIGHT,GL_DEPTH_COMPONENT,GL_FLOAT,kinect_depthmap_gl);
}

void read_rgb()
{
	glReadPixels(0,0,SENSOR_WINDOW_WIDTH,SENSOR_WINDOW_HEIGHT,GL_RGB,GL_UNSIGNED_BYTE,kinect_rgb);
}


//Render depthmap
void draw_depthmap()
{
	glBindTexture(GL_TEXTURE_2D,0);
	
	//Set up the viewport
	glPixelZoom((float)winx/(float)SENSOR_WINDOW_WIDTH,-(float)winy/(float)SENSOR_WINDOW_HEIGHT);
	raster_flipped();
	glRasterPos2f(0,winy);
	
	glDisable(GL_DEPTH_TEST);
	glDrawPixels(SENSOR_WINDOW_WIDTH,SENSOR_WINDOW_HEIGHT,GL_LUMINANCE,GL_FLOAT,kinect_depthmap_view);
	glEnable(GL_DEPTH_TEST);
}

void draw_rgb()
{
	glBindTexture(GL_TEXTURE_2D,0);
	
	//Set up the viewport
	glPixelZoom((float)winx/(float)SENSOR_WINDOW_WIDTH,-(float)winy/(float)SENSOR_WINDOW_HEIGHT);
	raster_flipped();
	glRasterPos2f(0,winy);
	
	glDisable(GL_DEPTH_TEST);
	glDrawPixels(SENSOR_WINDOW_WIDTH,SENSOR_WINDOW_HEIGHT,GL_RGB,GL_UNSIGNED_BYTE,kinect_rgb);
	glEnable(GL_DEPTH_TEST);
}


//Convert depthmap for view
void convert_depthmap(float *depth_data,float *depth_out_view_buffer,uint16_t *depth_out_raw_buffer,float *depth_out_float_buffer)
{
	for(int i=0;i<SENSOR_WINDOW_WIDTH*SENSOR_WINDOW_HEIGHT;i++)
	{
		float depth=(-1.0/log(depth_data[i]))*100;
		
		//Clipping
		if(depth>9757)
			depth=9757;
		if(depth<480)
			depth=0;
		
		//Transform depth buffers
		if(depth_out_view_buffer)
		{
			depth_out_view_buffer[i]=depth/9757.0;
		}
		if(depth_out_raw_buffer)
		{
			depth_out_raw_buffer[i]=depth;
		}
		if(depth_out_float_buffer)
		{
			//TODO: Not Implemented!
		}
	}
}

//Routines for switching viewport between fixed size and restoring
bool viewport_fixed=false;
int unfixed_winx,unfixed_winy;
float unfixed_aspect;
void fix_viewport()
{
	//Don't do it twice
	if(viewport_fixed)
		return;
	viewport_fixed=true;
	
	//store the old values
	unfixed_winx=winx;
	unfixed_winy=winy;
	unfixed_aspect=aspect;
	
	//Fix the viewport size
	glViewport(0,0,SENSOR_WINDOW_WIDTH,SENSOR_WINDOW_HEIGHT);
	aspect=((GLfloat)SENSOR_WINDOW_WIDTH)/((GLfloat)SENSOR_WINDOW_HEIGHT);
	winx=SENSOR_WINDOW_WIDTH;
	winy=SENSOR_WINDOW_HEIGHT;
}

void restore_viewport()
{
	//Only if fixed
	if(!viewport_fixed)
		return;
	viewport_fixed=false;
	
	winx=unfixed_winx;
	winy=unfixed_winy;
	aspect=unfixed_aspect;
	glViewport(0,0,winx,winy);
}

// Display routine
int sensor_elapsed=0;
void display()
{
	int ttime=glutGet(GLUT_ELAPSED_TIME);
	
	
	int framerate_eltime=ttime-framerate_timebase;
	framerate_counter++;
	if (framerate_eltime > 1000) {
		fps = framerate_counter*1000.0/framerate_eltime;
		framerate_timebase = ttime;
		framerate_counter = 0;
	}
	
	move(ttime,ltime);
	sensor_elapsed+=ttime-ltime;
	ltime=ttime;
	
	//ROS stuff
	if(	sensor_elapsed>=(1000/ROS_HZ)//Send data at ROS_HZ rate- speed up slightly if framerate is low to compensate for 
		-(1000/(fps+.0001)/2))		 //Speed up slightly if framerate is low to compensate for the delay (this makes it send slightly faster than normal, rather than significantly slower than normal)
	{
		//Compute FPS statistics and warn on computer-too-slow
		if(sensor_elapsed>=(1000/ROS_HZ)*1.5 && roshz_timebase)
		{
			printf("WARNING: ROS sensor pass was missed- computer is too slow, simulation won't be accurate! [%d]\n",sensor_elapsed);
		}
		int roshz_eltime=ttime-roshz_timebase;
		roshz_counter++;
		if (roshz_eltime > 1000) {
			roshz = roshz_counter*1000.0/roshz_eltime + 0.4999;//force it to round up
			roshz_timebase = ttime;
			roshz_counter = 0;
		}
		sensor_elapsed=0;
		
		//Run sensor render passes
		if( ( (rostopic_sensors_mode==FORCE || (rostopic_sensors_mode==ENABLE&&rostopic_sensors_subscribed)) )//Only if ROS enabled and subscribed

			&& fps)		//Don't start transmitting ROS data until framerate is stabilized
		{
			
			Actor actor=actors[ACTOR_ROBOT];
			Sensor *sensors=actor.sensors;
			
			//Disable features that aren't needed (should speed up rendering)
			glColorMask(GL_FALSE,GL_FALSE,GL_FALSE,GL_FALSE);
			glDisable(GL_LIGHTING);
			glDisable(GL_TEXTURE_2D);
			glDisable(GL_NORMALIZE);
			
			//Use consistent size
			fix_viewport();
			
			bool data_bad=0;
			
			glClear(GL_DEPTH_BUFFER_BIT);
			perspective(actor,sensors[sensor_ultrasonic_frontLeft]);
			render_scene();
			sensordata.ultrasonic_frontLeft_distance=us_sim(getConvergence(data_bad));
			
			glClear(GL_DEPTH_BUFFER_BIT);
			perspective(actor,sensors[sensor_ultrasonic_frontCenter]);
			render_scene();
			sensordata.ultrasonic_frontCenter_distance=us_sim(getConvergence(data_bad));
			
			glClear(GL_DEPTH_BUFFER_BIT);
			perspective(actor,sensors[sensor_ultrasonic_frontRight]);
			render_scene();
			sensordata.ultrasonic_frontRight_distance=us_sim(getConvergence(data_bad));
			
			glClear(GL_DEPTH_BUFFER_BIT);
			perspective(actor,sensors[sensor_ultrasonic_rearRight]);
			render_scene();
			sensordata.ultrasonic_rearRight_distance=us_sim(getConvergence(data_bad));
			
			glClear(GL_DEPTH_BUFFER_BIT);
			perspective(actor,sensors[sensor_ultrasonic_rearCenter]);
			render_scene();
			sensordata.ultrasonic_rearCenter_distance=us_sim(getConvergence(data_bad));
			
			glClear(GL_DEPTH_BUFFER_BIT);
			perspective(actor,sensors[sensor_ultrasonic_rearLeft]);
			render_scene();
			sensordata.ultrasonic_rearLeft_distance=us_sim(getConvergence(data_bad));

			
			//IR//////////////
			glClear(GL_DEPTH_BUFFER_BIT);
			perspective(actor,sensors[sensor_infrared_frontLeftLeft]);
			render_scene();
			sensordata.infrared_frontLeftLeft_distance=ir_sim(getConvergence(data_bad));
			
			glClear(GL_DEPTH_BUFFER_BIT);
			perspective(actor,sensors[sensor_infrared_frontLeftCenter]);
			render_scene();
			sensordata.infrared_frontLeftCenter_distance=ir_sim(getConvergence(data_bad));
			
			glClear(GL_DEPTH_BUFFER_BIT);
			perspective(actor,sensors[sensor_infrared_frontRightCenter]);
			render_scene();
			sensordata.infrared_frontRightCenter_distance=ir_sim(getConvergence(data_bad));
			
			glClear(GL_DEPTH_BUFFER_BIT);
			perspective(actor,sensors[sensor_infrared_frontRightRight]);
			render_scene();
			sensordata.infrared_frontRightRight_distance=ir_sim(getConvergence(data_bad));
			
			glClear(GL_DEPTH_BUFFER_BIT);
			perspective(actor,sensors[sensor_infrared_right]);
			render_scene();
			sensordata.infrared_right_distance=ir_sim(getConvergence(data_bad));
			
			glClear(GL_DEPTH_BUFFER_BIT);
			perspective(actor,sensors[sensor_infrared_rear]);
			render_scene();
			sensordata.infrared_rear_distance=ir_sim(getConvergence(data_bad));
			
			glClear(GL_DEPTH_BUFFER_BIT);
			perspective(actor,sensors[sensor_infrared_left]);
			render_scene();
			sensordata.infrared_left_distance=ir_sim(getConvergence(data_bad));

			sensordata.human_left_motion=HUMAN_NULL;
			sensordata.human_left_presence=HUMAN_NULL;
			sensordata.human_right_motion=HUMAN_NULL;
			sensordata.human_right_presence=HUMAN_NULL;
			
			
			if(!data_bad)
				ros_send_sensordata(sensordata);
			
			//restore mask
			glColorMask(GL_TRUE,GL_TRUE,GL_TRUE,GL_TRUE);
			glEnable(GL_LIGHTING);
			glEnable(GL_TEXTURE_2D);
			glEnable(GL_NORMALIZE);
		}
		
		//Process ROS callbacks
		ros_do_callbacks();
	}
	
	Actor actor=actors[actor_active];
	Sensor sensor=actor.sensors[actor.sensor_active];
	
	//Do Kinect/video render passes
	{
		ros_get_subscribers();
		
		//Figure out what to do
		bool need_render				=false,
			 
			 need_rgb					=false,
			 need_depth_gl				=false,
			 need_depth_view			=false,
			 need_depth_raw				=false,
			 need_depth_float			=false,
			 need_pointcloud			=false,
			 
			 need_publish_rgb			=false,
			 need_publish_depth_raw		=false,
			 need_publish_depth_float	=false,
			 need_publish_pointcloud	=false;
		
		need_publish_rgb		=rostopic_video_mode		==FORCE || (rostopic_video_mode		==ENABLE&&rostopic_rgb_subscribed);
		need_publish_depth_raw	=rostopic_video_mode		==FORCE || (rostopic_video_mode		==ENABLE&&rostopic_depth_raw_subscribed);
		need_publish_depth_float=rostopic_video_mode		==FORCE || (rostopic_video_mode		==ENABLE&&rostopic_depth_float_subscribed);
		need_publish_pointcloud	=rostopic_pointcloud_mode	==FORCE || (rostopic_pointcloud_mode==ENABLE&&rostopic_pointcloud_subscribed);
		
		if(need_publish_pointcloud)
		{
			need_render		=true;
			need_pointcloud	=true;
			need_depth_gl	=true;
			need_rgb		=true;
		}
		else if(sensor.type==SENSOR_TYPE_KINECT_RGB || need_publish_rgb)
		{
			need_render		=true;
			need_rgb		=true;
		}
		
		if(sensor.type==SENSOR_TYPE_KINECT_DEPTHMAP)
		{
			need_render		=true;
			need_depth_gl	=true;
			need_depth_view	=true;
		}
		
		if(need_publish_depth_raw)
		{
			need_render		=true;
			need_depth_gl	=true;
			need_depth_raw	=true;
		}
		if(need_publish_depth_float)
		{
			need_render		=true;
			need_depth_gl	=true;
			need_depth_float=true;
		}
		
		
		//Do what needs done
		if(need_render)
		{
			//Use consistent size
			fix_viewport();
			
			//Do render pass
			glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
			Actor actor=actors[ACTOR_ROBOT];
			Sensor *sensors=actor.sensors;
			perspective_flipped(actor,sensors[sensor_kinect_rgb]);
			render_scene();
			
			//Read raw images
			if(need_depth_gl)
			{
				read_depthmap_gl();
			}
			if(need_rgb)
			{
				read_rgb();
			}
			
			
			//Figure out what depthmap modes need computed from the GL depthmap
			float		*depth_view	=0;
			uint16_t	*depth_raw	=0;
			float		*depth_float=0;
			
			if(need_depth_view)	depth_view	=kinect_depthmap_view;
			if(need_depth_raw)	depth_raw	=kinect_depthmap_raw;
			if(need_depth_float)depth_float	=kinect_depthmap_float;
			
			
			//Compute depthmap and pointcloud data
			if(need_pointcloud)
			{
				//Depthmaps are computed in send_pointcloud() for efficiency. This also sends the pointcloud.
				send_pointcloud(kinect_depthmap_gl,kinect_rgb,depth_view,depth_raw,depth_float);
			}
			else if(need_depth_view||need_depth_raw||need_depth_float)
			{
				//do conversion if it wasn't done above
				convert_depthmap(kinect_depthmap_gl,depth_view,depth_raw,depth_float);
			}
			
			//Publish rgb
			if(need_publish_rgb)
			{
				ros_publish_rgb();
			}
			
			//Publish the depthmaps
			if(need_publish_depth_raw)
			{
				ros_publish_depth_raw();
			}
			if(need_publish_depth_float)
			{
				ros_publish_depth_float();
			}
		}
	}
	
	//restore viewport
	restore_viewport();
	
	//Do the actual display of scene
	if(sensor.type==SENSOR_TYPE_KINECT_DEPTHMAP)
	{
		draw_depthmap();
	}
	else if(sensor.type==SENSOR_TYPE_KINECT_RGB)
	{
		draw_rgb();
	}
	else
	{
		//Render the regular scene
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		perspective(actor,sensor);
		render_scene();
	}
	
	//FIXME: Doesn't draw consistently sized pixels(might be fixed now?)
	if(sensor.type==SENSOR_TYPE_INFRARED||sensor.type==SENSOR_TYPE_ULTRASONIC)
		drawConvergence();
	//else if(sensor.type==SENSOR_TYPE_KINECT)
	//	clone_depth();
	
	///////////
	
	
#ifdef NO_DISPLAY
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
#endif
	
	//Render HUD
#ifndef	NO_LIGHTING
	glDisable(GL_LIGHTING);
#endif
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_TEXTURE_2D);
	render_hud(ttime);
	glEnable(GL_TEXTURE_2D);
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
}

void timer(int id)
{
#ifdef FPS
	glutTimerFunc(1000/FPS-2,timer,TIMER_ID);
#else
	glutTimerFunc(0,timer,TIMER_ID);
#endif
	
	//NOTE: The redisplay handler is called directly here. This is so the simulation won't pause when the window is minimized.
	//glutPostRedisplay();
	display();
}

// Reshape callback
void reshape(int w, int h)
{
	//Rendering will fail(?) if real window is smaller than sensor window
	//FIXME: Still needed?
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

void perspective_flipped(Actor actor,Sensor sensor)
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
	glScalef(1,1,-1);
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
