#define ACTOR_MAX_MESHES	10

struct named_node{
	const char *name;
	struct node *node;
};

struct Actor{
	const char *name;//Display name of sensor
	
	float x,y,z;
	float xtheta,ztheta;
	
	//sensors
	int sensor_count,sensor_active;
	Sensor *sensors;///arr
	
	struct named_node mesh[ACTOR_MAX_MESHES];
	int mesh_active;
	
	//struct node *node;
	
	//ros name thing?
};

//nodes(scene graph meshes)
node root,camera,robot,
wheels,IR_block,US_block,
nullmesh,box,fence,table,suzanne;

#define NUM_OBSTACLES	10
#define NUM_ACTORS		2+NUM_OBSTACLES

#define ACTOR_CAMERA	0
#define ACTOR_ROBOT		1

Sensor obstacle_3p_sensor;

Actor actors[NUM_ACTORS];
void init_actors()
{
	int i,ii;
	Sensor *s;
	
	i=0;
	actors[i].name="Camera";
	actors[i].x=0;
	actors[i].y=0;
	actors[i].z=1.7;
	actors[i].xtheta=0;
	actors[i].ztheta=0;
	actors[i].mesh_active=0;
	actors[i].mesh[0].name="Camera";
	actors[i].mesh[0].node=&camera;
	actors[i].mesh[1].node=0;

	i=1;
	actors[i].name="Robot";
	actors[i].x=0;
	actors[i].y=0;
	actors[i].z=0;
	actors[i].xtheta=0;
	actors[i].ztheta=0;
	actors[i].mesh_active=0;
	actors[i].mesh[0].name="X80SVP";
	actors[i].mesh[0].node=&robot;
	actors[i].mesh[1].node=0;
	
	for(i=2;i<NUM_ACTORS;i++)
	{
		char *name=(char*)malloc(MAX_LEN*sizeof(char));
		if(name==NULL)lerr("Out of memory (this should never happen)");
		name[0]='A';
		name[1]=0;
		
		actors[i].name=name;
		actors[i].x=0;
		actors[i].y=0;
		actors[i].z=0;
		actors[i].xtheta=0;
		actors[i].ztheta=0;
		actors[i].mesh_active=0;
		
		int m=0;
		actors[i].mesh[m].name="None";
		actors[i].mesh[m].node=&nullmesh;
		
		m++;
		actors[i].mesh[m].name="Box";
		actors[i].mesh[m].node=&box;
		
		m++;
		actors[i].mesh[m].name="Fence";
		actors[i].mesh[m].node=&fence;
		
		m++;
		actors[i].mesh[m].name="Table";
		actors[i].mesh[m].node=&table;
		
		m++;
		actors[i].mesh[m].name="Suzanne";
		actors[i].mesh[m].node=&suzanne;
		
		m++;
		actors[i].mesh[m].node=0;
		
		actors[i].sensor_count=1;
		actors[i].sensor_active=0;
		actors[i].sensors=&obstacle_3p_sensor;
	}
	//Table
	actors[NUM_ACTORS-NUM_OBSTACLES+1].mesh_active=3;
	actors[NUM_ACTORS-NUM_OBSTACLES+1].x=-6;
	actors[NUM_ACTORS-NUM_OBSTACLES+1].y=+7;
	
	//Box
	actors[NUM_ACTORS-NUM_OBSTACLES+0].mesh_active=1;
	actors[NUM_ACTORS-NUM_OBSTACLES+0].x=-2;
	actors[NUM_ACTORS-NUM_OBSTACLES+0].y=+2;
	
	obstacle_3p_sensor.type=	SENSOR_TYPE_3PCAM;
	obstacle_3p_sensor.name=	"3P Camera";
	obstacle_3p_sensor.x=		  0;
	obstacle_3p_sensor.y=		-200;
	obstacle_3p_sensor.z=		+200;
	obstacle_3p_sensor.xtheta=	+30;
	obstacle_3p_sensor.ztheta=	  0;
	
	
	i=0;
	actors[i].sensor_count=3;
	s=(Sensor*)malloc(actors[i].sensor_count*sizeof(Sensor));
	if(s==NULL)lerr("Out of memory (probably bad sensor_count)");
	
	ii=0;
	s[ii].type=		SENSOR_TYPE_CAMERA;
	s[ii].name=		"Origin";
	s[ii].x=		0;
	s[ii].y=		0;
	s[ii].z=		0;
	s[ii].xtheta=	0;
	s[ii].ztheta=	0;
	
	ii++;
	s[ii].type=		SENSOR_TYPE_CAMERA;
	s[ii].name=		"50cm";
	s[ii].x=		0;
	s[ii].y=		0;
	s[ii].z=		-170+50;
	s[ii].xtheta=	0;
	s[ii].ztheta=	0;
	
	ii++;
	s[ii].type=		SENSOR_TYPE_CAMERA;
	s[ii].name=		"Sensor level";
	s[ii].x=		0;
	s[ii].y=		0;
	s[ii].z=		-170+11;
	s[ii].xtheta=	0;
	s[ii].ztheta=	0;
	
	actors[i].sensor_active=0;
	actors[i].sensors=s;
	
	//Robot
	i=1;
	actors[i].sensor_count=NUM_SENSORS;
	actors[i].sensor_active=0;
	actors[i].sensors=load_X80SVP_sensors();
}
int actor_active=1;//?




//movement factors
#ifdef _WIN32
#define MOUSE_SCALE	.2	//in degrees per pixel
#else
//#define MOUSE_SCALE	.01	//in degrees per pixel
#define MOUSE_SCALE	.05	//in degrees per pixel
#endif

#define player_dtheta	60	//in degrees/sec
#define player_dxy	5		//in meters/sec
//#define player_z	1.7		//initial height (m)
#define player_z	.15		//initial height (m)

//movement vars
//float player_xtheta=0,player_ztheta=0,player_x=0,player_y=0;

bool enable_motors=true;

bool check_bounds=false;

bool fullscreen=false;

#define CONVERGENCE_POINTS			3.0		//3x3 (9-point) grid
#define CONVERGENCE_WINDOW_SIZE_X	.1		//size of grid (in % window size)
#define CONVERGENCE_WINDOW_SIZE_Y	.01		//size of grid (in % window size)
//#define CONVERGENCE_WEIGHT			.1		//how fast to apply new convergence values, avoids "jitter" (%)
//#define CONVERGENCE_MIN				.5		//prevent camera from going cross-eyed (m)
//#define CONVERGENCE_DEBUG_DRAW	//for debug

#define osd_mode_count	4
int osd_mode=4;

void toggleFullscreen();
void mouseMove(int x,int y);
void mouseClick(int button, int state,int x, int y);

//controls movement

//doesnt seem to be a way to do this directly in linux, without loading heavy libraries
#ifndef _WIN32
bool keystate[128];
#endif

void init_keystate()
{
#ifndef _WIN32
	memset(keystate,0,sizeof(keystate));
#endif
}

void keydown(unsigned char key,int x, int y)
{
	switch(toupper(key))
	{
/*
		case '1':
			light=!light;
			if(light)
				glEnable(GL_LIGHT0);
			else
				glDisable(GL_LIGHT0);
			break;
		case '2':
			fan_on=!fan_on;
			break;
		case '3':
			door_dir*=-1;
			break;
		case '4':
			blinds_tilt_dir*=-1;
			break;
		case '5':
			blinds_open_dir*=-1;
			break;

		case 'C':
			stereo_ipd=stereo_ipd_initial;
			break;
		case 'B':
			stereo_convergence_auto=!stereo_convergence_auto;
			break;
*/

		case '8':
			if(actor_active>0)
				actor_active--;
			break;
		case '2':
			if(actor_active<NUM_ACTORS-1)
				actor_active++;
			break;
		case '4':
			if(actors[actor_active].sensor_active>0)
				actors[actor_active].sensor_active--;
			break;
		case '5':
			actor_active=0;
			actors[actor_active].sensor_active=0;
			break;
		case '6':
			if(actors[actor_active].sensor_active<actors[actor_active].sensor_count-1)
				actors[actor_active].sensor_active++;
			break;
		case '0':
			//FIXME: Memory leak.
			printf("BUG: Memory leak.\n");
			init_actors();
			noise_factor=1.0;
			noise_enable=true;
			enable_motors=true;
			check_bounds=false;
			actor_active=1;
			break;
			
		case '*':
			actors[actor_active].mesh_active++;
			if(actors[actor_active].mesh[actors[actor_active].mesh_active].node==0)
				actors[actor_active].mesh_active--;
			break;
		case '/':
			if(actors[actor_active].mesh_active>0)
				actors[actor_active].mesh_active--;
			break;
		case '.':
			actors[actor_active].mesh_active=0;
			break;
			
		case 'H':
			osd_mode=-osd_mode;
			break;
			
		case 'O':
			if(osd_mode<0)
				osd_mode=-osd_mode;
			else
				osd_mode+=1;
			if(osd_mode>osd_mode_count)
				osd_mode=1;
			break;
			
		case 'N':
			noise_enable=!noise_enable;
			break;
		
		case 'M':
			enable_motors=!enable_motors;
			break;
		
		case 'B':
			check_bounds=!check_bounds;
			break;
		case '`':
			toggleFullscreen();
			break;
		case VK_ESCAPE:
			exit(0);
	}
#ifndef _WIN32
	if(key<128)
		keystate[tolower(key)]=true;
#endif
//	printf("v [%d]\n",key,key);
}

void keyup(unsigned char key,int x, int y)
{
#ifndef _WIN32
	if(key<128)
		keystate[tolower(key)]=false;
#endif
	//printf("v [%d]\n",key,key);
}

bool getkey(unsigned char key)
{
#ifdef _WIN32
	//windows API
	return GetKeyState(key)&0x80;
#else
	//fallback
	//printf("? %c = %d\n",key,keystate[key]);
	if(key<128)
		return keystate[tolower(key)];
	return false;
#endif
}


void update_transforms(float ttime,float eltime);
void move(float ttime,float eltime);

#define WMAX	10

#ifndef _WIN32
	Display *dpy;
	Window root_window;
#endif
void init_xorg()
{
#ifndef _WIN32
	dpy = XOpenDisplay(0);
	root_window = XRootWindow(dpy, 0);
	XSelectInput(dpy, root_window, KeyReleaseMask);
#endif
}

void setMousePos(int x,int y)
{
#ifdef _WIN32
	SetCursorPos(x,y);
#else
	XWarpPointer(dpy, None, root_window, 0, 0, 0, 0, x,y);
	XFlush(dpy); // Flushes the output buffer, therefore updates the cursor's position. Thanks to Achernar.
#endif
}

void toggleFullscreen()
{
	fullscreen=!fullscreen;
	if(fullscreen)
	{
		glutFullScreen();
		
		glutMotionFunc(mouseMove);
		glutPassiveMotionFunc(mouseMove);
		glutSetCursor(GLUT_CURSOR_NONE);
		setMousePos(glutGet(GLUT_SCREEN_WIDTH)/2,glutGet(GLUT_SCREEN_HEIGHT)/2);
	}
	else
	{//turn off
		glutPositionWindow(0,0);
		glutReshapeWindow(INITIAL_WINDOW_WIDTH,INITIAL_WINDOW_HEIGHT);

		glutMotionFunc(NULL);
		glutPassiveMotionFunc(NULL);
		glutSetCursor(GLUT_CURSOR_INHERIT);
	}
}

void mouseMove(int x,int y)
{
	int cx=winx/2,
		cy=winy/2,
		dx=x-cx,
		dy=y-cy;
	if(dx==0&&dy==0)
		return;

	actors[actor_active].ztheta+=dx*MOUSE_SCALE;
	actors[actor_active].xtheta+=dy*MOUSE_SCALE;

	//clamp
	/**/ if(actors[actor_active].xtheta> 90)actors[actor_active].xtheta= 90;
	else if(actors[actor_active].xtheta<-90)actors[actor_active].xtheta=-90;
	
	setMousePos(cx,cy);
}
void mouseClick(int button, int state,int x, int y)
{
	//printf("%d=%d@%dx%d\n",button,state,x,y);

	if(state!=GLUT_DOWN)
		return;

	switch(button)
	{
	case 3:
		if(actor_active>0)
			actor_active--;
		break;
	case 4:
		if(actor_active<NUM_ACTORS-1)
			actor_active++;
		break;
	case GLUT_MIDDLE_BUTTON:
		actors[actor_active].sensor_active++;
		if(actors[actor_active].sensor_active>=actors[actor_active].sensor_count)
			actors[actor_active].sensor_active=0;
		break;
	case GLUT_RIGHT_BUTTON:
		actors[actor_active].mesh_active++;
		if(actors[actor_active].mesh[actors[actor_active].mesh_active].node==0)
			actors[actor_active].mesh_active=0;
		break;
	case GLUT_LEFT_BUTTON:
		toggleFullscreen();
		break;
	}
	
	
/*
 * 

	switch(button)
	{/*case '8':

		case '2':

		case '4':
			if(actors[actor_active].sensor_active>0)
				actors[actor_active].sensor_active--;
			break;
		case '5':
			actor_active=0;
			actors[actor_active].sensor_active=0;
			break;
		case '6':
			if(actors[actor_active].sensor_active<actors[actor_active].sensor_count-1)
				actors[actor_active].sensor_active++;
			break;
		case '0':
			//FIXME: Memory leak.
			printf("FIXME: Memory leak.\n");
			init_actors();
			noise_factor=1.0;
			noise_enable=true;
			enable_motors=true;
			check_bounds=false;
			actor_active=1;
			break;
			
		case '*':
			actors[actor_active].mesh_active++;
			if(actors[actor_active].mesh[actors[actor_active].mesh_active].node==0)
				actors[actor_active].mesh_active--;
			break;
		case '/':
			if(actors[actor_active].mesh_active>0)
				actors[actor_active].mesh_active--;
			break;
		case '.':
			actors[actor_active].mesh_active=0;
			break;
			
		case 'H':
			osd_mode=-osd_mode;
			break;//
	case 4:
		stereo_convergence_auto=false;stereo_convergence-=stereo_convergence_delta_scroll;
		break;
	case 3:
		stereo_convergence_auto=false;stereo_convergence+=stereo_convergence_delta_scroll;
		break;
	case GLUT_MIDDLE_BUTTON:
		stereo_convergence_auto=!stereo_convergence_auto;
		break;
	case GLUT_LEFT_BUTTON:
		if(!fullscreen)
		{
			toggleFullscreen();
			return;
		}
		break;
	case GLUT_RIGHT_BUTTON:
		light=!light;
		if(light)
			glEnable(GL_LIGHT0);
		else
			glDisable(GL_LIGHT0);
		break;
	}

*/
}

void move(int ttime,int ltime)
{
	int eltime=ttime-ltime;
	
	float elfactor=(eltime/1000.0);
	
	/**/ if(getkey('Q'))actors[actor_active].ztheta-=player_dtheta*elfactor;
	else if(getkey('E'))actors[actor_active].ztheta+=player_dtheta*elfactor;
	
	/**/ if(getkey('R'))actors[actor_active].xtheta-=player_dtheta*elfactor;
	else if(getkey('F'))actors[actor_active].xtheta+=player_dtheta*elfactor;
	
	//clamp
	/**/ if(actors[actor_active].xtheta> 90)actors[actor_active].xtheta= 90;
	else if(actors[actor_active].xtheta<-90)actors[actor_active].xtheta=-90;
	
	
	/**/ if(getkey('-')){noise_factor-=NOISE_D*elfactor;if(noise_factor<0)noise_factor=0;}
	else if(getkey('+')){noise_factor+=NOISE_D*elfactor;if(noise_factor>NOISE_MAX)noise_factor=NOISE_MAX;}
	
	
	//motor commands from ROS
	if(enable_motors)
	{
		float dlx,dly,theta;
		apply_motors(ttime,eltime,dlx,dly,theta);
		
		actors[ACTOR_ROBOT].ztheta+=deg(theta);
		
		/**/ if(actors[ACTOR_ROBOT].ztheta>360)
			actors[ACTOR_ROBOT].ztheta-=360;
		else if(actors[ACTOR_ROBOT].ztheta<0)
			actors[ACTOR_ROBOT].ztheta+=360;
		
		//ugh, trig.
		float
			sin_ztheta=sin(rad(actors[ACTOR_ROBOT].ztheta)),
			cos_ztheta=cos(rad(actors[ACTOR_ROBOT].ztheta));
		
		actors[ACTOR_ROBOT].x+= cos_ztheta*dlx+sin_ztheta*dly;
		actors[ACTOR_ROBOT].y+=-sin_ztheta*dlx+cos_ztheta*dly;
	}
	
	//ugh, trig.
	float dx=0,dy=0,
		sin_ztheta=sin(rad(actors[actor_active].ztheta)),
		cos_ztheta=cos(rad(actors[actor_active].ztheta));
	
	/**/ if(getkey('A'))dx+=-player_dxy*(eltime/1000.0);
	else if(getkey('D'))dx+=+player_dxy*(eltime/1000.0);
	/**/ if(getkey('S'))dy+=-player_dxy*(eltime/1000.0);
	else if(getkey('W'))dy+=+player_dxy*(eltime/1000.0);
	
	actors[actor_active].x+= cos_ztheta*dx+sin_ztheta*dy;
	actors[actor_active].y+=-sin_ztheta*dx+cos_ztheta*dy;
	
	if(check_bounds)
	{
		/**/ if(actors[actor_active].x> WMAX)actors[actor_active].x= WMAX;
		else if(actors[actor_active].x<-WMAX)actors[actor_active].x=-WMAX;
		/**/ if(actors[actor_active].y> WMAX)actors[actor_active].y= WMAX;
		else if(actors[actor_active].y<-WMAX)actors[actor_active].y=-WMAX;
	}
	
	/**/ if(actors[actor_active].ztheta>360)
		actors[actor_active].ztheta-=360;
	else if(actors[actor_active].ztheta<0)
		actors[actor_active].ztheta+=360;
	//update_transforms(ttime!!/1000.0!!,eltime);
}

/*
//update transforms
void update_transforms(float ttime,float eltime)
{
	//float xtheta=ttime*360.0;

	time_t rawtime;
	struct tm * timeinfo;
	time ( &rawtime );
	timeinfo = localtime ( &rawtime );
	
	//clock
	float 
		sec=timeinfo->tm_sec/60.0,
		min=(timeinfo->tm_min+sec)/60.0,
		hour=(timeinfo->tm_hour+min)/12.0;
	glLoadIdentity();
	glRotatef(sec*360.0,0,1,0);
	glGetFloatv(GL_MODELVIEW_MATRIX,clock_second.transform);
	glLoadIdentity();
	glRotatef(min*360.0,0,1,0);
	glGetFloatv(GL_MODELVIEW_MATRIX,clock_minute.transform);
	glLoadIdentity();
	glRotatef(hour*360.0,0,1,0);
	glGetFloatv(GL_MODELVIEW_MATRIX,clock_hour.transform);
	


	//fan
	if(fan_on)
		fan_theta+=eltime*360;

	glLoadIdentity();
	glRotatef(sin(rad(fan_theta*FAN_CAGE_SPEED))*90,0,0,1);
	glGetFloatv(GL_MODELVIEW_MATRIX,fan_cage.transform);
	
	glLoadIdentity();
	glRotatef(fan_theta*FAN_BLADE_SPEED,0,1,0);
	glGetFloatv(GL_MODELVIEW_MATRIX,fan_blade.transform);

	//switch
	
	glLoadIdentity();
	glRotatef(light?-SWITCH_TILT:SWITCH_TILT,1,0,0);
	glGetFloatv(GL_MODELVIEW_MATRIX,lightswitch.transform);

	//blinds tilt
	blinds_tilt_pos+=eltime*blinds_tilt_dir*BLINDS_TILT_SPEED;
	if(blinds_tilt_pos>BLINDS_TILT_MAX)
		blinds_tilt_pos=BLINDS_TILT_MAX;
	else if(blinds_tilt_pos<BLINDS_TILT_MIN)
		blinds_tilt_pos=BLINDS_TILT_MIN;

	//blinds open
	blinds_open_pos+=eltime*blinds_open_dir*BLINDS_OPEN_SPEED;
	if(blinds_open_pos>BLINDS_OPEN_MAX)
		blinds_open_pos=BLINDS_OPEN_MAX;
	else if(blinds_open_pos<BLINDS_OPEN_MIN)
		blinds_open_pos=BLINDS_OPEN_MIN;
	
	//blinds slats transform
	glLoadIdentity();
	glScalef(1,1,blinds_open_pos);
	glxShear(blinds_tilt_pos,Y,Z);
	glGetFloatv(GL_MODELVIEW_MATRIX,blinds.transform);

	//blinds bottom transform
	glLoadIdentity();
	glTranslatef(0,0,((1-blinds_open_pos)*BLINDS_HEIGHT));
	glGetFloatv(GL_MODELVIEW_MATRIX,blinds_bottom.transform);
	
	//door
	door_pos+=eltime*door_dir*DOOR_SPEED;
	if(door_pos>DOOR_MAX)
		door_pos=DOOR_MAX;
	else if(door_pos<DOOR_MIN)
		door_pos=DOOR_MIN;

	glLoadIdentity();
	glRotatef(door_pos,0,0,1);
	glGetFloatv(GL_MODELVIEW_MATRIX,door.transform);

	//snow
	snow_pos+=eltime*SNOW_DELTA;
	if(snow_pos>SNOW_HEIGHT)
		snow_pos=0;

	glLoadIdentity();
	glRotatef(SNOW_ANGLE,1,0,0);
	glTranslatef(0,0,-snow_pos);
	glGetFloatv(GL_MODELVIEW_MATRIX,snow.transform);
	
	//glLoadIdentity();
	//glScalef(0,0,0);
	//glxShear(sin(rad(theta/2))*1,Z,X);
	//glGetFloatv(GL_MODELVIEW_MATRIX,root.transform);
}
*/