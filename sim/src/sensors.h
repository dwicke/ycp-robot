//Sensor parameters
// Cory Boyle 2011

///From robot.h
float bot_xtheta=0,bot_ztheta=0,//degrees
      bot_x=0,bot_y=0;//meters

float wheel_l_velocity=0,wheel_r_velocity=0;//speeds in m/s
int wheel_l_end=0,wheel_r_end=0;

///

#define SENSOR_TYPE_ULTRASONIC		1
#define SENSOR_TYPE_INFRARED		2
#define SENSOR_TYPE_HUMAN			3
#define SENSOR_TYPE_CAMERA			4
#define SENSOR_TYPE_3PCAM			5
#define SENSOR_TYPE_FIXEDCAM		6
#define SENSOR_TYPE_KINECT_DEPTHMAP	7
#define SENSOR_TYPE_KINECT_RGB		8

struct Sensor{
	const char *name;//Display name of sensor
	int type;
	
	float x,y,z;//position offset from center at ground
	float xtheta,ztheta;//angular offset
	
	//float pov;//pov
};

bool noise_enable=true;
float noise_factor=1;
#define NOISE_MAX	100.0
#define NOISE_D		2	//in units/sec

//Distance between wheels in cm
#define WHEEL_SPACING	30.0
//wheel diameter, 17cm

//Apply noise and limits for motor
//Noise is simulated in apply_motors()
float motor_sim(float velocity)
{
	//Range: 0-75cm/s (float)
	if(velocity>75)
		velocity=75;
	else if(velocity<-75)
		velocity=-75;
	return velocity/100.0;
}

//Velocity noise function is +/- (θ^2/120000+1)*x/75
void apply_motors(int ttime,int eltime,float &dlx,float &dly,float &theta)
{
	float sl=0,sr=0;
	//(θ^2/120000+1)*x/75
	
	//TODO: fix
	////////////////////////////////////////////////////////NEED SLIP FUNCTION
	
	//move from ltime to ttime.
	//wheel_*_start, wheel_*_end
	
	//time (in ms) to apply motion on this frame
	int wheel_l_remaining=wheel_l_end-ttime,
		wheel_r_remaining=wheel_r_end-ttime;
	
	if(wheel_l_remaining>0)
	{//command still valid
		if(wheel_l_remaining>eltime)
			wheel_l_remaining=eltime;
		sl=wheel_l_velocity*(((float)wheel_l_remaining)/1000.0);
	}
	
	if(wheel_r_remaining>0)
	{//command still valid
		if(wheel_r_remaining>eltime)
			wheel_r_remaining=eltime;
		sr=wheel_r_velocity*(((float)wheel_r_remaining)/1000.0);
	}
	
	//compute+update displacement
	//length of arc
	float sc = (sl+sr)/2;

	//angle of rotation
	theta = (sl-sr) / (WHEEL_SPACING/100.0);

	float dl;
	//sanity: if angle is 0
	if(theta==0)
		dl=sl;
	else
		//compute length of chord (length of displacement vector)
		dl=2*(sc/theta)*sin(theta/2);

	//Angle of displacement vector
	float dt=theta/2;


	//decompose the result vector
	float
		sin_dt=sin(dt),
		cos_dt=cos(dt);

	dlx=sin_dt*dl;
	dly=cos_dt*dl;
	
	
	if(noise_enable)
	{
		float noise;
		noise=((pow(theta,2)/120000.0+1.0)*dly/75.0)*noise_factor;
		theta+=((float)rand()/(float)RAND_MAX)*noise*2-noise;
		dly+=((float)rand()/(float)RAND_MAX)*noise*2-noise;
	}
}

//Average of randomly selected 7 of 9 points, noise function +/- x^3/50000
float ir_sim(float d)
{
	//noise first
	if(noise_enable)
	{
		float noise=(pow(d,3)/50000.0)*noise_factor;
		if(noise>d*.1)
		{//limit max noise to 10%
			noise=d*.1;
		}
		
		d+=((float)rand()/(float)RAND_MAX)*noise*2-noise;
	}
	
	//Range: 7.5-80cm (float)
	if(d<7.5)
	{//Under: Random value 7.5-80
		if(noise_enable)
			d=7.5+((float)rand()/(float)RAND_MAX)*(80-7.5);
		else
			d=7.5;
	}
	else if(d>80)
	{//Over:  Clamp to max
		d=80;
	}
	return d;
}

//Minimum of randomly selected 3 of 9 points, noise function +/- 2
float us_sim(float d)
{
	//noise first
	if(noise_enable)
	{
		d+=(((float)rand()/(float)RAND_MAX)*4-2)*noise_factor;
	}
	
	//Range: 4-255 (uint8)
	if(d<4)
	{//Under: Clamp to min
		d=4;
	}
	else if(d>255)
	{//Over:  Clamp to max
		d=255;
	}
	return d;
}

#define NUM_SENSORS	17
int
	sensor_ultrasonic_frontLeft,
	sensor_ultrasonic_frontCenter,
	sensor_ultrasonic_frontRight,
	sensor_ultrasonic_rearRight,
	sensor_ultrasonic_rearCenter,
	sensor_ultrasonic_rearLeft,

	sensor_infrared_frontLeftLeft,
	sensor_infrared_frontLeftCenter,
	sensor_infrared_frontRightCenter,
	sensor_infrared_frontRightRight,
	sensor_infrared_right,
	sensor_infrared_rear,
	sensor_infrared_left,
	
	sensor_kinect_depthmap,
	sensor_kinect_rgb;

Sensor* load_X80SVP_sensors()
{
	Sensor *sensors=(Sensor*)malloc(NUM_SENSORS*sizeof(Sensor));
	if(sensors==NULL)lerr("Out of memory (probably bad sensor_count)");
	
	//ultrasonic
	int i=0;
	sensors[i].type=	SENSOR_TYPE_CAMERA;
	sensors[i].name=	"POV Camera";
	sensors[i].x=		  0;
	sensors[i].y=		+13;
	sensors[i].z=		+15;
	sensors[i].xtheta=	  0;
	sensors[i].ztheta=	  0;
	
	i++;
	sensors[i].type=	SENSOR_TYPE_3PCAM;
	sensors[i].name=	"3P Camera";
	sensors[i].x=		  0;
	sensors[i].y=		-100;
	sensors[i].z=		+100;
	sensors[i].xtheta=	+30;
	sensors[i].ztheta=	  0;
	
	sensor_kinect_depthmap=++i;
	sensors[i].type=	SENSOR_TYPE_KINECT_DEPTHMAP;
	sensors[i].name=	"Kinect Depthmap";
	sensors[i].x=		  0;
	sensors[i].y=		+13;
	sensors[i].z=		+15;
	sensors[i].xtheta=	  0;
	sensors[i].ztheta=	  0;
	
	sensor_kinect_rgb=++i;
	sensors[i].type=	SENSOR_TYPE_KINECT_RGB;
	sensors[i].name=	"Kinect RGB";
	sensors[i].x=		  0;
	sensors[i].y=		+13;
	sensors[i].z=		+15;
	sensors[i].xtheta=	  0;
	sensors[i].ztheta=	  0;
	
	sensor_ultrasonic_frontLeft=++i;
	sensors[i].type=	SENSOR_TYPE_ULTRASONIC;
	sensors[i].name=	"US frontLeft";
	sensors[i].x=		-12;
	sensors[i].y=		+13;
	sensors[i].z=		+11;
	sensors[i].xtheta=	  0;
	sensors[i].ztheta=	-45;

	sensor_ultrasonic_frontCenter=++i;
	sensors[i].type=	SENSOR_TYPE_ULTRASONIC;
	sensors[i].name=	"US frontCenter";
	sensors[i].x=		  0;
	sensors[i].y=		+19;
	sensors[i].z=		+11;
	sensors[i].xtheta=	  0;
	sensors[i].ztheta=	  0;

	sensor_ultrasonic_frontRight=++i;
	sensors[i].type=	SENSOR_TYPE_ULTRASONIC;
	sensors[i].name=	"US frontRight";
	sensors[i].x=		+12;
	sensors[i].y=		+13;
	sensors[i].z=		+11;
	sensors[i].xtheta=	  0;
	sensors[i].ztheta=	+45;

	sensor_ultrasonic_rearRight=++i;
	sensors[i].type=	SENSOR_TYPE_ULTRASONIC;
	sensors[i].name=	"US rearRight";
	sensors[i].x=		+12;
	sensors[i].y=		-11;
	sensors[i].z=		+16;
	sensors[i].xtheta=	  0;
	sensors[i].ztheta=	+135;

	sensor_ultrasonic_rearCenter=++i;
	sensors[i].type=	SENSOR_TYPE_ULTRASONIC;
	sensors[i].name=	"US rearCenter";
	sensors[i].x=		  0;
	sensors[i].y=		-17;
	sensors[i].z=		+16;
	sensors[i].xtheta=	  0;
	sensors[i].ztheta=	180;

	sensor_ultrasonic_rearLeft=++i;
	sensors[i].type=	SENSOR_TYPE_ULTRASONIC;
	sensors[i].name=	"US rearLeft";
	sensors[i].x=		-12;
	sensors[i].y=		-11;
	sensors[i].z=		+16;
	sensors[i].xtheta=	  0;
	sensors[i].ztheta=	-145;

	//infrared
	sensor_infrared_frontLeftLeft=++i;
	sensors[i].type=	SENSOR_TYPE_INFRARED;
	sensors[i].name=	"IR frontLeftLeft";
	sensors[i].x=		-10;
	sensors[i].y=		+16;
	sensors[i].z=		+11;
	sensors[i].xtheta=	  0;
	sensors[i].ztheta=	-33.75;

	sensor_infrared_frontLeftCenter=++i;
	sensors[i].type=	SENSOR_TYPE_INFRARED;
	sensors[i].name=	"IR frontLeftCenter";
	sensors[i].x=		- 4;
	sensors[i].y=		+19;
	sensors[i].z=		+11;
	sensors[i].xtheta=	  0;
	sensors[i].ztheta=	-11.25;

	sensor_infrared_frontRightCenter=++i;
	sensors[i].type=	SENSOR_TYPE_INFRARED;
	sensors[i].name=	"IR frontRightCenter";
	sensors[i].x=		+ 4;
	sensors[i].y=		+19;
	sensors[i].z=		+11;
	sensors[i].xtheta=	  0;
	sensors[i].ztheta=	+11.25;

	sensor_infrared_frontRightRight=++i;
	sensors[i].type=	SENSOR_TYPE_INFRARED;
	sensors[i].name=	"IR frontRightRight";
	sensors[i].x=		+ 10;
	sensors[i].y=		+16;
	sensors[i].z=		+11;
	sensors[i].xtheta=	  0;
	sensors[i].ztheta=	+33.75;

	sensor_infrared_right=++i;
	sensors[i].type=	SENSOR_TYPE_INFRARED;
	sensors[i].name=	"IR right";
	sensors[i].x=		+11;
	sensors[i].y=		  0;
	sensors[i].z=		+20;
	sensors[i].xtheta=	  0;
	sensors[i].ztheta=	+90;

	sensor_infrared_rear=++i;
	sensors[i].type=	SENSOR_TYPE_INFRARED;
	sensors[i].name=	"IR rear";
	sensors[i].x=		  0;
	sensors[i].y=		-17;
	sensors[i].z=		+ 6;
	sensors[i].xtheta=	  0;
	sensors[i].ztheta=	180;

	sensor_infrared_left=++i;
	sensors[i].type=	SENSOR_TYPE_INFRARED;
	sensors[i].name=	"IR left";
	sensors[i].x=		-11;
	sensors[i].y=		  0;
	sensors[i].z=		+20;
	sensors[i].xtheta=	  0;
	sensors[i].ztheta=	-90;
	
	if(i+1!=NUM_SENSORS)lerr("SENSOR ERROR: %d!=%d",i+1,NUM_SENSORS);
	
	return sensors;
}
/*
//human
sensors[i].type=		SENSOR_TYPE_HUMAN;
sensors[i].name=		"Human left";
sensors[i].x=		-12;
sensors[i].y=		+ 8;
sensors[i].z=		+22;
sensors[i].xtheta=	0;//NOT USED
sensors[i].ztheta=	0;//NOT USED

sensors[i].type=		SENSOR_TYPE_HUMAN;
sensors[i].name=		"Human right";
sensors[i].x=		+12;
sensors[i].y=		;
sensors[i].z=		22;
sensors[i].xtheta=	0;//NOT USED
sensors[i].ztheta=	0;//NOT USED
*/