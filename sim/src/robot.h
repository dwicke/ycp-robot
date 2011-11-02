float bot_xtheta=0,bot_ztheta=0,//degrees
      bot_x=0,bot_y=0;//meters

float wheel_l_velocity=0,wheel_r_velocity=0;//speeds in m/s
int wheel_l_end=0,wheel_r_end=0;




/*
Ok, this seems to be escaping me.. Given the ground-speed-velocity (meters/sec) for 2 wheels, the distance between them (meters), and time (seconds), I need to find the resulting desplacement (of the point centered between them) in X, Y and Theta... Ideas?

Ok, this seems to be escaping me.. Given the displacement (meters) for 2 wheels, and the distance between them (meters), I need to find the resulting desplacement (of the point centered between them) in X, Y and Theta... Ideas?
Its for a robot simulator.
If velocity_left = velocity_right, only Y will change (moving straight forward). If velocity_left = -velocity_right, only Theta will change (turning in place). If 



I know the lengths of 2 arcs, and the distence between them. How do I find their radius?

geometric displacement along an arc


22:10 < CoJaBo> Ok, this seems to be escaping me.. Given the displacement (meters) for 2 wheels, and the distance between them (meters), I need to find the resulting desplacement (of the point centered between them) in X, Y and Theta... Ideas?
22:12 < PlanckWalk> CoJaBo: displacement as a vector?  From what?  What is theta?
22:13 < CoJaBo> PlanckWalk: The direction it is facing after the wheels move.
22:15 < PlanckWalk> CoJaBo: Are you assuming front wheels at constant angle from the back wheels, and back wheels following the front wheels, or something else?
22:15 < PlanckWalk> CoJaBo: Also, which displacement?  When turning, all the wheels move different distances
22:16 < CoJaBo> PlanckWalk: Assuming a 2-wheeled vehicle (theres a third "ball" shaped one at the rear so it doesn't fall over, but it should be able to be disregarded)
22:17 < CoJaBo> PlanckWalk: I need to find out how the point centered between the 2 wheels moves.
22:17 < PlanckWalk> CoJaBo: In a circle, if the angle is constant.


22:18 < PlanckWalk> CoJaBo: Do you have a diagram labelling all the quantities you know and the ones you want to find?
22:18 < CoJaBo> PlanckWalk: The wheels are side by side:  left wheel-> | [middle point] | <- right wheel
22:20 < PlanckWalk> CoJaBo: There's still lots of questions.  E.g. is the displacement for each wheel controllable independently?  If so, is it assumed to be constant ratio throughout a given segment of movement?
22:21 < CoJaBo> PlanckWalk: Maybe wrong term.. By displacement of the wheel, I mean it turns enough that it would move forward by that amount if both are turning "forward". If turning the same amount in oposite directions, it would spin in place.
22:24 < PlanckWalk> CoJaBo: Okay, it's starting to get a little clearer.  So the amount of turning is independent, and assumed to be constant ratio?


22:26 < CoJaBo> PlanckWalk: Those two cases would be trivial, but of course, the wheels could be moving by different amounts (e.g. left moves forward 1m, right 2m, which should curve it to the left).
22:27 < PlanckWalk> CoJaBo: so if one wheel moves 1m and the other moves 2m, then the 2m wheel is twice as far from the center of rotation
22:29 < PlanckWalk> And so the midpoint between the wheels moves around a circle through a distance of 1.5m, with the circle centered a distance 1.5d away (where d is the distance between the wheels)
22:32 < PlanckWalk> CoJaBo: So the angle rotated is (1.5m / (1.5 d)) radians
22:32 < CoJaBo> PlanckWalk: Does that still hold true when the wheels are moving at equal speeds, both in the same and oposite directions?
22:33 < PlanckWalk> CoJaBo: you can use the same reasoning for any ratio r


22:34 < CoJaBo> PlanckWalk: Thanks!, I'll try that..
22:37 < CoJaBo> PlanckWalk: Hm, where did you get the 1.5 from? If they were travelling equal speeds (say, 1m) tho, wouldnt that mean the circle would be 1d away, and thus it would model it as turning when it should be really moving straight forward?
22:37 < PlanckWalk> CoJaBo: The 1.5 was specific to that r=2 ratio
22:37 < CoJaBo> What would it be at moving left 1m, right 1m?
22:39 < CoJaBo> PlanckWalk: then how would I compute that "1.5" for arbitrary ratio?
22:39 < CoJaBo> PlanckWalk: Is there a way to write that programmatically? Has to be coded in C...
22:42 < PlanckWalk> CoJaBo: let the ratio be r, the radius L, and the angle theta.  Then r = (L + d/2) / (L - d/2), which you can solve for L.


22:48 < CoJaBo> PlanckWalk: How do I solve for L tho (in a way that can be coded in C..)
22:49 < PlanckWalk> CoJaBo: The usual way, multiply out to give (L - f/2) r = (L + d/2), collect terms of L on one side, etc.
22:52 < PlanckWalk> CoJaBo: ending up at  L = (d/2) (r + 1) / (r - 1).  Then  theta = s0 / L = 2 s0 (r-1) / [d (r+1)]
22:52 < PlanckWalk> Oops, mistake in theta
22:53 < PlanckWalk> CoJaBo: Oh wait, no it's not.  Just didn't mention that s0 = distance moved by center = average of distances for each wheel.

22:54 < PlanckWalk> CoJaBo: Though probably better to express in different terms.
22:56 < PlanckWalk> E.g. Call sl = left wheel displacement, sr = right wheel.  Then (after doing the substitutions)  sc = (sl+sr)/2,  theta = (sl-sr) / d

23:01 < CoJaBo> PlanckWalk: Ok, hope I'm not missing something you already said, but how do I then take those to get the displacement, from origin, in terms of X and Y cordinates as well as rotation?

23:02 < PlanckWalk> CoJaBo: the displacement from origin is the chord of a circle, with length sc and angle theta

23:06 < PlanckWalk> CoJaBo: So the straight-line distance travelled is  2 (sc/theta) sin(theta/2)  for theta != 0.

23:12 < CoJaBo> PlanckWalk: Is the direction traveled the same as theta, or is theta refering to something else?
23:12 < PlanckWalk> Theta is the angle the vehicle rotates
23:12 < PlanckWalk> So it's in the direction theta/2


23:26 < CoJaBo> PlanckWalk: Ok, then so sc is the length of the arc, 2 (sc/theta) sin(theta/2) gives me the length of the displacement vector, and theta/2 gives the angle of the displacement vector?
23:26 < PlanckWalk> CoJaBo: That's the one
23:27 < CoJaBo> PlanckWalk: And then the rotation angle is just theta?
23:27 < PlanckWalk> CoJaBo: Yes
23:30 < CoJaBo> PlanckWalk: Ok, HUGE thanks! Now, assuming I can remember my trig, I should finally be able to get the all-important X and Y components :P

sc = 1
theta = 0/d

sc = 1.5
theta = 1 / d

sc = 0
theta = 2 / d


{

//compute+update displacement
//length of arc
sc = (sl+sr)/2

//angle of rotation
theta = (sl-sr) / d

//sanity: if 

//compute length of chord (length of displacement vector)
dl=2 (sc/theta) sin(theta/2)

//Angle of displacement vector
dt=theta/2


}
XX

r(L - d/2) = (L + d/2)
2rL - 2rd = 2L + d

2rL - 2L = 2rd + d

2 (rL - L) = 2rd + d

(rL - L) = rd + d/2
XX

L = (d/2) (r + 1) / (r - 1)


L = (d/2) (r + 1) / (r - 1)

*/
/*

float bot_xtheta=0,bot_ztheta=0,//degrees
      bot_x=0,bot_y=0;//meters

float wheel_l=0,wheel_r=0;//speeds in m/s

//setMotors()?

//readSensors()?


//stuck  CON 100  '
#define stuck	100	//?
#define speed	.1	//m/s

function nav()
{//Navi.bs2
	int st=0,
	il,ir;

	st=st+4-((il+ir)*2);//no idea where this equation comes from

*/
/*    IF st>=stuck THEN
      FREQOUT spkrP,beep2,beepN,beepN

      FOR st=1 TO 44
        PULSOUT motR,nClkW
        PULSOUT motL,nClkW
        PAUSE 20
      NEXT
      st=0
    ENDIF
    'DEBUG DEC st,CR
    IF st THEN
      st=st-1
    ENDIF

    IF il AND ir THEN ' All clear!
      sl=cClkW
      sr=nClkW
    ELSEIF ir=0 THEN  ' Somthing ahead right...
      sl=nClkW
      sr=nClkW
    ELSEIF il=0 THEN  ' Somthing ahead left...
      sl=cClkW
      sr=cClkW
    ENDIF

*/


/*
function read_motors()
{
	chdir("iodata");
		FILE * f=fopen("motors.state","r");
	chdir("..");
	if( f == NULL) lerr("File 's' not found");
	
	char line[MAX_LEN+1],token[MAX_LEN+1],param[MAX_LEN+1];
	
	//read file
	while(fgets(line,MAX_LEN,f))
	{
		//token[0]=0;//clear
		if(sscanf(line,"%s %s ",&token,&param)!=2)
			continue;//Skip blank lines
		
		if(strcmp("Frame",token)==0)
		{
			if(strcmp("RootFrame",param)==0)continue;//ignore the root frame
			
			//init mesh
			lm=m;
			
			m=(mesh*)malloc(sizeof(mesh));
			if(m==NULL)lerr("Out of memory creating mesh - this shouldn't happen");
			
			//store addr to it
			if(first==NULL)
				first=m;
			else
			{
				if(!read_matrix)lerr("Missing matrix");
				init_mesh(*lm);
				(*lm).next=m;
			}
			
			//init vars
			strcpy((*m).name,param);
			printf("\tLoading frame: %s\n",m->name);
			//geom info
			(*m).vertices_n=0;(*m).normals_n=0;
			//material info
			(*m).materials_n=0,(*m).mappings_n=0;
			//coords
			(*m).coords_n=0;
			
			(*m).next=NULL;
			read_matrix=false;
		}
		else if(strcmp("FrameTransformMatrix",token)==0)
		{
			if(first==NULL)
				continue;//still in root frame
			if(read_matrix)lerr("Frame must contain only one mesh - duplicate Mesh definition");
			read_matrix=true;
			parse_matrix(f,(*m).static_transform);
		}
		else if(strcmp("Mesh",token)==0)
		{
			if((*m).vertices_n)lerr("Frame must contain only one mesh - duplicate Mesh definition");
			parse_vectors(f,(*m).vertices_n,(*m).vertices_v);
		}
		else if(strcmp("MeshNormals",token)==0)
		{
			if((*m).normals_n)lerr("Frame must contain only one mesh - duplicate MeshNormals definition");
			parse_vectors(f,(*m).normals_n,(*m).normals_v);
		}
		else if(strcmp("MeshMaterialList",token)==0)
		{
			if((*m).mappings_n)lerr("Frame must contain only one mesh - duplicate MeshMaterialList definition");
			parse_materials(f,(*m).materials_n,(*m).materials_v,(*m).mappings_n,(*m).mappings_v);
		}
		else if(strcmp("MeshTextureCoords",token)==0)
		{
			if((*m).coords_n)lerr("Frame must contain only one mesh - duplicate MeshTextureCoords definition");
			parse_texture_coords(f,(*m).coords_n,(*m).coords_v);
		}
			
	}
	fclose(f);
*/