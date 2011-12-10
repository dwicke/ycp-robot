//assorted extra functions
// Cory Boyle 2011

//#define cot(theta) (1/tan(theta))
#define rad(deg) ((deg) * (3.14/180.0))
#define deg(rad) ((rad) * (180.0/3.14))
#define X	0
#define Y	1
#define Z	2

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

void initMatrix(GLfloat m[16])
{
	memset(m,0,sizeof(GLfloat)*16);
	for(int i=0;i<16;i+=5)m[i]=1;
}

//for debug
void printMatrix(GLfloat m[16])
{
	for(int i=0;i<4;i++)
	{
		printf("%f\t%f\t%f\t%f\n",m[i],m[i+4],m[i+8],m[i+12]);
	}
	printf("\n");
}

void glxShear(GLfloat factor,GLuint axis,GLuint by)
{
	GLfloat m[4][4];
	
	//init identity
	initMatrix((GLfloat*)m);
	
	//skew axis by factor
	m[axis][by]=factor;
	
	//scale too
	m[axis][axis]=(cos(rad(factor*180.0))+1.0)/2.0;
	
	//apply transform
	glMultMatrixf((GLfloat*)m);
}
