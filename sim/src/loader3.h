/*
IMPORTANT- Mesh must be triangulated and right-handed!
Must not export in edit mode(blender bug)
Must have at least one material
Must "apply transform" (Ctrl+a) all meshes - otherwise static_transform isnt a simple translation, which screws things up
Scene graph relationships MUST match those in blender
*/


//fatal error :-(
void lerr(const char * format, ... )
{
	printf("FAIL: ");
	va_list args;
	va_start (args, format);
	vprintf (format, args);
	va_end (args);
	printf("\n");
	exit(1);
}

struct vector{
	float x,y,z;
};

struct material{
	char texture[MAX_LEN+1];
	int tex_id;
	float color[4];
	float specular;
};

struct uvcoords{
	float u,v;
};

struct texcache{
	char texture[MAX_LEN+1];
	int tex_id;
};

struct mesh{
	char name[MAX_LEN+1];
	
	//geom info
	int vertices_n,normals_n;
	vector *vertices_v,*normals_v;
	
	//material info
	int materials_n,mappings_n;
	material *materials_v;
	int *mappings_v;
	
	//coords
	int coords_n;
	uvcoords *coords_v;
	
	float static_transform[16];
	
	mesh * next;
};


struct node{
	float transform[16];		//user transform
	float static_transform[16];	//frame transform
	int list;					//display list
	bool transparent;
	node *sibling,*child;
};

int JUNKI;char *JUNKC;//avoid nonsense warnings

//returns transp
bool apply_material(material &m)
{
	glBindTexture(GL_TEXTURE_2D,m.tex_id);
#ifndef	NO_LIGHTING
	//Blender does not provide an ambient color
	glMaterialfv(GL_FRONT,GL_AMBIENT,m.color);
	glMaterialfv(GL_FRONT,GL_DIFFUSE,m.color);
	
	//compute a specular color by mixing with white
	float specular[4];
	for(int i=0;i<4;i++)
		specular[i]=m.color[i]+(1-m.color[i])*(m.specular/2);
	glMaterialfv(GL_FRONT,GL_SPECULAR,specular);
	//glMaterialfv(GL_FRONT,GL_SPECULAR,m.color);
	
	/*//if(m.color[3]<1){
	printf("Color: %f %f %f %f\n",m.color[0],m.color[1],m.color[2],m.color[3]);
	printf("Specular: %f %f %f %f\n",specular[0],specular[1],specular[2],specular[3]);
	//}*/
	//Blender calls this "Hard", but it does not get exported. Default is 50.
	glMaterialf(GL_FRONT,GL_SHININESS,50);
#else
	//Do nothing- colors will get assigned per-vertex in draw_mesh()
	
	//glColor3fv(m.color);
#endif
	return m.color[3]<1.0;
}

texcache texture_cache_v[MAX_TEXTURES];
int texture_cache_n=0;

//Parse vectors from file
//NOTE: vectors must be free()'d
void parse_vectors(FILE * f,int &count,vector * &vectors)
{
	if(fscanf(f,"%d; ",&count)!=1 || count<1)lerr("Invalid format reading vector value count");
	
	//Load the vector values
	vector *v;
	v=(vector*)malloc(count*sizeof(vector));
	if(v==NULL)lerr("Out of memory reading vector values - file currupt?");
	
	for(int i=0;i<count;i++)
	{
		if(fscanf(f,"%f; %f; %f;%*c ",&v[i].x,&v[i].y,&v[i].z)!=3)lerr("Invalid format reading vector values");
	}
	
	//Load the vector indexes
	int index_count=0;
	if(fscanf(f,"%d; ",&index_count)!=1 || index_count<1)lerr("Invalid format reading vector index count");
	
	if(count!=index_count*3)lerr("count!=index_count*3 - this shouldnt happen - is the mesh triangulated?");
	
	vectors=(vector*)malloc(count*sizeof(vector));
	if(vectors==NULL)lerr("Out of memory reading vector indexes - file currupt?");
	
	int a=0,b=0,c=0,j=0;
	for(int i=0;i<index_count;i++)
	{
		//Note: we only care to load triangulated meshes, so this will always be 3.
		if(fscanf(f,"3; %d, %d, %d;%*c ",&a,&b,&c)!=3)lerr("Invalid format reading vector indexes- is the mesh triangulated?");
		
		if(a<0||a>=count||b<0||b>=count||c<0||c>=count)lerr("Vector index out of bounds");
		vectors[j++]=v[a];
		vectors[j++]=v[b];
		vectors[j++]=v[c];
	}
	free(v);
}

void parse_matrix(FILE * f,float m[])
{
	//Load the matrix values
	for(int i=0;i<16;i+=4)
	{
		if(fscanf(f,"%f,%f,%f,%f,%*c ",
			&m[i  ],&m[i+1],&m[i+2],&m[i+3])!=4)lerr("Invalid format reading matrix values");
	}
}


/*

MeshMaterialList{
number of materials;
number of face-material mappings;
m,
m,
m;;
}

*/
//Parse materials
//NOTE: materials_v must be free()'d
//NOTE: mappings_v must be free()'d
void parse_materials(FILE * f,int &materials_n,material * &materials_v,int &mappings_n,int * &mappings_v)
{
	if(fscanf(f,"%d; ",&materials_n)!=1 || materials_n<1)lerr("Invalid format reading material count (loading of models with no material is not supported)");
	if(fscanf(f,"%d; ",&mappings_n)!=1 || mappings_n<1)lerr("Invalid format reading mapping count");

	mappings_v=(int*)malloc(mappings_n*sizeof(int));
	if(mappings_v==NULL)lerr("Out of memory reading map values - file currupt?");

	//read map
	for(int i=0;i<mappings_n;i++)
	{
		if(fscanf(f," %d%*c%*c ",&mappings_v[i])!=1 || mappings_v[i]<0)lerr("Invalid format reading map");
	}

	//read tex filenames
	char line[MAX_LEN+1],token[MAX_LEN+1];

	materials_v=(material*)malloc(materials_n*sizeof(material));
	if(materials_v==NULL)lerr("Out of memory materials - file currupt?");

	for(int i=0;i<materials_n;i++)
	{
		//printf("reading mat\n");
		line[0]=0;token[0]=0;//clear
		
		//make sure we're at the right spot
		JUNKC=fgets(line,MAX_LEN,f);
		sscanf(line,"%s",&token);
		if(strcmp("Material",token)!=0)lerr("Expected material");
		
		//Read material color
		if(fscanf(f,"%f; %f; %f; %f;; ",
			&materials_v[i].color[0],
			&materials_v[i].color[1],
			&materials_v[i].color[2],
			&materials_v[i].color[3])!=4)lerr("Invalid format reading material color");
		//Specularity
		if(fscanf(f,"%f; ",
			&materials_v[i].specular)!=1)lerr("Invalid format reading material specularity");
		
		//Blender does not seem to export these next 2 lines, so they are ignored
		JUNKC=fgets(line,MAX_LEN,f);JUNKC=fgets(line,MAX_LEN,f);

		//this will either be a TextureFilename or the end of material (token })
		line[0]=0;token[0]=0;//clear
		JUNKC=fgets(line,MAX_LEN,f);
		sscanf(line,"%s %*s \"%s %*s",&token,&materials_v[i].texture);
		//printf("Line: %s\nToken: %s\nMat: %s\n\n",line,token,&materials_v[i].texture);

		if(strcmp("TextureFilename",token)==0)
		{
			materials_v[i].texture[strlen(materials_v[i].texture)-2]=0;//cut 2 chars off
			JUNKC=fgets(line,MAX_LEN,f);//ignore eom line
		}
		else if(strcmp("}",token)==0)
		{//end of texture
			//initialize to ""
			materials_v[i].texture[0]=0;
		}
		else
		{
			lerr("Unexpected token '%s' reading texture",token);
		}
		//printf("Material: %s\n\n",materials_v[i].texture);
	}
}

//NOTE: coords must be free()'d
void parse_texture_coords(FILE * f,int &count,uvcoords * &v)
{
	if(fscanf(f,"%d; ",&count)!=1 || count<1)lerr("Invalid format reading texture coords count");
	
	//Load the uv values
	v=(uvcoords*)malloc(count*sizeof(uvcoords));
	if(v==NULL)lerr("Out of memory reading texture coords values - file currupt?");
	
	for(int i=0;i<count;i++)
	{
		if(fscanf(f,"%f;%f;%*c ",&v[i].u,&v[i].v)!=2)lerr("Invalid format reading texture coords values");
	}
	/*
	for(int i=0;i<count;i++)
	{
		printf("UV: %f;%f;\n",v[i].u,v[i].v);
	}*/
	
	//printf("parse_texture_coords() is not impl\n");
}

int load_texture(char * texture)
{
	printf("\t\tLoading texture %s...",texture);
	
	for(int i=0;i<texture_cache_n;i++)
	{
		if(strcmp(texture_cache_v[i].texture,texture)==0)
		{
			printf("CACHED id=%d\n",texture_cache_v[i].tex_id);
			return texture_cache_v[i].tex_id;
		}
	}
	int tex_id = SOIL_load_OGL_texture(texture, SOIL_LOAD_AUTO, SOIL_CREATE_NEW_ID, SOIL_FLAG_MIPMAPS);
	
	if (tex_id==0)lerr("Texture load failed");
	
	//Set application mode
	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
	//glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	
	//Set scaling filters
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
	
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_NEAREST);
	
	//Set wrapping modes
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	
	printf("id=%d\n",tex_id);
	
	if(texture_cache_n>=MAX_TEXTURES)
		lerr("Too many textures- increase MAX_TEXTURES");
	strcpy(texture_cache_v[texture_cache_n].texture,texture);
	texture_cache_v[texture_cache_n].tex_id=tex_id;
	texture_cache_n++;
	
	return tex_id;
}


//Check mesh and load its textures
void init_mesh(mesh &m)
{
	//make sure all nessecery bits were got
	if(!m.vertices_n)lerr("File contains no geometry");
	if(m.vertices_n!=m.normals_n)lerr("vertices_n!=normals_n - File truncated?");
	//if(m.vertices_n!=m.coords_n)lerr("vertices_n(%d)!=coords_n(%d) - File truncated?",m.vertices_n,m.coords_n);
	
	//load textures
	JUNKI=chdir("textures");
		for(int i=0;i<m.materials_n;i++)
		{
			if(m.materials_v[i].texture[0]==0)
			{//No texture
				m.materials_v[i].tex_id=0;
				continue;
			}
			m.materials_v[i].tex_id=load_texture(m.materials_v[i].texture);
		}
	JUNKI=chdir("..");
}

//Renders a _triangulated_ mesh from file to display list.
//NOTE: This should be called from within the res/ directory, and expects models/ and textures/.
//void load_mesh(const char * file,GLuint list)

//Load meshes from file; returns pointer to first mesh
mesh * load_meshfile(const char * file)
{
	printf("Loading meshfile: %s\n",file);
	JUNKI=chdir("models");
		FILE * f=fopen(file,"r");
	JUNKI=chdir("..");
	if( f == NULL) lerr("File '%s' not found",file);
	
	mesh *first=NULL,*m=NULL,*lm;
	
	char line[MAX_LEN+1],token[MAX_LEN+1],param[MAX_LEN+1];
	bool read_matrix=false;
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
	if(!read_matrix)lerr("Missing matrix");
	init_mesh(*m);
	
	return first;
}


//draw mesh to list
bool draw_mesh(mesh &m)
{
	bool transparent=false;
//	set_material(GL_FRONT,&red_plastic);
//	//apply_material(m.materials_v[0]);
//	glutSolidSphere(1,200,200);
//	return;
	//Render mesh
	int vertex=0,face=0;
	while(vertex<m.vertices_n)
	{
		//set texture
		if(face>=m.mappings_n||face<0)
			lerr("Invalid face index");
		int mapping=m.mappings_v[face];
		if(mapping>=m.materials_n||mapping<0)
			lerr("Invalid mapping index");
		
		if(apply_material(m.materials_v[mapping]))
			transparent=true;
		//printf("id:%d\n",materials_v[mappings_v[face]].tex_id);
		
		//draw triangle
		glBegin(GL_TRIANGLES);
		{
			for(int i=1;i<=3;i++)
			{
			//printf("!%d!",m.materials_v[m.mappings_v[face]].tex_id);
				if(m.coords_n)
					glTexCoord2f(m.coords_v[vertex].u,m.coords_v[vertex].v);
			//	printf("Drawnorm: %f,%f,%f\n",m.normals_v[vertex].x,m.normals_v[vertex].y,m.normals_v[vertex].z);
#ifndef	NO_LIGHTING
				//NOTE: Causes an immediate system hang on GMA950 gfx
				glNormal3f(m.normals_v[vertex].x,m.normals_v[vertex].y,m.normals_v[vertex].z);
#else
				//Produce a pseudo-lighting effect by varying the vertex colors
				float vary=(m.normals_v[vertex].x+m.normals_v[vertex].y+m.normals_v[vertex].z)/10.0;
				
				glColor3f(m.materials_v[mapping].color[0]+vary,
						   m.materials_v[mapping].color[1]+vary,
						   m.materials_v[mapping].color[2]+vary);
#endif
			//	printf("Drawvert: %f,%f,%f\n",m.vertices_v[vertex].x,m.vertices_v[vertex].y,m.vertices_v[vertex].z);
				glVertex3f(m.vertices_v[vertex].x,m.vertices_v[vertex].y,m.vertices_v[vertex].z);
				vertex++;
			}
		}
		glEnd();
		face++;
	}
	return transparent;
}

void free_mesh(mesh * m)
{
	free((*m).vertices_v);
	free((*m).normals_v);
	free((*m).materials_v);
	free((*m).mappings_v);
	if((*m).coords_n)
		free((*m).coords_v);
	free(m);
}

int lclist=1;

//?? mesh name to scene graph node
//NOTE: This deletes the mesh from the list. param first may be altered
void load_mesh(mesh * &first,const char * name,node &g)
{
	mesh *m=first,*lm=NULL;
	if(m==NULL)lerr("Attempt to render null mesh!");

	//init node
	g.list=++lclist;
	initMatrix(g.transform);
	g.child=NULL;
	g.sibling=NULL;

	if(name[0]!=0)
	{//render single
		printf("Rendering: %s\n",name);
		for(;;)
		{
			if(strcmp(name,(*m).name)==0)
			{//this is it
				glNewList(g.list,GL_COMPILE);
				{
					glPushAttrib(GL_CURRENT_BIT);
					
					g.transparent=draw_mesh(*m);
					
					glPopAttrib();
				}
				glEndList();
				
				//store matrix
				memcpy(g.static_transform,(*m).static_transform,sizeof(g.static_transform));
				
				//unlink
				if(lm==NULL)
				{//rm first
				//	printf("rm first\n");
					first=(*m).next;
				}
				else
				{//rm other
				//	printf("rm other\n");
					(*lm).next=(*m).next;
				}
				//free
				free_mesh(m);
				return;
			}
			lm=m;
			//try next one
			m=(*m).next;
			if(m==NULL)
				lerr("Mesh '%s' not found!",name);
		}
	}
	else
	{//render everything else
		printf("Rendering all\n");
		g.transparent=false;
		//m=	(*m).next;//FIXME: DEBUG:!!!!
		
		//return;
		
		glNewList(g.list,GL_COMPILE);
		{
			glPushAttrib(GL_CURRENT_BIT);
			{
				for(;;)
				{
					printf("\tRendering: %s\n",(*m).name);
					glPushMatrix();
					{
						glMultMatrixf((*m).static_transform);
						if(draw_mesh(*m))
							lerr("Unsupported: transparent mesh in render all");
					}
					glPopMatrix();
					
					//next one
					lm=m;
					m=(*m).next;
					free_mesh(lm);
					if(m==NULL)
						break;
				}
			}
			glPopAttrib();
		}
		glEndList();
		initMatrix(g.static_transform);
		first=NULL;
	}
}

//get matrix, material?, 
/*
void xxx(mesh * &first,const char * name,node &g)
{
	mesh *m=first,*lm=NULL;
	if(m==NULL)lerr("Attempt to render null mesh!");

		printf("?ing: %s\n",name);
		for(;;)
		{
			if(strcmp(name,(*m).name)==0)
			{//this is it

					//draw_mesh(*m);
					
				
				//store matrix
				memcpy(g.static_transform,(*m).static_transform,sizeof(g.static_transform));
				
				//unlink
				if(lm==NULL)
				{//rm first
					first=(*m).next;
				}
				else
				{//rm other
					(*lm).next=(*m).next;
				}
				//free
				free_mesh(m);
				return;
			}
			
			//try next one
			m=(*m).next;
			if(m==NULL)
				lerr("Mesh '%s' not found!",name);
		}
	}

}

*/