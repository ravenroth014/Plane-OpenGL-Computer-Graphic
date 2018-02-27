/*____________________________________________________________________
|
| File: main.cpp
|
| Description: Hermite curve drawing.
|
| Functions:
|
| (C) Copyright 2007 Mores Prachyabrued
|___________________________________________________________________*/

// Enable older CRT functions (such as strcpy) without warnings from vc8 (vc 2005 .net)
#if _MSC_VER >= 1400
#define _CRT_SECURE_NO_DEPRECATE				// shut up the vc8 compiler
#define _CRT_NONSTDC_NO_DEPRECATE
#endif

/*___________________
|
| Include Files
|__________________*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#include <gmtl/gmtl.h>

#include <GL/glut.h>

/*___________________
|
| Constants
|__________________*/

const int WIN_WIDTH_INIT = 800;
const int WIN_HEIGHT_INIT = 600;

const float C_TSTEP = 0.01f;            // Curve time step (for rendering curve)

const gmtl::Vec3f WORLD_UP(0, 1, 0);                  // World up axis (Y)

													  //First Route
const int PT_NB = 6;                                  // Number of input points (equals #tangents and #segments)
const gmtl::Point3f input_pts[PT_NB] =
{
	gmtl::Point3f(-180, -25, 150),
	gmtl::Point3f(0, -50, 0),
	gmtl::Point3f(-180, 0, -150),
	gmtl::Point3f(0, 25, -150),
	gmtl::Point3f(-180, -10, 0),
	gmtl::Point3f(0, -50, 150)

};

//Second Route
const int SECOUND_PT_NB = 6;
const gmtl::Point3f second_input_pts[SECOUND_PT_NB] =
{
	gmtl::Point3f(0, 25, -150),
	gmtl::Point3f(-180, -10, 0),
	gmtl::Point3f(0, -50, 150),
	gmtl::Point3f(-180, -25, 150),
	gmtl::Point3f(0, -50, 0),
	gmtl::Point3f(-180, 0, -150)
};

gmtl::Matrix44f MMAT;                                 // Basis matrix for Hermite curve form (const)

const float R_MAX = 2.5f*1200.0f;                     // Expected maximum magnitude of local rightward component of plane's acceleration

const int PARTICLE_NB = 20;                    		  // Number of particles
const int F_PARTICLE_NB = 20;

const float VMAG_MEAN = 100.0f;                       // Velocity
const float VMAG_STD = 25.0f;
const float VDIR_STD = 0.25f;

const int TTL_BASE = 200;                          // Time to live
const int TTL_OFFSET = 50;

const float SMOKE_SIZE = 1.5f;                         // Smoke size
const float FIRE_SIZE = 0.5f;

const float S_TSTEP = 0.01f;                          // Simulation time step (for particle update)
const float FS_TSTEP = 0.1f;

const gmtl::Vec3f GRAVITY(0, -100.0f, 0);                 // w.r.t. world, can be upward for smoke
const gmtl::Vec3f FGRAVITY(0, -10.0f, 0);

const gmtl::Vec3f V_WIND(100, 0, 0);                  // Wind velocity and drag coefficient (K)
const float K_COEF = 1.0f;

// Keyboard modifiers
enum KeyModifier { KM_SHIFT = 0, KM_CTRL, KM_ALT };

const GLfloat NO_LIGHT[] = { 0.0, 0.0, 0.0, 1.0 };
const GLfloat AMBIENT_LIGHT[] = { 0.1, 0.1, 0.1, 1.0 };
const GLfloat DIFFUSE_LIGHT[] = { 0.5, 0.5, 0.5, 1.0 };
const GLfloat SPECULAR_LIGHT[] = { 0.5, 0.5, 0.5, 1.0 };

// Materials
const GLfloat DARKRED_COL[] = { 0.1, 0.0, 0.0, 1.0 };
const GLfloat BRIGHTRED_COL[] = { 0.7, 0.0, 0.0, 1.0 };
const GLfloat DARKBLUE_COL[] = { 0.0, 0.0, 0.1, 1.0 };
const GLfloat BRIGHTBLUE_COL[] = { 0.0, 0.0, 0.7, 1.0 };
const GLfloat DARK_COL[] = { 0.1, 0.1, 0.1, 1.0 };
const GLfloat MEDIUMWHITE_COL[] = { 0.7, 0.7, 0.7, 1.0 };
const GLfloat SPECULAR_COL[] = { 0.7, 0.7, 0.7, 1.0 };

//My Own Materials
const GLfloat DARKPLANEBODY_COL[] = { 0.117, 0.180, 0.196, 1.0 };
const GLfloat BRIGHTPLANEBODY_COL[] = { 0.412, 0.663, 0.725, 1.0 };
const GLfloat DARKPLANEWING_COL[] = { 0.035, 0.172, 0.208, 1.0 };
const GLfloat BRIGHTPLANEWING_COL[] = { 0.129, 0.745, 0.898, 1.0 };
const GLfloat DARKPLANEJET_COL[] = { 0.0, 0.0, 0.0, 1.0 };
const GLfloat BRIGHTPLANEJET_COL[] = { 0.0, 0.373, 0.431, 1.0 };
const GLfloat DARKMISSILE_COL[] = { 0.1, 0.0, 0.0, 1.0 };
const GLfloat BRIGHTMISSILE_COL[] = { 0.9, 0.1, 0.0, 1.0 };
const GLfloat DARKLIGHT_COL[] = { 0.0, 0.0, 0.0, 1.0 };
const GLfloat BRIGHTLIGHT_COL[] = { 1.0, 1.0, 0.4, 1.0 };

const float SB_SIZE = 850.0f;                     // Skybox dimension

enum TextureID {
	TID_SKYBACK = 0, TID_SKYLEFT, TID_SKYBOTTOM, TID_SKYRIGHT,
	TID_SKYFRONT, TID_SKYTOP, TID_TERRAIN_1, TID_TERRAIN_2, TID_SHORE, TID_FIRE, TEXTURE_NB
};  // Texture IDs, with the last ID indicating the total number of textures


	/*___________________
	|
	| Type Definitions
	|__________________*/

	// Particle structure storing position, velocity, and other properties
typedef struct _MyParticle {
	gmtl::Point3f p;      // Position
	gmtl::Vec3f v;        // Velocity
	float m;              // Mass
	int ttl;              // Time to live, decremented each iteration
	int ttl_init;	  			// Initial ttl, used to fade color as the age increases
} MyParticle;

/*___________________
|
| Global variables
|__________________*/

// camera w.r.t. plane
float distance = 300.0f;
float elevation = -15.0f;                 // In degs
float azimuth = 180.0f;                 // In degs

										// Mouse & keyboard
int mx_prev = 0, my_prev = 0;
bool mbuttons[3] = { false, false, false };
bool kmodifiers[3] = { false, false, false };

gmtl::Vec3f tangents[PT_NB];                          // Tangent at each input point
float s_tan = 1.0f;                                   // A scaling factor for tangents

gmtl::Vec3f tangents_2nd[SECOUND_PT_NB];                          // Tangent at each input point
float s_tan_2nd = 1.0f;                                   // A scaling factor for tangents

gmtl::Matrix44f Cmats[PT_NB];                         // Coefficient matrix (C) for each Hermite curve segment (It's actually 4x3, ignore last column)
gmtl::Matrix44f Cmats_2nd[SECOUND_PT_NB];

int cam_id = 0;

//First Plane
gmtl::Matrix44f ppose;                                // The plane's pose
gmtl::Matrix44f pposeadj;                             // Adjusted plane coordinate system that the (plane) camera will be attached to
gmtl::Matrix44f pposeadj_inv;                         // Adjusted plane coordinate system (plane's camera is attached to this frame), inverted 
int ps = 0;                                        // The segment in which the plane currently belongs
float pt = 0;                                        // The current t-value for the plane 
float pdt = 0.005f;                                    // delta_t for the plane

													   //Second Plane
gmtl::Matrix44f ppose_2nd;                                // The plane's pose
gmtl::Matrix44f pposeadj_2nd;                             // Adjusted plane coordinate system that the (plane) camera will be attached to
gmtl::Matrix44f pposeadj_inv_2nd;                         // Adjusted plane coordinate system (plane's camera is attached to this frame), inverted 
int ps_2nd = 0;                                        // The segment in which the plane currently belongs
float pt_2nd = 0;                                        // The current t-value for the plane 
float pdt_2nd = 0.005f;                                    // delta_t for the plane

MyParticle particles[PARTICLE_NB];    			          // Array of particles
MyParticle Fparticles[F_PARTICLE_NB];
MyParticle particles_2nd[PARTICLE_NB];    			          // Array of particles
MyParticle Fparticles_2nd[F_PARTICLE_NB];

GLuint texture;
GLuint texturesAddOn[TEXTURE_NB];

// Rendering option
bool render_curve = true;
bool render_constraint = false;

// Lighting
gmtl::Point4f light_pos(-20.0, 40.0, 0.0, 1.0);
bool is_diffuse_on = true;
bool is_ambient_on = true;
bool is_specular_on = true;

/*___________________
|
| Function Prototypes
|__________________*/

void Init();
void Init_AParticle(MyParticle &par, MyParticle &par2);
void Compute_Tangents();
void Compute_Coefficiences();
void Display_Func();
void Idle_Func();
void Update_Plane();
void Update_PSystem();
void Keyboard_Func(unsigned char key, int x, int y);
void Reshape_Func(int width, int height);
void Mouse_Func(int button, int state, int x, int y);
void Motion_Func(int x, int y);
void Draw_World_Axes();
void Draw_Path();
void Draw_Rocket();
void Draw_Particles();
float FastGauss(float mean, float std);
void LoadPPM(char *fname, unsigned int *w, unsigned int *h, unsigned char **data, int mallocflag);

void SetLight(const gmtl::Point4f &pos, const bool is_ambient, const bool is_diffuse, const bool is_specular);
void DrawPlaneBody();
void DrawMissileEngine();
void DrawCargoDoor();
void DrawLightBody();
void DrawLightObject();
void DrawSkybox(const float s);
void DrawTerrain();
void DrawFireParticles();
void UpdateSecondPlane();
void DrawWorldLight();


/*____________________________________________________________________
|
| Function: main
|
| Input:
| Output: Program entry point.
|___________________________________________________________________*/

int main(int argc, char **argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(WIN_WIDTH_INIT, WIN_HEIGHT_INIT);
	glutCreateWindow("Hermite Curve Drawing");

	glutDisplayFunc(Display_Func);
	glutIdleFunc(Idle_Func);
	glutKeyboardFunc(Keyboard_Func);
	glutReshapeFunc(Reshape_Func);
	glutMouseFunc(Mouse_Func);
	glutMotionFunc(Motion_Func);

	Init();

	glutMainLoop();

	return 0;
}

/*____________________________________________________________________
|
| Function: Init
|
| Input:
| Output: Initialization routine.
|___________________________________________________________________*/

void Init()
{
	int i;
	unsigned int texwidth, texheight;
	unsigned char *imagedata;

	// OpenGL
	glClearColor(0, 0, 0, 0);
	glEnable(GL_DEPTH_TEST);
	glShadeModel(GL_SMOOTH);

	// Inits Hermite basis matrix
	MMAT.set(
		2, -2, 1, 1,
		-3, 3, -2, -1,
		0, 0, 1, 0,
		1, 0, 0, 0
		);

	// Inits tangents and coefficient matrices
	Compute_Tangents();
	Compute_Coefficiences();

	// Init adjusted plane-coordinate-system that the camera is attached to (HACK! by calling Update_Plane here)
	Update_Plane();
	UpdateSecondPlane();

	// Inits particle system
	for (i = 0; i<PARTICLE_NB; i++) {
		Init_AParticle(particles[i], particles_2nd[i]);
	}

	for (i = 0; i < F_PARTICLE_NB; i++) {
		Init_AParticle(Fparticles[i], Fparticles_2nd[i]);
	}

	//|___________________________________________________________________
	//|
	//| Setup lighting
	//|___________________________________________________________________

	// Disable global ambient
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, NO_LIGHT);

	// NOTE: for specular reflections, the "local viewer" model produces better
	// results than the default, but is slower. The default would not use the correct
	// vertex-to-eyepoint vector, treating it as always parallel to Z.
	glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);

	// Enable two sided lighting
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);

	// Enable lighting
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	/*____________________________________________________________________
	|
	| Load texture
	|___________________________________________________________________*/

	// describe how data will be stored in memory
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

	// generate a new "texture object" and select it for setup
	glGenTextures(1, &texture);
	glBindTexture(GL_TEXTURE_2D, texture);

	// load an image into memory
	LoadPPM("smoketex.ppm", &texwidth, &texheight, &imagedata, 1);

	// describe the image to the graphics system as a texture map
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, texwidth, texheight, 0,
		GL_RGB, GL_UNSIGNED_BYTE, imagedata);
	free(imagedata);

	// select methods for "scaling" a texture region to a pixel
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

	// select the method for combining texture color with the lighting equation
	// (look up the third parameter)
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

	// NOTE: to have another texture map, generate another texture object and
	//       repeat the setup steps. To select which texture is being applied 
	//       during drawing, use glBindTexture() to select.

	// Describe how data will be stored in memory
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

	// Select the method for combining texture color with the lighting equation
	// (look up the third parameter)
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

	// Generate and setup texture objects
	glGenTextures(TEXTURE_NB, texturesAddOn);

	// Skybox back wall
	glBindTexture(GL_TEXTURE_2D, texturesAddOn[TID_SKYBACK]);
	LoadPPM("skybox_back.ppm", &texwidth, &texheight, &imagedata, 1);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, texwidth, texheight, 0, GL_RGB, GL_UNSIGNED_BYTE, imagedata);
	free(imagedata);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

	// Skybox left wall
	glBindTexture(GL_TEXTURE_2D, texturesAddOn[TID_SKYLEFT]);
	LoadPPM("skybox_left.ppm", &texwidth, &texheight, &imagedata, 1);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, texwidth, texheight, 0, GL_RGB, GL_UNSIGNED_BYTE, imagedata);
	free(imagedata);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

	// Skybox bottom wall
	glBindTexture(GL_TEXTURE_2D, texturesAddOn[TID_SKYBOTTOM]);
	LoadPPM("skybox_bottom.ppm", &texwidth, &texheight, &imagedata, 1);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, texwidth, texheight, 0, GL_RGB, GL_UNSIGNED_BYTE, imagedata);
	free(imagedata);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

	// Skybox front wall
	glBindTexture(GL_TEXTURE_2D, texturesAddOn[TID_SKYFRONT]);
	LoadPPM("skybox_front.ppm", &texwidth, &texheight, &imagedata, 1);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, texwidth, texheight, 0, GL_RGB, GL_UNSIGNED_BYTE, imagedata);
	free(imagedata);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

	// Skybox right wall
	glBindTexture(GL_TEXTURE_2D, texturesAddOn[TID_SKYRIGHT]);
	LoadPPM("skybox_right.ppm", &texwidth, &texheight, &imagedata, 1);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, texwidth, texheight, 0, GL_RGB, GL_UNSIGNED_BYTE, imagedata);
	free(imagedata);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

	// Skybox top wall
	glBindTexture(GL_TEXTURE_2D, texturesAddOn[TID_SKYTOP]);
	LoadPPM("skybox_top.ppm", &texwidth, &texheight, &imagedata, 1);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, texwidth, texheight, 0, GL_RGB, GL_UNSIGNED_BYTE, imagedata);
	free(imagedata);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

	// Terrain 1
	glBindTexture(GL_TEXTURE_2D, texturesAddOn[TID_TERRAIN_1]);
	LoadPPM("Terrain1.ppm", &texwidth, &texheight, &imagedata, 1);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, texwidth, texheight, 0, GL_RGB, GL_UNSIGNED_BYTE, imagedata);
	free(imagedata);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

	// Terrain 2
	glBindTexture(GL_TEXTURE_2D, texturesAddOn[TID_TERRAIN_2]);
	LoadPPM("Terrain2.ppm", &texwidth, &texheight, &imagedata, 1);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, texwidth, texheight, 0, GL_RGB, GL_UNSIGNED_BYTE, imagedata);
	free(imagedata);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

	// Shore 
	glBindTexture(GL_TEXTURE_2D, texturesAddOn[TID_SHORE]);
	LoadPPM("Shore.ppm", &texwidth, &texheight, &imagedata, 1);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, texwidth, texheight, 0, GL_RGB, GL_UNSIGNED_BYTE, imagedata);
	free(imagedata);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

	// Fire 
	glBindTexture(GL_TEXTURE_2D, texturesAddOn[TID_FIRE]);
	LoadPPM("Fire.ppm", &texwidth, &texheight, &imagedata, 1);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, texwidth, texheight, 0, GL_RGB, GL_UNSIGNED_BYTE, imagedata);
	free(imagedata);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
}

/*____________________________________________________________________
|
| Function: Init_AParticle
|
| Input:
| Output: Init a single particle.
|___________________________________________________________________*/

void Init_AParticle(MyParticle &par, MyParticle &par2)
{
	gmtl::Vec3f v_dir;
	gmtl::Vec3f v_dir2;

	// position (= plane position)
	par.p.set(ppose[0][3] - (20 * ppose[0][2]), ppose[1][3] - (20 * ppose[1][2]), ppose[2][3] - (20 * ppose[2][2]));
	par2.p.set(ppose_2nd[0][3] - (20 * ppose_2nd[0][2]), ppose_2nd[1][3] - (20 * ppose_2nd[1][2]), ppose_2nd[2][3] - (20 * ppose_2nd[2][2]));

	// velocity (consider plane's -Z axis as mean velocity direction)
	v_dir.set(-ppose[0][2], -ppose[1][2], -ppose[2][2]);
	v_dir2.set(-ppose_2nd[0][2], -ppose_2nd[1][2], -ppose_2nd[2][2]);

	par.v.set(
		FastGauss(v_dir[0], VDIR_STD),
		FastGauss(v_dir[1], VDIR_STD),
		FastGauss(v_dir[2], VDIR_STD)
		);
	par2.v.set(
		FastGauss(v_dir2[0], VDIR_STD),
		FastGauss(v_dir2[1], VDIR_STD),
		FastGauss(v_dir2[2], VDIR_STD)
		);
	par.v = FastGauss(VMAG_MEAN, VMAG_STD) * par.v;
	par2.v = FastGauss(VMAG_MEAN, VMAG_STD) * par2.v;

	// mass
	par.m = 1;
	par2.m = 1;

	// ttl
	par.ttl = par.ttl_init = (rand() % TTL_BASE) + TTL_OFFSET;
	par2.ttl = par2.ttl_init = (rand() % TTL_BASE) + TTL_OFFSET;
}

/*____________________________________________________________________
|
| Function: Compute_Tangents
|
| Input:
| Output: Computes tangents from input points and the scale factor
|___________________________________________________________________*/

void Compute_Tangents()
{
	int i;
	int prev, next;       // Adjacent points

						  //First Plane
	for (i = 0; i<PT_NB; i++) {
		// Compute adjacent points
		if (i == 0) { // First point
			prev = PT_NB - 1;
			next = i + 1;
		}
		else
			if (i == PT_NB - 1) { // Last point
				prev = i - 1;
				next = 0;
			}
			else { // Interior point
				prev = i - 1;
				next = i + 1;
			}

			tangents[i] = s_tan * (input_pts[next] - input_pts[prev]);
	}

	//Second Plane
	for (i = 0; i < SECOUND_PT_NB; i++) {
		// Compute adjacent points
		if (i == 0) { // First point
			prev = SECOUND_PT_NB - 1;
			next = i + 1;
		}
		else
			if (i == SECOUND_PT_NB - 1) { // Last point
				prev = i - 1;
				next = 0;
			}
			else { // Interior point
				prev = i - 1;
				next = i + 1;
			}

			tangents_2nd[i] = s_tan_2nd * (second_input_pts[next] - second_input_pts[prev]);
	}
}

/*____________________________________________________________________
|
| Function: Compute_Coefficiences
|
| Input:
| Output: Computes coefficiences matrices for curve segments
|___________________________________________________________________*/

void Compute_Coefficiences()
{
	int i;
	int n;                               // Next input point (index)
	gmtl::Matrix44f Gmat;                // Geometry matrix (It's actually 4x3, ignore last column)

										 //First Plane
	for (i = 0; i<PT_NB; i++) {
		if (i == PT_NB - 1) {
			n = 0;
		}
		else {
			n = i + 1;
		}

		Gmat.set(
			input_pts[i][0], input_pts[i][1], input_pts[i][2], 0,
			input_pts[n][0], input_pts[n][1], input_pts[n][2], 0,
			tangents[i][0], tangents[i][1], tangents[i][2], 0,
			tangents[n][0], tangents[n][1], tangents[n][2], 0
			);

		Cmats[i] = MMAT * Gmat;
	}

	//Second Plane
	for (i = 0; i<SECOUND_PT_NB; i++) {
		if (i == SECOUND_PT_NB - 1) {
			n = 0;
		}
		else {
			n = i + 1;
		}

		Gmat.set(
			second_input_pts[i][0], second_input_pts[i][1], second_input_pts[i][2], 0,
			second_input_pts[n][0], second_input_pts[n][1], second_input_pts[n][2], 0,
			tangents_2nd[i][0], tangents_2nd[i][1], tangents_2nd[i][2], 0,
			tangents_2nd[n][0], tangents_2nd[n][1], tangents_2nd[n][2], 0
			);

		Cmats_2nd[i] = MMAT * Gmat;
	}
}

/*____________________________________________________________________
|
| Function: Display_Func
|
| Input:
| Output: GLUT display callback function.
|___________________________________________________________________*/

void Display_Func(void)
{
	// Clear screen & depth buffer
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	switch (cam_id) {
		// Add view matrix (camera)
	case 0:
		glTranslatef(0, 0, -distance);
		glRotatef(-elevation, 1, 0, 0);
		glRotatef(-azimuth, 0, 1, 0);
		glMultMatrixf(pposeadj_inv.mData);
		break;

	case 1:
		glTranslatef(0, 0, -distance);
		glRotatef(-elevation, 1, 0, 0);
		glRotatef(-azimuth, 0, 1, 0);
		glMultMatrixf(pposeadj_inv_2nd.mData);
	}

	SetLight(light_pos, is_ambient_on, is_diffuse_on, is_specular_on);
	glPushMatrix();
	glTranslatef(light_pos[0], light_pos[1], light_pos[2]);
	DrawWorldLight();
	glPopMatrix();

	Draw_World_Axes();
	Draw_Path();
	DrawSkybox(SB_SIZE);			//Sky box

	glPushMatrix();
	glTranslatef(-(SB_SIZE / 2), -(SB_SIZE / 2) + 1, -(SB_SIZE / 2));
	DrawTerrain();
	glPopMatrix();

	glPushMatrix();
	glMultMatrixf(ppose.mData);
	DrawPlaneBody();
	DrawMissileEngine();
	DrawCargoDoor();
	DrawLightBody();
	DrawLightObject();
	glPopMatrix();

	glPushMatrix();
	glMultMatrixf(ppose_2nd.mData);
	DrawPlaneBody();
	glPopMatrix();

	Draw_Particles();
	DrawFireParticles();
	// Display!
	glutSwapBuffers();
}

/*____________________________________________________________________
|
| Function: Idle_Func
|
| Input:
| Output: GLUT idle callback function. Called for each simulation step.
|___________________________________________________________________*/

void Idle_Func()
{
	Update_Plane();
	Update_PSystem();
	UpdateSecondPlane();

	glutPostRedisplay();
}

/*____________________________________________________________________
|
| Function: Update_Plane
|
| Input:
| Output: Called every simulation step to update the plane's pose.
|___________________________________________________________________*/

void Update_Plane()
{
	float x, y, z;                    // Plane's coordinates
	gmtl::Vec3f xa, ya, za;           // Plane's axes
	gmtl::Vec3f ac;                   // Acceleration vector
	float r;                          // Local rightward component of acceleration
	float ra;                         // Roll angle
	gmtl::AxisAnglef roll_aa;         // Local roll in axis angle and matrix forms
	gmtl::Matrix44f roll_mat;
	gmtl::Vec3f zadj;

	/*____________________________________________________________________
	|
	| Update t and possibly current segment
	|___________________________________________________________________*/

	pt += pdt;
	if (pt > 1) {
		pt -= 1;
		ps++;
		if (ps == PT_NB) {
			ps = 0;
		}
	}

	/*____________________________________________________________________
	|
	| Compute plane's new position by evaluating the polynomials (use Horner's rule for slight speedup)
	|___________________________________________________________________*/

	x = ((Cmats[ps][0][0] * pt + Cmats[ps][1][0])*pt + Cmats[ps][2][0])*pt + Cmats[ps][3][0];
	y = ((Cmats[ps][0][1] * pt + Cmats[ps][1][1])*pt + Cmats[ps][2][1])*pt + Cmats[ps][3][1];
	z = ((Cmats[ps][0][2] * pt + Cmats[ps][1][2])*pt + Cmats[ps][2][2])*pt + Cmats[ps][3][2];

	/*____________________________________________________________________
	|
	| Compute plane's orientation
	|___________________________________________________________________*/

	// Compute direction of motion (z) by evaluating the polynomial derivatives (use Horner's rule for slight speedup)
	za[0] = (3 * Cmats[ps][0][0] * pt + 2 * Cmats[ps][1][0])*pt + Cmats[ps][2][0];
	za[1] = (3 * Cmats[ps][0][1] * pt + 2 * Cmats[ps][1][1])*pt + Cmats[ps][2][1];
	za[2] = (3 * Cmats[ps][0][2] * pt + 2 * Cmats[ps][1][2])*pt + Cmats[ps][2][2];
	gmtl::normalize(za);

	// Compute lateral axis (x)
	gmtl::cross(xa, WORLD_UP, za);
	gmtl::normalize(xa);

	// Compute up axis (x)
	gmtl::cross(ya, za, xa);
	gmtl::normalize(ya);

	/*____________________________________________________________________
	|
	| Compute banked turn (local roll)
	|___________________________________________________________________*/

	// Compute acceleration vector
	ac[0] = 6 * Cmats[ps][0][0] * pt + 2 * Cmats[ps][1][0];
	ac[1] = 6 * Cmats[ps][0][1] * pt + 2 * Cmats[ps][1][1];
	ac[2] = 6 * Cmats[ps][0][2] * pt + 2 * Cmats[ps][1][2];

	// Compute local rightward component of acceleration
	r = gmtl::dot(ac, xa);

	// Clamp r to R_MAX
	if (r > R_MAX) {
		r = R_MAX;
	}
	else
		if (r < -R_MAX) {
			r = -R_MAX;
		}

	// Compute roll angle
	ra = asin(r / R_MAX);

	//printf("%.2f\n", r);

	/*____________________________________________________________________
	|
	| Update plane's pose
	|___________________________________________________________________*/

	ppose.set(
		xa[0], ya[0], za[0], x,
		xa[1], ya[1], za[1], y,
		xa[2], ya[2], za[2], z,
		0, 0, 0, 1
		);
	ppose.setState(gmtl::Matrix44f::AFFINE);

	// Compute local roll (rotation about longitudinal axis (z))
	roll_aa.set(ra, za[0], za[1], za[2]);
	gmtl::set(roll_mat, roll_aa);

	// Apply local roll
	ppose = ppose * roll_mat;

	/*____________________________________________________________________
	|
	| Compute adjusted plane coordinate system and its inverse
	|___________________________________________________________________*/

	gmtl::cross(zadj, xa, WORLD_UP);
	gmtl::normalize(zadj);

	pposeadj.set(
		xa[0], WORLD_UP[0], zadj[0], x,
		xa[1], WORLD_UP[1], zadj[1], y,
		xa[2], WORLD_UP[2], zadj[2], z,
		0, 0, 0, 1
		);
	pposeadj.setState(gmtl::Matrix44f::AFFINE);

	gmtl::invert(pposeadj_inv, pposeadj);

}

void UpdateSecondPlane()
{
	float x, y, z;                    // Plane's coordinates
	gmtl::Vec3f xa, ya, za;           // Plane's axes
	gmtl::Vec3f ac;                   // Acceleration vector
	float r;                          // Local rightward component of acceleration
	float ra;                         // Roll angle
	gmtl::AxisAnglef roll_aa;         // Local roll in axis angle and matrix forms
	gmtl::Matrix44f roll_mat;
	gmtl::Vec3f zadj;

	/*____________________________________________________________________
	|
	| Update t and possibly current segment
	|___________________________________________________________________*/

	pt_2nd += pdt_2nd;
	if (pt_2nd > 1) {
		pt_2nd -= 1;
		ps_2nd++;
		if (ps_2nd == SECOUND_PT_NB) {
			ps_2nd = 0;
		}
	}

	/*____________________________________________________________________
	|
	| Compute plane's new position by evaluating the polynomials (use Horner's rule for slight speedup)
	|___________________________________________________________________*/

	x = ((Cmats_2nd[ps_2nd][0][0] * pt_2nd + Cmats_2nd[ps_2nd][1][0])*pt_2nd + Cmats_2nd[ps_2nd][2][0])*pt_2nd + Cmats_2nd[ps_2nd][3][0];
	y = ((Cmats_2nd[ps_2nd][0][1] * pt_2nd + Cmats_2nd[ps_2nd][1][1])*pt_2nd + Cmats_2nd[ps_2nd][2][1])*pt_2nd + Cmats_2nd[ps_2nd][3][1];
	z = ((Cmats_2nd[ps_2nd][0][2] * pt_2nd + Cmats_2nd[ps_2nd][1][2])*pt_2nd + Cmats_2nd[ps_2nd][2][2])*pt_2nd + Cmats_2nd[ps_2nd][3][2];

	/*____________________________________________________________________
	|
	| Compute plane's orientation
	|___________________________________________________________________*/

	// Compute direction of motion (z) by evaluating the polynomial derivatives (use Horner's rule for slight speedup)
	za[0] = (3 * Cmats_2nd[ps_2nd][0][0] * pt_2nd + 2 * Cmats_2nd[ps_2nd][1][0])*pt_2nd + Cmats_2nd[ps_2nd][2][0];
	za[1] = (3 * Cmats_2nd[ps_2nd][0][1] * pt_2nd + 2 * Cmats_2nd[ps_2nd][1][1])*pt_2nd + Cmats_2nd[ps_2nd][2][1];
	za[2] = (3 * Cmats_2nd[ps_2nd][0][2] * pt_2nd + 2 * Cmats_2nd[ps_2nd][1][2])*pt_2nd + Cmats_2nd[ps_2nd][2][2];
	gmtl::normalize(za);

	// Compute lateral axis (x)
	gmtl::cross(xa, WORLD_UP, za);
	gmtl::normalize(xa);

	// Compute up axis (x)
	gmtl::cross(ya, za, xa);
	gmtl::normalize(ya);

	/*____________________________________________________________________
	|
	| Compute banked turn (local roll)
	|___________________________________________________________________*/

	// Compute acceleration vector
	ac[0] = 6 * Cmats_2nd[ps_2nd][0][0] * pt_2nd + 2 * Cmats_2nd[ps_2nd][1][0];
	ac[1] = 6 * Cmats_2nd[ps_2nd][0][1] * pt_2nd + 2 * Cmats_2nd[ps_2nd][1][1];
	ac[2] = 6 * Cmats_2nd[ps_2nd][0][2] * pt_2nd + 2 * Cmats_2nd[ps_2nd][1][2];

	// Compute local rightward component of acceleration
	r = gmtl::dot(ac, xa);

	// Clamp r to R_MAX
	if (r > R_MAX) {
		r = R_MAX;
	}
	else
		if (r < -R_MAX) {
			r = -R_MAX;
		}

	// Compute roll angle
	ra = asin(r / R_MAX);

	//printf("%.2f\n", r);

	/*____________________________________________________________________
	|
	| Update plane's pose
	|___________________________________________________________________*/

	ppose_2nd.set(
		xa[0], ya[0], za[0], x,
		xa[1], ya[1], za[1], y,
		xa[2], ya[2], za[2], z,
		0, 0, 0, 1
		);
	ppose_2nd.setState(gmtl::Matrix44f::AFFINE);

	// Compute local roll (rotation about longitudinal axis (z))
	roll_aa.set(ra, za[0], za[1], za[2]);
	gmtl::set(roll_mat, roll_aa);

	// Apply local roll
	ppose_2nd = ppose_2nd * roll_mat;

	/*____________________________________________________________________
	|
	| Compute adjusted plane coordinate system and its inverse
	|___________________________________________________________________*/

	gmtl::cross(zadj, xa, WORLD_UP);
	gmtl::normalize(zadj);

	pposeadj_2nd.set(
		xa[0], WORLD_UP[0], zadj[0], x,
		xa[1], WORLD_UP[1], zadj[1], y,
		xa[2], WORLD_UP[2], zadj[2], z,
		0, 0, 0, 1
		);
	pposeadj_2nd.setState(gmtl::Matrix44f::AFFINE);

	gmtl::invert(pposeadj_inv_2nd, pposeadj_2nd);

}

/*____________________________________________________________________
|
| Function: Update_PSystem
|
| Input:
| Output: Called every simulation step to update the particle system.
|___________________________________________________________________*/

void Update_PSystem()
{
	int i;
	gmtl::Vec3f F;             // Net force
	gmtl::Vec3f a;             // Acceleration

							   //Smoke Update
	for (i = 0; i<PARTICLE_NB; i++) {
		// Life is shorten by one time unit
		particles[i].ttl--;

		if (particles[i].ttl > 0) { // Still active
									// Update position
			particles[i].p += S_TSTEP * particles[i].v;

			// Compute net force from gravity and vicous drag (from wind)
			F = (particles[i].m * GRAVITY) - (K_COEF*(particles[i].v - V_WIND));

			// Calculate acceleration from froce
			a = F / particles[i].m;

			// Update velocity
			particles[i].v += S_TSTEP*a;
		}
		else {  // Died, make it reborn
			Init_AParticle(particles[i], particles_2nd[i]);
		}
	}

	//Fire Update
	for (i = 0; i<F_PARTICLE_NB; i++) {
		// Life is shorten by one time unit
		Fparticles[i].ttl--;

		if (Fparticles[i].ttl > 0) { // Still active
									 // Update position
			Fparticles[i].p += S_TSTEP * Fparticles[i].v;

			// Compute net force from gravity and vicous drag (from wind)
			F = (Fparticles[i].m * FGRAVITY) - (K_COEF*(Fparticles[i].v - V_WIND));

			// Calculate acceleration from froce
			a = F / Fparticles[i].m;

			// Update velocity
			Fparticles[i].v += S_TSTEP*a;
		}
		else {  // Died, make it reborn
			Init_AParticle(Fparticles[i], Fparticles_2nd[i]);
		}
	}
}

/*____________________________________________________________________
|
| Function: Keyboard_Func
|
| Input:
| Output: GLUT keyboard callback function.
|___________________________________________________________________*/

void Keyboard_Func(unsigned char key, int x, int y)
{
	switch ((char)key) {
	case 'c':       // Toggle curve drawing
		render_curve = !render_curve;
		break;
	case 'x':       // Toggle constraint drawing
		render_constraint = !render_constraint;
		break;
	case 'v':		// Select observed plane (camera)
		cam_id = (cam_id + 1) % 2;
		break;

		//|____________________________________________________________________
		//|
		//| Lighting controls
		//|____________________________________________________________________

	case 'o': // Light up (+Y translation)
		light_pos[1]++;
		printf("Light-Y = %.2f\n", light_pos[1]);
		break;
	case 'l': // Light down (-Y translation)
		light_pos[1]--;
		printf("Light-Y = %.2f\n", light_pos[1]);
		break;
	case 'p': // Light up (+X translation)
		light_pos[0]++;
		printf("Light-X = %.2f\n", light_pos[0]);
		break;
	case ';': // Light down (+X translation)
		light_pos[0]--;
		printf("Light-X = %.2f\n", light_pos[0]);
		break;
	case '[': // Light up (+Z translation)
		light_pos[2]++;
		printf("Light-Z = %.2f\n", light_pos[2]);
		break;
	case ']': // Light down (+X translation)
		light_pos[2]--;
		printf("Light-Z = %.2f\n", light_pos[2]);
		break;

	case '9': // Toggles diffuse light ON/OFF
		is_diffuse_on = !is_diffuse_on;
		printf("Light-diffuse = %s\n", is_diffuse_on ? "ON" : "OFF");
		break;

	case '8': // Toggles diffuse light ON/OFF
		is_ambient_on = !is_ambient_on;
		printf("Light-diffuse = %s\n", is_ambient_on ? "ON" : "OFF");
		break;

	case '0': // Toggles diffuse light ON/OFF
		is_specular_on = !is_specular_on;
		printf("Light-diffuse = %s\n", is_specular_on ? "ON" : "OFF");
		break;

	default:
		break;
	}

	glutPostRedisplay();
}

/*____________________________________________________________________
|
| Function: Reshape_Func
|
| Input:
| Output: GLUT reshape callback function.
|___________________________________________________________________*/

void Reshape_Func(int width, int height)
{
	glViewport(0, 0, width, height);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0f, ((float)width) / height, 1.0f, 10000.0f);
}

//|____________________________________________________________________
//|
//| Function: Mouse_Func
//|
//! \param button     [in] one of GLUT_LEFT_BUTTON, GLUT_MIDDLE_BUTTON, or GLUT_RIGHT_BUTTON.
//! \param state      [in] one of GLUT_UP (event is due to release) or GLUT_DOWN (press).
//! \param x          [in] X-coordinate of mouse when an event occured.
//! \param y          [in] Y-coordinate of mouse when an event occured.
//! \return None.
//!
//! GLUT mouse-callback function: called for each mouse click.
//|____________________________________________________________________

void Mouse_Func(int button, int state, int x, int y)
{
	int km_state;

	// Updates button's sate and mouse coordinates
	if (state == GLUT_DOWN) {
		mbuttons[button] = true;
		mx_prev = x;
		my_prev = y;
	}
	else {
		mbuttons[button] = false;
	}

	// Updates keyboard modifiers
	km_state = glutGetModifiers();
	kmodifiers[KM_SHIFT] = km_state & GLUT_ACTIVE_SHIFT ? true : false;
	kmodifiers[KM_CTRL] = km_state & GLUT_ACTIVE_CTRL ? true : false;
	kmodifiers[KM_ALT] = km_state & GLUT_ACTIVE_ALT ? true : false;

	//glutPostRedisplay();      // Asks GLUT to redraw the screen
}

//|____________________________________________________________________
//|
//| Function: Motion_Func
//|
//! \param x      [in] X-coordinate of mouse when an event occured.
//! \param y      [in] Y-coordinate of mouse when an event occured.
//! \return None.
//!
//! GLUT motion-callback function: called for each mouse motion.
//|____________________________________________________________________

void Motion_Func(int x, int y)
{
	int dx, dy, d;

	if (mbuttons[GLUT_LEFT_BUTTON] || mbuttons[GLUT_RIGHT_BUTTON]) {
		// Computes distances the mouse has moved
		dx = x - mx_prev;
		dy = y - my_prev;

		// Updates mouse coordinates
		mx_prev = x;
		my_prev = y;

		// Hold left button to rotate camera
		if (mbuttons[GLUT_LEFT_BUTTON]) {
			if (!kmodifiers[KM_CTRL]) {
				elevation += dy;            // Elevation update
			}
			if (!kmodifiers[KM_SHIFT]) {
				azimuth += dx;              // Azimuth update
			}
		}

		// Hold right button to zoom
		if (mbuttons[GLUT_RIGHT_BUTTON]) {
			if (abs(dx) >= abs(dy)) {
				d = dx;
			}
			else {
				d = -dy;
			}
			distance += d;
		}

		glutPostRedisplay();      // Asks GLUT to redraw the screen
	}
}

/*____________________________________________________________________
|
| Function: Draw_World_Axes
|
| Input:
| Output: Draw world axes.
|___________________________________________________________________*/

void Draw_World_Axes()
{
	glDisable(GL_LIGHTING);
	glBegin(GL_LINES);
	// x-axis is red
	glColor3f(1.0, 0.0, 0.0);
	glVertex3f(0.0, 0.0, 0.0);
	glVertex3f(100.0, 0.0, 0.0);
	// y-axis is green
	glColor3f(0.0, 1.0, 0.0);
	glVertex3f(0.0, 0.0, 0.0);
	glVertex3f(0.0, 100.0, 0.0);
	// z-axis is blue
	glColor3f(0.0, 0.0, 1.0);
	glVertex3f(0.0, 0.0, 0.0);
	glVertex3f(0.0, 0.0, 100.0);
	glEnd();
	glEnable(GL_LIGHTING);
}

/*____________________________________________________________________
|
| Function: Draw_Path
|
| Input:
| Output: Draws a path.
|___________________________________________________________________*/

void Draw_Path()
{
	int i;
	float x, y, z, t;

	/*____________________________________________________________________
	|
	| Draw input points and tangents
	|___________________________________________________________________*/
	glDisable(GL_LIGHTING);
	if (render_constraint) {
		for (i = 0; i<PT_NB; i++) {
			glPushMatrix();
			glTranslatef(input_pts[i][0], input_pts[i][1], input_pts[i][2]);
			glColor3f(1, 0, 0);
			glutSolidSphere(5, 16, 16);
			glColor3f(1, 1, 0);
			glBegin(GL_LINES);
			glVertex3f(0, 0, 0);
			glVertex3f(tangents[i][0], tangents[i][1], tangents[i][2]);
			glEnd();
			glPopMatrix();
		}
	}

	/*____________________________________________________________________
	|
	| Draw Hermite curve segments using line strips
	|___________________________________________________________________*/

	if (render_curve) {
		glColor3f(0, 1, 0);
		for (i = 0; i<PT_NB; i++) {
			// Draw each segment
			glBegin(GL_LINE_STRIP);
			for (t = 0; t <= 1; t += C_TSTEP) {
				// Simple polynomial evaluation
				//float t2 = t*t;
				//float t3 = t2*t;
				//x = Cmats[i][0][0]*t3 + Cmats[i][1][0]*t2 + Cmats[i][2][0]*t + Cmats[i][3][0];
				//y = Cmats[i][0][1]*t3 + Cmats[i][1][1]*t2 + Cmats[i][2][1]*t + Cmats[i][3][1];
				//z = Cmats[i][0][2]*t3 + Cmats[i][1][2]*t2 + Cmats[i][2][2]*t + Cmats[i][3][2];

				// Use Horner's rule for slight speedup
				x = ((Cmats[i][0][0] * t + Cmats[i][1][0])*t + Cmats[i][2][0])*t + Cmats[i][3][0];
				y = ((Cmats[i][0][1] * t + Cmats[i][1][1])*t + Cmats[i][2][1])*t + Cmats[i][3][1];
				z = ((Cmats[i][0][2] * t + Cmats[i][1][2])*t + Cmats[i][2][2])*t + Cmats[i][3][2];
				glVertex3f(x, y, z);
			}
			glEnd();
		}
	}
	glEnable(GL_LIGHTING);
}

/*____________________________________________________________________
|
| Function: Draw_Particles
|
| Input:
| Output: Draw particles as view-oriented billboards.
|___________________________________________________________________*/

void Draw_Particles()
{
	int i;
	gmtl::Matrix44f cam_mat;                    // Camera matrix
	gmtl::Matrix44f dt_mat, el_mat, az_mat;     // distance, elevation, and azimuth matrices, initialized to IDENTITY
	float age_scale;                            // Age factor

												/*____________________________________________________________________
												|
												| Enable texturing and blending
												|___________________________________________________________________*/

	glDisable(GL_LIGHTING);
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, texture);
	glDepthMask(GL_FALSE);
	glEnable(GL_BLEND);
	glBlendFunc(GL_ONE, GL_ONE);

	/*____________________________________________________________________
	|
	| Orient billboards to face the camera
	|___________________________________________________________________*/

	// Set distance matrix
	dt_mat[2][3] = distance;

	// Set elevation matrix
	gmtl::set(el_mat,
		gmtl::AxisAnglef(gmtl::Math::deg2Rad(elevation), 1, 0, 0)
		);

	// Set azimuth matrix
	gmtl::set(az_mat,
		gmtl::AxisAnglef(gmtl::Math::deg2Rad(azimuth), 0, 1, 0)
		);

	// Compute camera w.r.t. world and discard translation
	cam_mat = pposeadj * az_mat * el_mat * dt_mat;
	cam_mat[0][3] = cam_mat[1][3] = cam_mat[2][3] = 0;

	/*____________________________________________________________________
	|
	| Render particles as billboards
	|___________________________________________________________________*/

	for (i = 0; i<PARTICLE_NB; i++) {
		glPushMatrix();
		glTranslatef(particles[i].p[0], particles[i].p[1], particles[i].p[2]);
		glMultMatrixf(cam_mat.mData);       // Orient billboards to face camera

		glBegin(GL_QUADS);
		age_scale = ((float)particles[i].ttl) / particles[i].ttl_init;
		glColor3f(age_scale, age_scale, age_scale);
		glTexCoord2f(0.0, 0.0);
		glVertex3f(-SMOKE_SIZE, -SMOKE_SIZE, 0.0);
		glTexCoord2f(1.0, 0.0);
		glVertex3f(SMOKE_SIZE, -SMOKE_SIZE, 0.0);
		glTexCoord2f(1.0, 1.0);
		glVertex3f(SMOKE_SIZE, SMOKE_SIZE, 0.0);
		glTexCoord2f(0.0, 1.0);
		glVertex3f(-SMOKE_SIZE, SMOKE_SIZE, 0.0);
		glEnd();
		glPopMatrix();

		glPushMatrix();
		glTranslatef(particles_2nd[i].p[0], particles_2nd[i].p[1], particles_2nd[i].p[2]);
		glMultMatrixf(cam_mat.mData);       // Orient billboards to face camera

		glBegin(GL_QUADS);
		age_scale = ((float)particles_2nd[i].ttl) / particles_2nd[i].ttl_init;
		glColor3f(age_scale, age_scale, age_scale);
		glTexCoord2f(0.0, 0.0);
		glVertex3f(-SMOKE_SIZE, -SMOKE_SIZE, 0.0);
		glTexCoord2f(1.0, 0.0);
		glVertex3f(SMOKE_SIZE, -SMOKE_SIZE, 0.0);
		glTexCoord2f(1.0, 1.0);
		glVertex3f(SMOKE_SIZE, SMOKE_SIZE, 0.0);
		glTexCoord2f(0.0, 1.0);
		glVertex3f(-SMOKE_SIZE, SMOKE_SIZE, 0.0);
		glEnd();
		glPopMatrix();
	}

	/*____________________________________________________________________
	|
	| Restore rendering states
	|___________________________________________________________________*/

	glDisable(GL_BLEND);
	glDepthMask(GL_TRUE);
	glDisable(GL_TEXTURE_2D);
	glEnable(GL_LIGHTING);
}

void DrawFireParticles() {
	int i;
	gmtl::Matrix44f cam_mat;                    // Camera matrix
	gmtl::Matrix44f dt_mat, el_mat, az_mat;     // distance, elevation, and azimuth matrices, initialized to IDENTITY
	float age_scale;                            // Age factor

												/*____________________________________________________________________
												|
												| Enable texturing and blending
												|___________________________________________________________________*/

	glDisable(GL_LIGHTING);
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, texturesAddOn[TID_FIRE]);
	glDepthMask(GL_FALSE);
	glEnable(GL_BLEND);
	glBlendFunc(GL_ONE, GL_ONE);

	/*____________________________________________________________________
	|
	| Orient billboards to face the camera
	|___________________________________________________________________*/

	// Set distance matrix
	dt_mat[2][3] = distance;

	// Set elevation matrix
	gmtl::set(el_mat,
		gmtl::AxisAnglef(gmtl::Math::deg2Rad(elevation), 1, 0, 0)
		);

	// Set azimuth matrix
	gmtl::set(az_mat,
		gmtl::AxisAnglef(gmtl::Math::deg2Rad(azimuth), 0, 1, 0)
		);

	// Compute camera w.r.t. world and discard translation
	cam_mat = pposeadj * az_mat * el_mat * dt_mat;
	cam_mat[0][3] = cam_mat[1][3] = cam_mat[2][3] = 0;

	/*____________________________________________________________________
	|
	| Render particles as billboards
	|___________________________________________________________________*/

	for (i = 0; i<F_PARTICLE_NB; i++) {
		glPushMatrix();
		glTranslatef(Fparticles[i].p[0], Fparticles[i].p[1], Fparticles[i].p[2]);
		glMultMatrixf(cam_mat.mData);       // Orient billboards to face camera

		glBegin(GL_QUADS);
		age_scale = ((float)Fparticles[i].ttl) / Fparticles[i].ttl_init;
		glColor3f(age_scale, age_scale, age_scale);
		glTexCoord2f(0.0, 0.0);
		glVertex3f(-FIRE_SIZE, -FIRE_SIZE, 0.0);
		glTexCoord2f(1.0, 0.0);
		glVertex3f(FIRE_SIZE, -FIRE_SIZE, 0.0);
		glTexCoord2f(1.0, 1.0);
		glVertex3f(FIRE_SIZE, FIRE_SIZE, 0.0);
		glTexCoord2f(0.0, 1.0);
		glVertex3f(-FIRE_SIZE, FIRE_SIZE, 0.0);
		glEnd();
		glPopMatrix();

		glPushMatrix();
		glTranslatef(Fparticles_2nd[i].p[0], Fparticles_2nd[i].p[1], Fparticles_2nd[i].p[2]);
		glMultMatrixf(cam_mat.mData);       // Orient billboards to face camera

		glBegin(GL_QUADS);
		age_scale = ((float)Fparticles_2nd[i].ttl) / Fparticles_2nd[i].ttl_init;
		glColor3f(age_scale, age_scale, age_scale);
		glTexCoord2f(0.0, 0.0);
		glVertex3f(-FIRE_SIZE, -FIRE_SIZE, 0.0);
		glTexCoord2f(1.0, 0.0);
		glVertex3f(FIRE_SIZE, -FIRE_SIZE, 0.0);
		glTexCoord2f(1.0, 1.0);
		glVertex3f(FIRE_SIZE, FIRE_SIZE, 0.0);
		glTexCoord2f(0.0, 1.0);
		glVertex3f(-FIRE_SIZE, FIRE_SIZE, 0.0);
		glEnd();
		glPopMatrix();
	}

	/*____________________________________________________________________
	|
	| Restore rendering states
	|___________________________________________________________________*/

	glDisable(GL_BLEND);
	glDepthMask(GL_TRUE);
	glDisable(GL_TEXTURE_2D);
	glEnable(GL_LIGHTING);
}

/*
Random number generator by Donald H. House.
Modified to compile on Visual Studio.Net 2003
*/
float FastGauss(float mean, float std)
{
#define RESOLUTION 2500
	static float lookup[RESOLUTION + 1];

#define itblmax    20    
	/* length - 1 of table describing F inverse */
#define didu    40.0    
	/* delta table position / delta ind. variable           itblmax / 0.5 */

	static float tbl[] =
	{ 0.00000E+00, 6.27500E-02, 1.25641E-01, 1.89000E-01,
		2.53333E-01, 3.18684E-01, 3.85405E-01, 4.53889E-01,
		5.24412E-01, 5.97647E-01, 6.74375E-01, 7.55333E-01,
		8.41482E-01, 9.34615E-01, 1.03652E+00, 1.15048E+00,
		1.28167E+00, 1.43933E+00, 1.64500E+00, 1.96000E+00,
		3.87000E+00 };

	static int hereb4;
	//static struct timeval tv;

	float u, di, delta/*, result*/;
	int i, index, minus = 1;

	if (!hereb4) {
		for (i = 0; i <= RESOLUTION; i++) {
			if ((u = i / (float)RESOLUTION) > 0.5) {
				minus = 0;
				u -= 0.5;
			}

			/* interpolate gaussian random number using table */

			index = (int)(di = (didu * u));
			di -= (float)index;
			delta = tbl[index] + (tbl[index + 1] - tbl[index]) * di;
			lookup[i] = (minus ? -delta : delta);
		}

		/*gettimeofday(&tv, NULL);
		srand((unsigned int)tv.tv_usec);*/
		srand((unsigned)time(NULL));
		hereb4 = 1;
	}

	i = rand() / (RAND_MAX / RESOLUTION) + 1;
	if (i > RESOLUTION) {
		i = RESOLUTION;
	}

	return(mean + std * lookup[i]);
}

/*____________________________________________________________________
|
| Function: LoadPPM
|
| Input:
| Output: Draw world axes.
|  LoadPPM - a minimal Portable Pixelformat image file loader
|  fname: name of file to load (input)
|  w: width of loaded image in pixels (output)
|  h: height of loaded image in pixels (output)
|  data: image data address (input or output depending on mallocflag)
|  mallocflag: 1 if memory not pre-allocated, 0 if data already points
|              to allocated memory that can hold the image. Note that
|              if new memory is allocated, free() should be used to
|              deallocate when it is no longer needed.
|___________________________________________________________________*/

void LoadPPM(char *fname, unsigned int *w, unsigned int *h, unsigned char **data, int mallocflag)
{
	FILE *fp;
	char P, num;
	int max;
	char s[1000];

	if (!(fp = fopen(fname, "rb")))
	{
		perror("cannot open image file\n");
		exit(0);
	}

	fscanf(fp, "%c%c\n", &P, &num);
	if ((P != 'P') || (num != '6'))
	{
		perror("unknown file format for image\n");
		exit(0);
	}

	do
	{
		fgets(s, 999, fp);
	} while (s[0] == '#');


	sscanf(s, "%d%d", w, h);
	fgets(s, 999, fp);
	sscanf(s, "%d", &max);

	if (mallocflag)
		if (!(*data = (unsigned char *)malloc(*w * *h * 3)))
		{
			perror("cannot allocate memory for image data\n");
			exit(0);
		}

	fread(*data, 3, *w * *h, fp);

	fclose(fp);
}

//|____________________________________________________________________
//|
//| Function: SetLight
//|
//! \param pos          [in] Light position.
//! \param is_ambient   [in] Is ambient enabled?
//! \param is_diffuse   [in] Is diffuse enabled?
//! \param is_specular  [in] Is specular enabled?
//! \return None.
//!
//! Set light properties.
//|____________________________________________________________________

void SetLight(const gmtl::Point4f &pos, const bool is_ambient, const bool is_diffuse, const bool is_specular)
{
	glLightfv(GL_LIGHT0, GL_POSITION, pos.mData);
	if (is_ambient) {
		glLightfv(GL_LIGHT0, GL_AMBIENT, AMBIENT_LIGHT);
	}
	else {
		glLightfv(GL_LIGHT0, GL_AMBIENT, NO_LIGHT);
	}
	if (is_diffuse) {
		glLightfv(GL_LIGHT0, GL_DIFFUSE, DIFFUSE_LIGHT);
	}
	else {
		glLightfv(GL_LIGHT0, GL_DIFFUSE, NO_LIGHT);
	}
	if (is_specular) {
		glLightfv(GL_LIGHT0, GL_SPECULAR, SPECULAR_LIGHT);
	}
	else {
		glLightfv(GL_LIGHT0, GL_SPECULAR, NO_LIGHT);
	}
}

void DrawPlaneBody()
{
	//glPushMatrix();

	//glMultMatrixf(ppose.mData);

	// Sets materials
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 20.0);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, SPECULAR_COL);

	//Center Body Quad
	glBegin(GL_QUAD_STRIP);
	//glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, DARKRED_COL);
	//glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, BRIGHTRED_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, DARKPLANEBODY_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, BRIGHTPLANEBODY_COL);
	//glColor3ub(105, 169, 185);	
	glNormal3f(0.96f, 0.29f, 0.0f);
	glVertex3f(2.0f, 0.0f, 0.0f);
	glVertex3f(2.0f, 0.0f, -10.0f);
	glVertex3f(1.7f, 1.0f, 0.0f);
	glVertex3f(1.7f, 1.0f, -10.0f);
	glNormal3f(0.7f, 0.7f, 0.0f);
	glVertex3f(1.0f, 1.7f, 0.0f);
	glVertex3f(1.0f, 1.7f, -10.0f);
	glNormal3f(0.29f, 0.96f, 0.0f);
	glVertex3f(0.0f, 2.0f, 0.0f);
	glVertex3f(0.0f, 2.0f, -10.0f);
	glNormal3f(-0.29f, 0.96f, 0.0f);
	glVertex3f(-1.0f, 1.7f, 0.0f);
	glVertex3f(-1.0f, 1.7f, -10.0f);
	glNormal3f(-0.7f, 0.7f, 0.0f);
	glVertex3f(-1.7f, 1.0f, 0.0f);
	glVertex3f(-1.7f, 1.0f, -10.0f);
	glNormal3f(-0.96f, 0.29f, 0.0f);
	glVertex3f(-2.0f, 0.0f, 0.0f);
	glVertex3f(-2.0f, 0.0f, -10.0f);
	glNormal3f(-0.96f, -0.29f, 0.0f);
	glVertex3f(-1.7f, -1.0f, 0.0f);
	glVertex3f(-1.7f, -1.0f, -10.0f);
	glNormal3f(-0.7f, -0.7f, 0.0f);
	glVertex3f(-1.0f, -1.7f, 0.0f);
	glVertex3f(-1.0f, -1.7f, -10.0f);
	glNormal3f(-0.29f, -0.96f, 0.0f);
	glVertex3f(0.0f, -2.0f, 0.0f);
	glVertex3f(0.0f, -2.0f, -10.0f);
	glNormal3f(0.29f, -0.96f, 0.0f);
	glVertex3f(1.0f, -1.7f, 0.0f);
	glVertex3f(1.0f, -1.7f, -10.0f);
	glNormal3f(0.7f, -0.7f, 0.0f);
	glVertex3f(1.7f, -1.0f, 0.0f);
	glVertex3f(1.7f, -1.0f, -10.0f);
	glNormal3f(0.96f, -0.29f, 0.0f);
	glVertex3f(2.0f, 0.0f, 0.0f);
	glVertex3f(2.0f, 0.0f, -10.0f);
	glEnd();

	//Front Quad
	glBegin(GL_QUAD_STRIP);
	//glBegin(GL_QUADS);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, DARKPLANEBODY_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, BRIGHTPLANEBODY_COL);
	//glColor3ub(21, 110, 148);
	glNormal3f(0.79f, 0.55f, 0.26f);
	glVertex3f(1.0f, 0.0f, 3.0f);
	glVertex3f(2.0f, 0.0f, 0.0f);
	glVertex3f(0.87f, 0.5f, 3.0f);
	glVertex3f(1.7f, 1.0f, 0.0f);
	glNormal3f(0.67f, 0.67f, 0.30f);
	glVertex3f(0.5f, 0.87f, 3.0f);
	glVertex3f(1.0f, 1.7f, 0.0f);
	glNormal3f(0.55f, 0.79f, 0.26f);
	glVertex3f(0.0f, 1.0f, 3.0f);
	glVertex3f(0.0f, 2.0f, 0.0f);
	glNormal3f(-0.55f, 0.79f, 0.26f);
	glVertex3f(-0.5f, 0.87f, 3.0f);
	glVertex3f(-1.0f, 1.7f, 0.0f);
	glNormal3f(-0.67f, 0.67f, 0.30f);
	glVertex3f(-0.87f, 0.5f, 3.0f);
	glVertex3f(-1.7f, 1.0f, 0.0f);
	glNormal3f(-0.79f, 0.55f, 0.26f);
	glVertex3f(-1.0f, 0.0f, 3.0f);
	glVertex3f(-2.0f, 0.0f, 0.0f);
	glNormal3f(-0.79f, -0.55f, 0.26f);
	glVertex3f(-0.87f, -0.5f, 3.0f);
	glVertex3f(-1.7f, -1.0f, 0.0f);
	glNormal3f(-0.67f, -0.67f, 0.30f);
	glVertex3f(-0.5f, -0.87f, 3.0f);
	glVertex3f(-1.0f, -1.7f, 0.0f);
	glNormal3f(-0.55f, -0.79f, 0.26f);
	glVertex3f(0.0f, -1.0f, 3.0f);
	glVertex3f(0.0f, -2.0f, 0.0f);
	glNormal3f(0.55f, -0.79f, 0.26f);
	glVertex3f(0.5f, -0.87f, 3.0f);
	glVertex3f(1.0f, -1.7f, 0.0f);
	glNormal3f(0.67f, -0.67f, 0.30f);
	glVertex3f(0.87f, -0.5f, 3.0f);
	glVertex3f(1.7f, -1.0f, 0.0f);
	glNormal3f(0.79f, 0.55f, 0.26f);
	glVertex3f(1.0f, 0.0f, 3.0f);
	glVertex3f(2.0f, 0.0f, 0.0f);
	glEnd();

	//Front Cone
	glBegin(GL_TRIANGLE_FAN);
	//glColor3ub(21, 110, 148);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, DARKRED_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, BRIGHTRED_COL);
	glNormal3f(0.81f, 0.21f, 0.54f);
	glVertex3f(0.0f, 0.0f, 4.5);
	glVertex3f(1.0f, 0.0f, 3.0f);
	glVertex3f(0.87f, 0.5f, 3.0f);
	glNormal3f(0.59f, 0.59f, 0.54f);
	glVertex3f(0.5f, 0.87f, 3.0f);
	glNormal3f(0.21f, 0.81f, 0.54f);
	glVertex3f(0.0f, 1.0f, 3.0f);
	glNormal3f(-0.21f, 0.81f, 0.54f);
	glVertex3f(-0.5f, 0.87f, 3.0f);
	glNormal3f(-0.59f, 0.59f, 0.54f);
	glVertex3f(-0.87f, 0.5f, 3.0f);
	glNormal3f(-0.81f, 0.21f, 0.54f);
	glVertex3f(-1.0f, 0.0f, 3.0f);
	glNormal3f(-0.81f, -0.21f, 0.54f);
	glVertex3f(-0.87f, -0.5f, 3.0f);
	glNormal3f(-0.59f, -0.59f, 0.54f);
	glVertex3f(-0.5f, -0.87f, 3.0f);
	glNormal3f(-0.21f, -0.81f, 0.54f);
	glVertex3f(0.0f, -1.0f, 3.0f);
	glNormal3f(0.21f, -0.81f, 0.54f);
	glVertex3f(0.5f, -0.87f, 3.0f);
	glNormal3f(0.59f, -0.59f, 0.54f);
	glVertex3f(0.87f, -0.5f, 3.0f);
	glNormal3f(0.81f, -0.21f, 0.54f);
	glVertex3f(1.0f, 0.0f, 3.0f);
	glEnd();


	////Back Quad

	glBegin(GL_QUAD_STRIP);
	//glColor3ub(105, 169, 185);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, DARKPLANEBODY_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, BRIGHTPLANEBODY_COL);
	glNormal3f(0.96f, 0.25f, -0.135f);
	glVertex3f(2.0f, 0.0f, -10.0f);
	glVertex3f(1.0f, 0.0f, -17.0f);
	glVertex3f(1.7f, 1.0f, -10.0f);
	glVertex3f(0.87f, 0.5f, -17.0f);
	glNormal3f(0.7f, 0.7f, -0.135f);
	glVertex3f(1.0f, 1.7f, -10.0f);
	glVertex3f(0.5f, 0.87f, -17.0f);
	glNormal3f(0.25f, 0.96f, -0.135f);
	glVertex3f(0.0f, 2.0f, -10.0f);
	glVertex3f(0.0f, 1.0f, -17.0f);
	glNormal3f(-0.25f, 0.96f, -0.135f);
	glVertex3f(-1.0f, 1.7f, -10.0f);
	glVertex3f(-0.5f, 0.87f, -17.0f);
	glNormal3f(-0.7f, 0.7f, -0.135f);
	glVertex3f(-1.7f, 1.0f, -10.0f);
	glVertex3f(-0.87f, 0.5f, -17.0f);
	glNormal3f(-0.96f, 0.25f, -0.135f);
	glVertex3f(-2.0f, 0.0f, -10.0f);
	glVertex3f(-1.0f, 0.0f, -17.0f);
	glNormal3f(-0.96f, -0.25f, -0.135f);
	glVertex3f(-1.7f, -1.0f, -10.0f);
	glVertex3f(-0.87f, -0.5f, -17.0f);
	glNormal3f(-0.7f, -0.7f, -0.135f);
	glVertex3f(-1.0f, -1.7f, -10.0f);
	glVertex3f(-0.5f, -0.87f, -17.0f);
	glNormal3f(-0.25f, -0.96f, -0.135f);
	glVertex3f(0.0f, -2.0f, -10.0f);
	glVertex3f(0.0f, -1.0f, -17.0f);
	glNormal3f(0.25f, -0.96f, -0.135f);
	glVertex3f(1.0f, -1.7f, -10.0f);
	glVertex3f(0.5f, -0.87f, -17.0f);
	glNormal3f(0.7f, -0.7f, -0.135f);
	glVertex3f(1.7f, -1.0f, -10.0f);
	glVertex3f(0.87f, -0.5f, -17.0f);
	glNormal3f(0.96f, -0.25f, -0.135f);
	glVertex3f(2.0f, 0.0f, -10.0f);
	glVertex3f(1.0f, 0.0f, -17.0f);
	glEnd();

	//Top Back Wing
	glBegin(GL_QUADS);
	//glColor3ub(66, 126, 141);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, DARKPLANEWING_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, BRIGHTPLANEWING_COL);

	glNormal3f(1.0f, 0.0f, 0.0f);
	glVertex3f(-0.15f, 0.0f, -12.0f);
	glVertex3f(-0.15f, 0.0f, -17.0f);
	glVertex3f(-0.15f, 5.0f, -17.0f);
	glVertex3f(-0.15f, 5.0f, -15.0f);

	glNormal3f(1.0f, 0.0f, 0.0f);
	glVertex3f(0.15f, 0.0f, -12.0f);
	glVertex3f(0.15f, 0.0f, -17.0f);
	glVertex3f(0.15f, 5.0f, -17.0f);
	glVertex3f(0.15f, 5.0f, -15.0f);

	glNormal3f(0.0f, -1.0f, 0.0f);
	glVertex3f(-0.15f, 5.0f, -17.0f);
	glVertex3f(0.15f, 5.0f, -17.0f);
	glVertex3f(0.15f, 5.0f, -15.0f);
	glVertex3f(-0.15f, 5.0f, -15.0f);

	glNormal3f(0.0f, 0.51f, 0.86f);
	glVertex3f(0.15f, 5.0f, -15.0f);
	glVertex3f(-0.15f, 5.0f, -15.0f);
	glVertex3f(-0.15f, 0.0f, -12.0f);
	glVertex3f(0.15f, 0.0f, -12.0f);

	glNormal3f(0.0f, 0.0f, 1.0f);
	glVertex3f(0.15f, 5.0f, -17.0f);
	glVertex3f(-0.15f, 5.0f, -17.0f);
	glVertex3f(-0.15f, 0.0f, -17.0f);
	glVertex3f(0.15f, 0.0f, -17.0f);
	glEnd();

	//Left Back Wing
	glBegin(GL_QUADS);
	//glColor3ub(66, 126, 141);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, DARKPLANEWING_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, BRIGHTPLANEWING_COL);
	glNormal3f(0.0f, -1.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, -14.0f);
	glVertex3f(0.0f, 0.0f, -17.0f);
	glVertex3f(5.0f, 0.0f, -17.0f);
	glVertex3f(5.0f, 0.0f, -15.0f);

	glNormal3f(0.0f, -1.0f, 0.0f);
	glVertex3f(0.0f, 0.5f, -14.0f);
	glVertex3f(0.0f, 0.5f, -17.0f);
	glVertex3f(5.0f, 0.5f, -17.0f);
	glVertex3f(5.0f, 0.5f, -15.0f);

	glNormal3f(1.0f, 0.0f, 0.0f);
	glVertex3f(5.0f, 0.5f, -17.0f);
	glVertex3f(5.0f, 0.5f, -15.0f);
	glVertex3f(5.0f, 0.0f, -15.0f);
	glVertex3f(5.0f, 0.0f, -17.0f);

	glNormal3f(0.17f, 0.0f, -0.98f);
	glVertex3f(5.0f, 0.5f, -15.0f);
	glVertex3f(5.0f, 0.0f, -15.0f);
	glVertex3f(0.0f, 0.0f, -14.0f);
	glVertex3f(0.0f, 0.5f, -14.0f);

	glNormal3f(0.0f, 0.0f, -1.0f);
	glVertex3f(5.0f, 0.5f, -17.0f);
	glVertex3f(5.0f, 0.0f, -17.0f);
	glVertex3f(0.0f, 0.0f, -17.0f);
	glVertex3f(0.0f, 0.5f, -17.0f);
	glEnd();

	//Rigth Back Wing
	glBegin(GL_QUADS);
	//glColor3ub(66, 126, 141);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, DARKPLANEWING_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, BRIGHTPLANEWING_COL);
	glNormal3f(0.0f, 1.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, -14.0f);
	glVertex3f(0.0f, 0.0f, -17.0f);
	glVertex3f(-5.0f, 0.0f, -17.0f);
	glVertex3f(-5.0f, 0.0f, -15.0f);

	glNormal3f(0.0f, 1.0f, 0.0f);
	glVertex3f(0.0f, 0.5f, -14.0f);
	glVertex3f(0.0f, 0.5f, -17.0f);
	glVertex3f(-5.0f, 0.5f, -17.0f);
	glVertex3f(-5.0f, 0.5f, -15.0f);

	glNormal3f(1.0f, 0.0f, 0.0f);
	glVertex3f(-5.0f, 0.5f, -17.0f);
	glVertex3f(-5.0f, 0.5f, -15.0f);
	glVertex3f(-5.0f, 0.0f, -15.0f);
	glVertex3f(-5.0f, 0.0f, -17.0f);

	glNormal3f(-0.17f, 0.0f, 0.98f);
	glVertex3f(-5.0f, 0.5f, -15.0f);
	glVertex3f(-5.0f, 0.0f, -15.0f);
	glVertex3f(0.0f, 0.0f, -14.0f);
	glVertex3f(0.0f, 0.5f, -14.0f);

	glNormal3f(0.0f, 0.0f, 1.0f);
	glVertex3f(-5.0f, 0.5f, -17.0f);
	glVertex3f(-5.0f, 0.0f, -17.0f);
	glVertex3f(0.0f, 0.0f, -17.0f);
	glVertex3f(0.0f, 0.5f, -17.0f);
	glEnd();

	//Left Center Wing
	glBegin(GL_QUADS);
	//glColor3ub(0, 40, 40);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, DARKPLANEWING_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, BRIGHTPLANEWING_COL);
	glNormal3f(0.0f, -1.0f, 0.0f);
	glVertex3f(0.0f, -0.5f, -3.0f);
	glVertex3f(0.0f, -0.5f, -8.0f);
	glVertex3f(12.0f, -0.5f, -9.0f);
	glVertex3f(12.0f, -0.5f, -5.0f);

	////glColor3ub(66, 126, 141);
	glNormal3f(0.0f, -1.0f, 0.0f);
	glVertex3f(0.0f, 0.5f, -3.0f);
	glVertex3f(0.0f, 0.5f, -8.0f);
	glVertex3f(12.0f, 0.5f, -9.0f);
	glVertex3f(12.0f, 0.5f, -5.0f);

	glNormal3f(1.0f, 0.0f, 0.0f);
	glVertex3f(12.0f, 0.5f, -9.0f);
	glVertex3f(12.0f, 0.5f, -5.0f);
	glVertex3f(12.0f, -0.5f, -5.0f);
	glVertex3f(12.0f, -0.5f, -9.0f);

	////glColor3ub(0, 0, 50);
	glNormal3f(0.08f, 0.0f, 0.99f);
	glVertex3f(12.0f, -0.5f, -9.0f);
	glVertex3f(12.0f, 0.5f, -9.0f);
	glVertex3f(0.0f, 0.5f, -8.0f);
	glVertex3f(0.0f, -0.5f, -8.0f);

	glNormal3f(0.16f, 0.0f, 0.99f);
	glVertex3f(12.0f, -0.5f, -5.0f);
	glVertex3f(12.0f, 0.5f, -5.0f);
	glVertex3f(0.0f, 0.5f, -3.0f);
	glVertex3f(0.0f, -0.5f, -3.0f);
	glEnd();

	//Right Center Wing
	glBegin(GL_QUADS);
	//glColor3ub(0, 40, 40);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, DARKPLANEWING_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, BRIGHTPLANEWING_COL);
	glNormal3f(0.0f, 1.0f, 0.0f);
	glVertex3f(0.0f, -0.5f, -3.0f);
	glVertex3f(0.0f, -0.5f, -8.0f);
	glVertex3f(-12.0f, -0.5f, -9.0f);
	glVertex3f(-12.0f, -0.5f, -5.0f);

	//glColor3ub(66, 126, 141);
	glNormal3f(0.0f, 1.0f, 0.0f);
	glVertex3f(0.0f, 0.5f, -3.0f);
	glVertex3f(0.0f, 0.5f, -8.0f);
	glVertex3f(-12.0f, 0.5f, -9.0f);
	glVertex3f(-12.0f, 0.5f, -5.0f);

	glNormal3f(1.0f, 0.0f, 0.0f);
	glVertex3f(-12.0f, 0.5f, -9.0f);
	glVertex3f(-12.0f, 0.5f, -5.0f);
	glVertex3f(-12.0f, -0.5f, -5.0f);
	glVertex3f(-12.0f, -0.5f, -9.0f);

	////glColor3ub(0, 0, 50);
	glNormal3f(0.08f, 0.0f, -0.99f);
	glVertex3f(-12.0f, -0.5f, -9.0f);
	glVertex3f(-12.0f, 0.5f, -9.0f);
	glVertex3f(0.0f, 0.5f, -8.0f);
	glVertex3f(0.0f, -0.5f, -8.0f);

	glNormal3f(0.16f, 0.0f, -0.99f);
	glVertex3f(-12.0f, -0.5f, -5.0f);
	glVertex3f(-12.0f, 0.5f, -5.0f);
	glVertex3f(0.0f, 0.5f, -3.0f);
	glVertex3f(0.0f, -0.5f, -3.0f);
	glEnd();

	//Jet Engine
	glBegin(GL_QUAD_STRIP);
	//glColor3f(0.2f, 0.2f, 0.2f);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, DARKPLANEJET_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, BRIGHTPLANEJET_COL);
	glNormal3f(0.93f, 0.26f, -0.23f);
	glVertex3f(1.0f, 0.0f, -17.0f);
	glVertex3f(0.5f, 0.0f, -19.0f);
	glVertex3f(0.87f, 0.5f, -17.0f);
	glVertex3f(0.43f, 0.25f, -19.0f);
	glNormal3f(0.69f, 0.69f, -0.24f);
	glVertex3f(0.5f, 0.87f, -17.0f);
	glVertex3f(0.25f, 0.43f, -19.0f);
	glNormal3f(0.26f, 0.93f, -0.23f);
	glVertex3f(0.0f, 1.0f, -17.0f);
	glVertex3f(0.0f, 0.5f, -19.0f);
	glNormal3f(-0.26f, 0.93f, -0.23f);
	glVertex3f(-0.5f, 0.87f, -17.0f);
	glVertex3f(-0.25f, 0.43f, -19.0f);
	glNormal3f(-0.69f, 0.69f, -0.24f);
	glVertex3f(-0.87f, 0.5f, -17.0f);
	glVertex3f(-0.43f, 0.25f, -19.0f);
	glNormal3f(-0.93f, 0.26f, -0.23f);
	glVertex3f(-1.0f, 0.0f, -17.0f);
	glVertex3f(-0.5f, 0.0f, -19.0f);
	glNormal3f(-0.93f, -0.26f, -0.23f);
	glVertex3f(-0.87f, -0.5f, -17.0f);
	glVertex3f(-0.43f, -0.25f, -19.0f);
	glNormal3f(-0.69f, -0.69f, -0.24f);
	glVertex3f(-0.5f, -0.87f, -17.0f);
	glVertex3f(-0.25f, -0.43f, -19.0f);
	glNormal3f(-0.26f, -0.93f, -0.23f);
	glVertex3f(0.0f, -1.0f, -17.0f);
	glVertex3f(0.0f, -0.5f, -19.0f);
	glNormal3f(0.26f, -0.93f, -0.23f);
	glVertex3f(0.5f, -0.87f, -17.0f);
	glVertex3f(0.25f, -0.43f, -19.0f);
	glNormal3f(0.69f, -0.69f, -0.24f);
	glVertex3f(0.87f, -0.5f, -17.0f);
	glVertex3f(0.43f, -0.25f, -19.0f);
	glNormal3f(0.69f, -0.69f, -0.24f);
	glVertex3f(1.0f, 0.0f, -17.0f);
	glVertex3f(0.5f, 0.0f, -19.0f);
	glEnd();

	// JET FIRE
	glBegin(GL_TRIANGLE_FAN);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, DARKRED_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, BRIGHTRED_COL);
	glNormal3f(0.0f, 0.0f, 1.0f);
	glVertex3f(0.0f, 0.0f, -18.0f);
	glVertex3f(0.75f, 0.0f, -18.0f);
	glVertex3f(0.65f, 0.375f, -18.0f);
	glVertex3f(0.375f, 0.65f, -18.0f);
	glVertex3f(0.0f, 0.75f, -18.0f);
	glVertex3f(-0.375f, 0.65f, -18.0f);
	glVertex3f(-0.65f, 0.375f, -18.0f);
	glVertex3f(-0.75f, 0.0f, -18.0f);
	glVertex3f(-0.65f, -0.375f, -18.0f);
	glVertex3f(-0.375f, -0.65f, -18.0f);
	glVertex3f(0.0f, -0.75f, -18.0f);
	glVertex3f(0.375f, -0.65f, -18.0f);
	glVertex3f(0.65f, -0.375f, -18.0f);
	glVertex3f(0.75f, 0.0f, -18.0f);
	glEnd();

	//glPopMatrix();
}

void DrawMissileEngine() {

	// Sets materials
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 20.0);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, SPECULAR_COL);

	glBegin(GL_QUADS);
	//glColor3ub(0, 40, 40);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, DARKPLANEBODY_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, BRIGHTPLANEBODY_COL);
	glNormal3f(0.0f, -1.0f, 0.0f);
	glVertex3f(4.0f, 0.0f, -4.0f);
	glVertex3f(4.0f, 0.0f, -7.0f);
	glVertex3f(8.0f, 0.0f, -7.0f);
	glVertex3f(8.0f, 0.0f, -5.5f);

	//glColor3ub(66, 140, 150);
	glNormal3f(0.0f, -1.0f, 0.0f);
	glVertex3f(4.0f, 0.55f, -4.0f);
	glVertex3f(4.0f, 0.55f, -7.0f);
	glVertex3f(8.0f, 0.55f, -7.0f);
	glVertex3f(8.0f, 0.55f, -5.5f);

	glNormal3f(1.0f, 0.0f, 0.0f);
	glVertex3f(4.0f, 0.0f, -4.0f);
	glVertex3f(4.0f, 0.0f, -7.0f);
	glVertex3f(4.0f, 0.55f, -7.0f);
	glVertex3f(4.0f, 0.55f, -4.0f);

	glNormal3f(-1.0f, 0.0f, 0.0f);
	glVertex3f(8.0f, 0.0f, -4.0f);
	glVertex3f(8.0f, 0.0f, -5.5f);
	glVertex3f(8.0f, 0.55f, -5.4f);
	glVertex3f(8.0f, 0.55f, -7.0f);

	//glColor3ub(0, 0, 50);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, DARKMISSILE_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, BRIGHTMISSILE_COL);
	glNormal3f(0.35f, 0.0f, -0.93f);
	glVertex3f(4.0f, 0.0f, -4.1f);
	glVertex3f(4.0f, 0.55f, -4.1f);
	glVertex3f(8.0f, 0.55f, -5.6f);
	glVertex3f(8.0f, 0.0f, -5.6f);

	//glColor3ub(0, 40, 40);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, DARKPLANEBODY_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, BRIGHTPLANEBODY_COL);
	glNormal3f(0.0f, 1.0f, 0.0f);
	glVertex3f(-4.0f, 0.0f, -4.0f);
	glVertex3f(-4.0f, 0.0f, -7.0f);
	glVertex3f(-8.0f, 0.0f, -7.0f);
	glVertex3f(-8.0f, 0.0f, -5.5f);

	//glColor3ub(66, 140, 150);
	glNormal3f(0.0f, 1.0f, 0.0f);
	glVertex3f(-4.0f, 0.55f, -4.0f);
	glVertex3f(-4.0f, 0.55f, -7.0f);
	glVertex3f(-8.0f, 0.55f, -7.0f);
	glVertex3f(-8.0f, 0.55f, -5.5f);

	glNormal3f(1.0f, 0.0f, 0.0f);
	glVertex3f(-4.0f, 0.0f, -4.0f);
	glVertex3f(-4.0f, 0.0f, -7.0f);
	glVertex3f(-4.0f, 0.55f, -7.0f);
	glVertex3f(-4.0f, 0.55f, -4.0f);

	glNormal3f(-1.0f, 0.0f, 0.0f);
	glVertex3f(-8.0f, 0.0f, -7.0f);
	glVertex3f(-8.0f, 0.0f, -5.5f);
	glVertex3f(-8.0f, 0.55f, -5.5f);
	glVertex3f(-8.0f, 0.55f, -7.0f);

	//glColor3ub(0, 0, 50);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, DARKMISSILE_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, BRIGHTMISSILE_COL);
	glNormal3f(0.35f, 0.0f, 0.93f);
	glVertex3f(-4.0f, 0.0f, -4.1f);
	glVertex3f(-4.0f, 0.55f, -4.1f);
	glVertex3f(-8.0f, 0.55f, -5.6f);
	glVertex3f(-8.0f, 0.0f, -5.6f);

	glEnd();

}

void DrawCargoDoor() {

	// Sets materials
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 20.0);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, SPECULAR_COL);

	glBegin(GL_QUAD_STRIP);
	//glColor3ub(66, 140, 150);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, DARKPLANEWING_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, BRIGHTPLANEWING_COL);
	glNormal3f(-0.55f, -0.83, -0.14);
	glVertex3f(-1.0f, -1.7f, -10.0f);
	glVertex3f(-0.5f, -0.87f, -17.0f);
	glVertex3f(0.0f, -2.2f, -10.0f);
	glVertex3f(0.0f, -1.2f, -17.0f);
	glNormal3f(0.55f, -0.83, -0.14);
	glVertex3f(1.0f, -1.7f, -10.0f);
	glVertex3f(0.5f, -0.87f, -17.0f);
	glEnd();
}

void DrawLightBody() {

	// Sets materials
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 20.0);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, SPECULAR_COL);

	glBegin(GL_QUADS);
	//glColor3ub(66, 140, 150);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, DARKPLANEWING_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, BRIGHTPLANEWING_COL);
	glNormal3f(0.0f, -1.0f, 0.0f);
	glVertex3f(4.0f, 0.0f, -4.5f);
	glVertex3f(4.0f, 0.0f, -8.0f);
	glVertex3f(4.4f, 0.0f, -8.0f);
	glVertex3f(4.4f, 0.0f, -4.5f);

	glNormal3f(0.0f, 1.0f, 0.0f);
	glVertex3f(8.0f, 0.0f, -4.5f);
	glVertex3f(8.0f, 0.0f, -8.0f);
	glVertex3f(7.6f, 0.0f, -8.0f);
	glVertex3f(7.6f, 0.0f, -4.5f);

	glNormal3f(0.0f, -1.0f, 0.0f);
	glVertex3f(4.0f, -0.55f, -4.5f);
	glVertex3f(4.0f, -0.55f, -8.0f);
	glVertex3f(4.4f, -0.55f, -8.0f);
	glVertex3f(4.4f, -0.55f, -4.5f);

	glNormal3f(0.0f, 1.0f, 0.0f);
	glVertex3f(8.0f, -0.55f, -4.5f);
	glVertex3f(8.0f, -0.55f, -8.0f);
	glVertex3f(7.6f, -0.55f, -8.0f);
	glVertex3f(7.6f, -0.55f, -4.5f);

	glNormal3f(-1.0f, 0.0f, 0.0f);
	glVertex3f(4.0f, 0.0f, -4.5f);
	glVertex3f(4.0f, 0.0f, -8.0f);
	glVertex3f(4.0f, -0.55f, -8.0f);
	glVertex3f(4.0f, -0.55f, -4.5f);

	glNormal3f(-1.0f, 0.0f, 0.0f);
	glVertex3f(4.4f, 0.0f, -4.5f);
	glVertex3f(4.4f, 0.0f, -8.0f);
	glVertex3f(4.4f, -0.55f, -8.0f);
	glVertex3f(4.4f, -0.55f, -4.5f);

	glNormal3f(-1.0f, 0.0f, 0.0f);
	glVertex3f(7.6f, 0.0f, -4.5f);
	glVertex3f(7.6f, 0.0f, -8.0f);
	glVertex3f(7.6f, -0.55f, -8.0f);
	glVertex3f(7.6f, -0.55f, -4.5f);

	glNormal3f(-1.0f, 0.0f, 0.0f);
	glVertex3f(8.0f, 0.0f, -4.5f);
	glVertex3f(8.0f, 0.0f, -8.0f);
	glVertex3f(8.0f, -0.55f, -8.0f);
	glVertex3f(8.0f, -0.55f, -4.5f);

	glNormal3f(0.0f, 0.0f, 1.0f);
	glVertex3f(4.0f, 0.0f, -8.0f);
	glVertex3f(4.0f, -0.55f, -8.0f);
	glVertex3f(8.0f, -0.55f, -8.0f);
	glVertex3f(8.0f, 0.0f, -8.0f);

	glNormal3f(0.0f, 0.0f, 1.0f);
	glVertex3f(4.0f, 0.0f, -7.5f);
	glVertex3f(4.0f, -0.55f, -7.5f);
	glVertex3f(8.0f, -0.55f, -7.5f);
	glVertex3f(8.0f, 0.0f, -7.5f);

	glNormal3f(0.0f, 1.0f, 0.0f);
	glVertex3f(4.0f, 0.0f, -8.0f);
	glVertex3f(4.0f, 0.0f, -7.5f);
	glVertex3f(8.0f, 0.0f, -7.5f);
	glVertex3f(8.0f, 0.0f, -8.0f);

	glNormal3f(0.0f, 1.0f, 0.0f);
	glVertex3f(4.0f, -0.55f, -8.0f);
	glVertex3f(4.0f, -0.55f, -7.5f);
	glVertex3f(8.0f, -0.55f, -7.5f);
	glVertex3f(8.0f, -0.55f, -8.0f);

	glNormal3f(0.0f, 0.0f, 1.0f);
	glVertex3f(4.0f, 0.0f, -5.0f);
	glVertex3f(4.0f, -0.55f, -5.0f);
	glVertex3f(8.0f, -0.55f, -5.0f);
	glVertex3f(8.0f, 0.0f, -5.0f);

	glNormal3f(0.0f, -1.0f, 0.0f);
	glVertex3f(4.0f, -0.55f, -4.5f);
	glVertex3f(4.0f, -0.55f, -5.0f);
	glVertex3f(8.0f, -0.55f, -5.0f);
	glVertex3f(8.0f, -0.55f, -4.5f);

	glNormal3f(0.0f, -1.0f, 0.0f);
	glVertex3f(4.0f, 0.0f, -4.5f);
	glVertex3f(4.0f, 0.0f, -5.0f);
	glVertex3f(8.0f, 0.0f, -5.0f);
	glVertex3f(8.0f, 0.0f, -4.5f);

	//glColor3ub(255, 255, 102);
	glNormal3f(0.0f, 0.0f, 1.0f);
	glVertex3f(4.0f, 0.0f, -4.5f);
	glVertex3f(4.0f, -0.55f, -4.5f);
	glVertex3f(8.0f, -0.55f, -4.5f);
	glVertex3f(8.0f, 0.0f, -4.5f);
	glEnd();

	glBegin(GL_QUADS);
	//glColor3ub(66, 140, 150);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, DARKPLANEWING_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, BRIGHTPLANEWING_COL);
	glNormal3f(0.0f, 1.0f, 0.0f);
	glVertex3f(-4.0f, 0.0f, -4.5f);
	glVertex3f(-4.0f, 0.0f, -8.0f);
	glVertex3f(-4.4f, 0.0f, -8.0f);
	glVertex3f(-4.4f, 0.0f, -4.5f);

	glNormal3f(0.0f, -1.0f, 0.0f);
	glVertex3f(-8.0f, 0.0f, -4.5f);
	glVertex3f(-8.0f, 0.0f, -8.0f);
	glVertex3f(-7.6f, 0.0f, -8.0f);
	glVertex3f(-7.6f, 0.0f, -4.5f);

	glNormal3f(0.0f, 1.0f, 0.0f);
	glVertex3f(-4.0f, -0.55f, -4.5f);
	glVertex3f(-4.0f, -0.55f, -8.0f);
	glVertex3f(-4.4f, -0.55f, -8.0f);
	glVertex3f(-4.4f, -0.55f, -4.5f);

	glNormal3f(0.0f, -1.0f, 0.0f);
	glVertex3f(-8.0f, -0.55f, -4.5f);
	glVertex3f(-8.0f, -0.55f, -8.0f);
	glVertex3f(-7.6f, -0.55f, -8.0f);
	glVertex3f(-7.6f, -0.55f, -4.5f);

	glNormal3f(-1.0f, 0.0f, 0.0f);
	glVertex3f(-4.0f, 0.0f, -4.5f);
	glVertex3f(-4.0f, 0.0f, -8.0f);
	glVertex3f(-4.0f, -0.55f, -8.0f);
	glVertex3f(-4.0f, -0.55f, -4.5f);

	glNormal3f(-1.0f, 0.0f, 0.0f);
	glVertex3f(-4.4f, 0.0f, -4.5f);
	glVertex3f(-4.4f, 0.0f, -8.0f);
	glVertex3f(-4.4f, -0.55f, -8.0f);
	glVertex3f(-4.4f, -0.55f, -4.5f);

	glNormal3f(-1.0f, 0.0f, 0.0f);
	glVertex3f(-7.6f, 0.0f, -4.5f);
	glVertex3f(-7.6f, 0.0f, -8.0f);
	glVertex3f(-7.6f, -0.55f, -8.0f);
	glVertex3f(-7.6f, -0.55f, -4.5f);

	glNormal3f(-1.0f, 0.0f, 0.0f);
	glVertex3f(-8.0f, 0.0f, -4.5f);
	glVertex3f(-8.0f, 0.0f, -8.0f);
	glVertex3f(-8.0f, -0.55f, -8.0f);
	glVertex3f(-8.0f, -0.55f, -4.5f);

	glNormal3f(0.0f, 0.0f, -1.0f);
	glVertex3f(-4.0f, 0.0f, -8.0f);
	glVertex3f(-4.0f, -0.55f, -8.0f);
	glVertex3f(-8.0f, -0.55f, -8.0f);
	glVertex3f(-8.0f, 0.0f, -8.0f);

	glNormal3f(0.0f, 0.0f, -1.0f);
	glVertex3f(-4.0f, 0.0f, -7.5f);
	glVertex3f(-4.0f, -0.55f, -7.5f);
	glVertex3f(-8.0f, -0.55f, -7.5f);
	glVertex3f(-8.0f, 0.0f, -7.5f);

	glNormal3f(0.0f, -1.0f, 0.0f);
	glVertex3f(-4.0f, 0.0f, -8.0f);
	glVertex3f(-4.0f, 0.0f, -7.5f);
	glVertex3f(-8.0f, 0.0f, -7.5f);
	glVertex3f(-8.0f, 0.0f, -8.0f);

	glNormal3f(0.0f, -1.0f, 0.0f);
	glVertex3f(-4.0f, -0.55f, -8.0f);
	glVertex3f(-4.0f, -0.55f, -7.5f);
	glVertex3f(-8.0f, -0.55f, -7.5f);
	glVertex3f(-8.0f, -0.55f, -8.0f);

	glNormal3f(0.0f, 0.0f, -1.0f);
	glVertex3f(-4.0f, 0.0f, -5.0f);
	glVertex3f(-4.0f, -0.55f, -5.0f);
	glVertex3f(-8.0f, -0.55f, -5.0f);
	glVertex3f(-8.0f, 0.0f, -5.0f);

	glNormal3f(0.0f, 1.0f, 0.0f);
	glVertex3f(-4.0f, -0.55f, -4.5f);
	glVertex3f(-4.0f, -0.55f, -5.0f);
	glVertex3f(-8.0f, -0.55f, -5.0f);
	glVertex3f(-8.0f, -0.55f, -4.5f);

	glNormal3f(0.0f, 1.0f, 0.0f);
	glVertex3f(-4.0f, 0.0f, -4.5f);
	glVertex3f(-4.0f, 0.0f, -5.0f);
	glVertex3f(-8.0f, 0.0f, -5.0f);
	glVertex3f(-8.0f, 0.0f, -4.5f);

	//glColor3ub(255, 255, 102);
	glNormal3f(0.0f, 0.0f, -1.0f);
	glVertex3f(-4.0f, 0.0f, -4.5f);
	glVertex3f(-4.0f, -0.55f, -4.5f);
	glVertex3f(-8.0f, -0.55f, -4.5f);
	glVertex3f(-8.0f, 0.0f, -4.5f);
	glEnd();

}

void DrawLightObject() {

	// Sets materials
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 20.0);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, SPECULAR_COL);

	glBegin(GL_QUADS);
	//glColor3ub(0, 0, 50);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, DARKRED_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, BRIGHTRED_COL);
	glNormal3f(0.0f, -1.0f, 0.0f);
	glVertex3f(4.5f, 0.05f, -5.1f);
	glVertex3f(4.5f, 0.05f, -7.5f);
	glVertex3f(7.5f, 0.05f, -7.5f);
	glVertex3f(7.5f, 0.05f, -5.1f);

	glNormal3f(0.0f, -1.0f, 0.0f);
	glVertex3f(4.5f, -0.60f, -5.1f);
	glVertex3f(4.5f, -0.60f, -7.5f);
	glVertex3f(7.5f, -0.60f, -7.5f);
	glVertex3f(7.5f, -0.60f, -5.1f);

	glNormal3f(-1.0f, 0.0f, 0.0f);
	glVertex3f(4.5f, 0.05f, -5.1f);
	glVertex3f(4.5f, 0.05f, -7.5f);
	glVertex3f(4.5f, -0.60f, -7.5f);
	glVertex3f(4.5f, -0.60f, -5.1f);

	glNormal3f(1.0f, 0.0f, 0.0f);
	glVertex3f(7.5f, 0.05f, -7.5f);
	glVertex3f(7.5f, 0.05f, -5.1f);
	glVertex3f(7.5f, -0.60f, -5.1f);
	glVertex3f(7.5f, -0.60f, -7.5f);

	//glColor3ub(255, 255, 102);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, DARKLIGHT_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, BRIGHTLIGHT_COL);
	glNormal3f(0.0f, 0.0f, 1.0f);
	glVertex3f(4.5f, 0.05f, -5.1f);
	glVertex3f(4.5f, -0.60f, -5.1f);
	glVertex3f(7.5f, -0.60f, -5.1f);
	glVertex3f(7.5f, 0.05f, -5.1f);
	glEnd();

	glBegin(GL_QUADS);
	//glColor3ub(0, 0, 50);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, DARKRED_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, BRIGHTRED_COL);
	glNormal3f(0.0f, 1.0f, 0.0f);
	glVertex3f(-4.5f, 0.05f, -5.1f);
	glVertex3f(-4.5f, 0.05f, -7.5f);
	glVertex3f(-7.5f, 0.05f, -7.5f);
	glVertex3f(-7.5f, 0.05f, -5.1f);

	glNormal3f(0.0f, 1.0f, 0.0f);
	glVertex3f(-4.5f, -0.60f, -5.1f);
	glVertex3f(-4.5f, -0.60f, -7.5f);
	glVertex3f(-7.5f, -0.60f, -7.5f);
	glVertex3f(-7.5f, -0.60f, -5.1f);

	glNormal3f(-1.0f, 0.0f, 0.0f);
	glVertex3f(-4.5f, 0.05f, -5.1f);
	glVertex3f(-4.5f, 0.05f, -7.5f);
	glVertex3f(-4.5f, -0.60f, -7.5f);
	glVertex3f(-4.5f, -0.60f, -5.1f);

	glNormal3f(1.0f, 0.0f, 0.0f);
	glVertex3f(-7.5f, 0.05f, -7.5f);
	glVertex3f(-7.5f, 0.05f, -5.1f);
	glVertex3f(-7.5f, -0.60f, -5.1f);
	glVertex3f(-7.5f, -0.60f, -7.5f);

	//glColor3ub(255, 255, 102);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, DARKLIGHT_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, BRIGHTLIGHT_COL);
	glNormal3f(0.0f, 0.0f, -1.0f);
	glVertex3f(-4.5f, 0.05f, -5.1f);
	glVertex3f(-4.5f, -0.60f, -5.1f);
	glVertex3f(-7.5f, -0.60f, -5.1f);
	glVertex3f(-7.5f, 0.05f, -5.1f);
	glEnd();
}

void DrawSkybox(const float s)
{
	float s2 = s / 2;

	// Turn on texture mapping and disable lighting
	glEnable(GL_TEXTURE_2D);
	glDisable(GL_LIGHTING);

	// Back wall
	glBindTexture(GL_TEXTURE_2D, texturesAddOn[TID_SKYBACK]);  // Specify which texture will be used   
	glBegin(GL_QUADS);
	glColor3f(1.0f, 1.0f, 1.0f);
	glTexCoord2f(0.01, 0.99);
	glVertex3f(-s2, -s2, -s2);
	glTexCoord2f(0.99, 0.99);
	glVertex3f(s2, -s2, -s2);
	glTexCoord2f(0.99, 0.01);
	glVertex3f(s2, s2, -s2);
	glTexCoord2f(0.01, 0.01);
	glVertex3f(-s2, s2, -s2);
	glEnd();

	// Left wall
	glBindTexture(GL_TEXTURE_2D, texturesAddOn[TID_SKYLEFT]);
	glBegin(GL_QUADS);
	glColor3f(1.0f, 1.0f, 1.0f);
	glTexCoord2f(0.01, 0.99);
	glVertex3f(-s2, -s2, s2);
	glTexCoord2f(0.99, 0.99);
	glVertex3f(-s2, -s2, -s2);
	glTexCoord2f(0.99, 0.01);
	glVertex3f(-s2, s2, -s2);
	glTexCoord2f(0.01, 0.01);
	glVertex3f(-s2, s2, s2);
	glEnd();

	// Bottom wall
	glBindTexture(GL_TEXTURE_2D, texturesAddOn[TID_SKYBOTTOM]);
	glBegin(GL_QUADS);
	glColor3f(1.0f, 1.0f, 1.0f);
	glTexCoord2f(0.01, 0.99);
	glVertex3f(-s2, -s2, s2);
	glTexCoord2f(0.99, 0.99);
	glVertex3f(s2, -s2, s2);
	glTexCoord2f(0.99, 0.01);
	glVertex3f(s2, -s2, -s2);
	glTexCoord2f(0.01, 0.01);
	glVertex3f(-s2, -s2, -s2);
	glEnd();

	//Add on
	// Right wall
	glBindTexture(GL_TEXTURE_2D, texturesAddOn[TID_SKYRIGHT]);
	glBegin(GL_QUADS);
	glColor3f(1.0f, 1.0f, 1.0f);
	glTexCoord2f(0.01, 0.99);
	glVertex3f(s2, -s2, -s2);
	glTexCoord2f(0.99, 0.99);
	glVertex3f(s2, -s2, s2);
	glTexCoord2f(0.99, 0.01);
	glVertex3f(s2, s2, s2);
	glTexCoord2f(0.01, 0.01);
	glVertex3f(s2, s2, -s2);
	glEnd();

	// Front wall
	glBindTexture(GL_TEXTURE_2D, texturesAddOn[TID_SKYFRONT]);
	glBegin(GL_QUADS);
	glColor3f(1.0f, 1.0f, 1.0f);
	glTexCoord2f(0.01, 0.99);
	glVertex3f(s2, -s2, s2);
	glTexCoord2f(0.99, 0.99);
	glVertex3f(-s2, -s2, s2);
	glTexCoord2f(0.99, 0.01);
	glVertex3f(-s2, s2, s2);
	glTexCoord2f(0.01, 0.01);
	glVertex3f(s2, s2, s2);
	glEnd();

	// Top wall
	glBindTexture(GL_TEXTURE_2D, texturesAddOn[TID_SKYTOP]);
	glBegin(GL_QUADS);
	glColor3f(1.0f, 1.0f, 1.0f);
	glTexCoord2f(0.01, 0.99);
	glVertex3f(-s2, s2, -s2);
	glTexCoord2f(0.99, 0.99);
	glVertex3f(s2, s2, -s2);
	glTexCoord2f(1.0, 0.0);
	glVertex3f(s2, s2, s2);
	glTexCoord2f(0.01, 0.01);
	glVertex3f(-s2, s2, s2);
	glEnd();

	// Turn off texture mapping and enable lighting
	glEnable(GL_LIGHTING);
	glDisable(GL_TEXTURE_2D);
}

void DrawTerrain() {

	int MAP_SIZE = 10;
	int size = 80;

	int height[11][11] = {
		{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
		{ 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0 },
		{ 0, 1, 3, 2, 2, 4, 3, 2, 2, 0, 0 },
		{ 0, 1, 2, 2, 3, 5, 5, 4, 1, 0, 0 },
		{ 0, 1, 2, 2, 3, 4, 3, 4, 1, 0, 0 },
		{ 0, 1, 2, 3, 2, 3, 2, 2, 1, 0, 0 },
		{ 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0 },
		{ 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0 },
		{ 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0 },
		{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
		{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } };

	glDisable(GL_LIGHTING);
	glEnable(GL_TEXTURE_2D);

	for (int x = 0; x < MAP_SIZE; x++) {
		for (int z = 0; z < MAP_SIZE; z++) {
			if (height[x][z] > 1)
				glBindTexture(GL_TEXTURE_2D, texturesAddOn[TID_TERRAIN_1]);
			else if (height[x][z] == 1)
				glBindTexture(GL_TEXTURE_2D, texturesAddOn[TID_TERRAIN_2]);
			else if (height[x][z] == 0)
				glBindTexture(GL_TEXTURE_2D, texturesAddOn[TID_SHORE]);
			glBegin(GL_QUADS);
			glColor3f(1.0f, 1.0f, 1.0f);
			glTexCoord2f(0.0f, 0.0f);
			glVertex3f(x * size, height[x][z] * size, z * size);
			glTexCoord2f(1.0f, 0.0f);
			glVertex3f((x * size) + size, height[x + 1][z] * size, z * size);
			glTexCoord2f(1.0f, 1.0f);
			glVertex3f((x * size) + size, height[x + 1][z + 1] * size, (z * size) + size);
			glTexCoord2f(0.0f, 1.0f);
			glVertex3f(x * size, height[x][z + 1] * size, (z * size) + size);
			glEnd();
		}
	}

	glEnable(GL_LIGHTING);
	glDisable(GL_TEXTURE_2D);
}

void DrawWorldLight() {

	GLUquadricObj *quadric;
	quadric = gluNewQuadric();

	glMaterialfv(GL_FRONT, GL_EMISSION, BRIGHTLIGHT_COL);
	gluSphere(quadric, 10, 100, 100);
	glMaterialfv(GL_FRONT, GL_EMISSION, NO_LIGHT);
}