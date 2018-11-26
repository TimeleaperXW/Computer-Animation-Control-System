//Lab 2£ºHierarchical Object Motion Control System

#include "stdafx.h"
#include <assert.h>
#include <math.h>
#include <GL/glut.h>

//screen size
int g_screenWidth = 0;
int g_screenHeight = 0;

//points on Spline
static int points = 0;    //index of points
static int number = 7;    //number of points 

static GLfloat t = 0;    //time

//vectors for computing tangent orientation
static GLfloat tangent[3] = { 0 };
static GLfloat binormal[3] = { 0 };
static GLfloat normal[3] = { 0 };
static GLfloat loopIndex = 0;

//The M matrix for torso, Left and Right Leg
static GLfloat M[16] = { 0 };     //torso
static GLfloat tempM[3] = {0};    //temporary matrix to keep the interpolated track of torso's movement
static GLfloat M1[16] = { 0 };    //left leg
static GLfloat M2[16] = { 0 };    //right leg


//M Marix for Catmull-Rom Spline
static GLfloat CRSplineM[16] = { -0.5f, 1.0f, -0.5f, 0.0f,
								1.5f, -2.5f, 0.0f, 1.0f,
								-1.5f, 2.0f, 0.5f, 0.0f,
								0.5f, -0.5f, 0.0f, 0.0f };

//M Marix for B Spline
static GLfloat BSplineM[16] = { -1.0f / 6.0f, 3.0f / 6.0f, -3.0f / 6.0f, 1.0f / 6.0f,
								3.0f / 6.0f, -6.0f / 6.0f, 0.0f / 6.0f, 4.0f / 6.0f,
								-3.0f / 6.0f, 3.0f / 6.0f, 3.0f / 6.0f, 1.0f / 6.0f,
								1.0f / 6.0f, 0.0f / 6.0f, 0.0f / 6.0f, 0.0f / 6.0f };

//7 points presented in world coordinate system
static GLfloat point[7][3] = { { 12, 0, -20 },
							   { 6, 0, -20 },
							   { -1, 0, -10 },
							   { -5, 0, -10 },
							   { -1, 0, -5 },
							   { 2, 0, -5 },
							   { 3, 0, -3 } };

//Blending Function: Q(t) = T*M*G
GLfloat blend(GLfloat T[4], GLfloat MS[16], GLfloat G[4]) {
	GLfloat B[4] = { 0 };    //B[4] = T*M
	B[0] = T[0] * MS[0] + T[1] * MS[1] + T[2] * MS[2] + T[3] * MS[3];
	B[1] = T[0] * MS[4] + T[1] * MS[5] + T[2] * MS[6] + T[3] * MS[7];
	B[2] = T[0] * MS[8] + T[1] * MS[9] + T[2] * MS[10] + T[3] * MS[11];
	B[3] = T[0] * MS[12] + T[1] * MS[13] + T[2] * MS[14] + T[3] * MS[15];
	GLfloat Qt = B[0] * G[0] + B[1] * G[1] + B[2] * G[2] + B[3] * G[3];
	return Qt;
}

//4 x 4 Matrix Multiplication
void matMult4x4(GLfloat M1[16], GLfloat M2[16], GLfloat MResult[16]) {
	MResult[0] = M1[0] * M2[0] + M1[4] * M2[1] + M1[8] * M2[2] + M1[12] * M2[3];
	MResult[1] = M1[1] * M2[0] + M1[5] * M2[1] + M1[9] * M2[2] + M1[13] * M2[3];
	MResult[2] = M1[2] * M2[0] + M1[6] * M2[1] + M1[10] * M2[2] + M1[14] * M2[3];
	MResult[3] = M1[3] * M2[0] + M1[7] * M2[1] + M1[11] * M2[2] + M1[15] * M2[3];
	MResult[4] = M1[0] * M2[4] + M1[4] * M2[5] + M1[8] * M2[6] + M1[12] * M2[7];
	MResult[5] = M1[1] * M2[4] + M1[5] * M2[5] + M1[9] * M2[6] + M1[13] * M2[7];
	MResult[6] = M1[2] * M2[4] + M1[6] * M2[5] + M1[10] * M2[6] + M1[14] * M2[7];
	MResult[7] = M1[3] * M2[4] + M1[7] * M2[5] + M1[11] * M2[6] + M1[15] * M2[7];
	MResult[8] = M1[0] * M2[8] + M1[4] * M2[9] + M1[8] * M2[10] + M1[12] * M2[11];
	MResult[9] = M1[1] * M2[8] + M1[5] * M2[9] + M1[9] * M2[10] + M1[13] * M2[11];
	MResult[10] = M1[2] * M2[8] + M1[6] * M2[9] + M1[10] * M2[10] + M1[14] * M2[11];
	MResult[11] = M1[3] * M2[8] + M1[7] * M2[9] + M1[11] * M2[10] + M1[15] * M2[11];
	MResult[12] = M1[0] * M2[12] + M1[4] * M2[13] + M1[8] * M2[14] + M1[12] * M2[15];
	MResult[13] = M1[1] * M2[12] + M1[5] * M2[13] + M1[9] * M2[14] + M1[13] * M2[15];
	MResult[14] = M1[2] * M2[12] + M1[6] * M2[13] + M1[10] * M2[14] + M1[14] * M2[15];
	MResult[15] = M1[3] * M2[12] + M1[7] * M2[13] + M1[11] * M2[14] + M1[15] * M2[15];
}

//Vector normalization
void normalization(GLfloat normalVec[3]) {
	GLfloat squareQuat = normalVec[0] * normalVec[0] + normalVec[1] * normalVec[1] + normalVec[2] * normalVec[2];
	if (squareQuat != 0) {
		GLfloat baseQuat = sqrt(squareQuat);
		normalVec[0] = normalVec[0] / baseQuat;
		normalVec[1] = normalVec[1] / baseQuat;
		normalVec[2] = normalVec[2] / baseQuat;
	}
}

//Vector multiplication - cross product
void VectorMult(GLfloat V1[3], GLfloat V2[3], GLfloat VResult[3]) {
	VResult[0] = V1[1] * V2[2] - V1[2] * V2[1];
	VResult[1] = V1[2] * V2[0] - V1[0] * V2[2];
	VResult[2] = V1[0] * V2[1] - V1[1] * V2[0];
}

//Timer: 16ms interval, about 60 FPS
void timer(int value) {
	glutPostRedisplay();

	//As time increases by 0.005, the value of points changes from 0 to 2
	t = t + 0.005;
	if (t >= 1) {
		t = 0;
		if (points < number - 1) {
			points++;
		}
		else {
			points = 0;
		}
	}
	//Reset timer
	glutTimerFunc(16, timer, 0);
}

//Interpolate the track of torso with given coordinates and spline type
void interpolate(GLfloat pCoord[7][3], GLfloat SplineM[16]) {
	GLfloat TMatrix[4] = { t*t*t, t*t, t, 1 };
	GLfloat TangentTMatrix[4] = { 3*t*t, 2*t, 1, 0 };

	//Interpolation of track
	for (int i = 0; i < 3; i++)	{
		GLfloat GMatrix[4] = { pCoord[points][i],
			pCoord[(points + 1) % number][i],
			pCoord[(points + 2) % number][i],
			pCoord[(points + 3) % number][i] };
		tempM[i] = blend(TMatrix, SplineM, GMatrix);
		tangent[i] = blend(TangentTMatrix, SplineM, GMatrix);
	}

	//Tangent vector
	normalization(tangent);

	if (points == 0 && loopIndex == 0) {    //from the start
		GLfloat TempVector[3] = { 1, 0, 0 };    //A random vector
		normalization(TempVector);
		VectorMult(tangent, TempVector, normal);
		normalization(normal);
		VectorMult(normal, tangent, binormal);
		normalization(binormal);
		loopIndex++;
	} else {     //not from the start
		VectorMult(tangent, binormal, normal);
		normalization(normal);
		VectorMult(normal, tangent, binormal);
		normalization(binormal);
	}

	//Interpolation Matrix M
	M[0] = tangent[0];     //1, 1
	M[1] = normal[0];      //2, 1
	M[2] = binormal[0];    //3, 1
	M[3] = 0;			   //4, 1
	M[4] = tangent[1];     //1, 2
	M[5] = normal[1];      //2, 2
	M[6] = binormal[1];    //3, 2
	M[7] = 0;			   //4, 2
	M[8] = tangent[2];     //1, 3
	M[9] = normal[2];      //2, 3
	M[10] = binormal[2];   //3, 3
	M[11] = 0;			   //4, 3
	M[12] = tempM[0];      //1, 4
	M[13] = tempM[1];      //2, 4
	M[14] = tempM[2];      //3, 4
	M[15] = 1;			   //4, 4
}

//Animation of torso - moving along given track and facing direction
void animateTorso() {
	interpolate(point, CRSplineM);
	glLoadMatrixf(M);
	glutSolidCube(1.0);
}

//Animation of legs: hierarchical object motion following the order of Translation, Rotation, Translation 
void animateLeft() {    //left leg
	//Translation 1
	GLfloat LT1[16] = { 1, 0, 0, 0,
						0, 1, 0, 0,
						0, 0, 1, 0,
						0, -1, 0, 1 };

	//Rotation
	GLfloat LAngle = (sin(4 * 3.14*t - 3.14 / 2)*3.14) / 4;
	GLfloat LT2[16] = { cos(LAngle), sin(LAngle), 0, 0,
						-sin(LAngle), cos(LAngle), 0, 0,
						0, 0, 1, 0,
						0, 0, 0, 1 };

	//Translation 2
	GLfloat LT3[16] = { 1, 0, 0, 0,
						0, 1, 0, 0,
						0, 0, 1, 0,
						0, 0, 0.3, 1 };

	//Concatenated transformation of one object in another object's coordinate system
	matMult4x4(M, LT2, M1);
	matMult4x4(M1, LT1, M1);
	matMult4x4(M1, LT3, M1);

	//Display
	glLoadMatrixf(M1);
	glScalef(0.3f, 2.0f, 0.3f);
	glutSolidCube(1.0);
}

void animateRight() {    //right leg
	//Translation 1
	GLfloat RT1[16] = { 1, 0, 0, 0,
						0, 1, 0, 0,
						0, 0, 1, 0,
						0, -1, 0, 1 };

	//Rotation by Z axis
	GLfloat LAngle = (sin(4*3.14*t-3.14/2)*3.14)/4;
	GLfloat RT2[16] = { cos(-LAngle), sin(-LAngle), 0, 0,
						-sin(-LAngle), cos(-LAngle), 0, 0,
						0, 0, 1, 0,
						0, 0, 0, 1 };

	//Translation 2
	GLfloat RT3[16] = { 1, 0, 0, 0,
						0, 1, 0, 0,
						0, 0, 1, 0,
						0, 0, -0.3, 1 };

	//Concatenated transformation
	matMult4x4(M, RT2, M2);
	matMult4x4(M2, RT1, M2);
	matMult4x4(M2, RT3, M2);

	//Display
	glLoadMatrixf(M2);
	glScalef(0.3f, 2.0f, 0.3f);
	glutSolidCube(1.0);
}


void render(void) {
	//Clear buffer
	glClearColor(0.0, 0.0, 0.0, 0.0);
	glClearDepth(1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//Render state
	glEnable(GL_DEPTH_TEST);
	glShadeModel(GL_SMOOTH);

	//Lighting
	GLfloat LightAmbient[] = { 0.7f, 0.7f, 0.7f, 1.0f };
	GLfloat LightDiffuse[] = { 0.4f, 0.4f, 0.4f, 1.0f };
	GLfloat LightSpecular[] = { 0.5f, 0.5f, 0.5f, 1.0f };
	GLfloat LightPosition[] = { 2.0f, 2.0f, 2.0f, 1.0f };
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glLightfv(GL_LIGHT0, GL_AMBIENT, LightAmbient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, LightDiffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, LightSpecular);
	glLightfv(GL_LIGHT0, GL_POSITION, LightPosition);

	//Surface material
	GLfloat material_Ka[] = { 0.1f, 0.1f, 0.0f, 1.0f };
	GLfloat material_Kd[] = { 0.5f, 0.5f, 0.75f, 1.0f };
	GLfloat material_Ks[] = { 0.67f, 0.5f, 0.67f, 1.0f };
	GLfloat material_Ke[] = { 0.1f, 0.2f, 0.5f, 1.0f };
	GLfloat material_Se = 3;
	glMaterialfv(GL_FRONT, GL_AMBIENT, material_Ka);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, material_Kd);
	glMaterialfv(GL_FRONT, GL_SPECULAR, material_Ks);
	glMaterialfv(GL_FRONT, GL_EMISSION, material_Ke);
	glMaterialf(GL_FRONT, GL_SHININESS, material_Se);

	//Modelview matrix
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	//Animation
	animateTorso();
	animateLeft();
	animateRight();

	//Disable lighting
	glDisable(GL_LIGHT0);
	glDisable(GL_LIGHTING);

	//Swap back and front buffers
	glutSwapBuffers();
}

//Update the viewport and projection matrix when the size of window is changed
void reshape(int w, int h) {
	g_screenWidth = w;
	g_screenHeight = h;
	glViewport(0, 0, (GLsizei)w, (GLsizei)h);

	//Projection matrix
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(70.0, (GLfloat)w / (GLfloat)h, 1.0, 30.0);
}

int main(int argc, char** argv) {
	//Create GL window
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(800, 600);
	glutInitWindowPosition(100, 100);
	glutCreateWindow("Animation Lab 2 - Hierarchical Object Motion Control System - Xie Wu");


	//Callback functions
	glutDisplayFunc(render);
	glutReshapeFunc(reshape);
	glutTimerFunc(16, timer, 0);

	//Main loop
	glutMainLoop();

	return 0;
}