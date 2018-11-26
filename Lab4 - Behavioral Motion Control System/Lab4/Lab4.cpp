// Lab 4£ºBehavioral Motion Control System

#include "stdafx.h"
#include <assert.h>
#include <math.h>
#include <GL/glut.h>

int g_screenWidth = 0;
int g_screenHeight = 0;

//Points on spline
static int points = 0;    //index of points
static int numPoint = 5;     //number of points

static GLfloat t = 0;    //time

//The amount that time increases each time
GLfloat timeInterval = 0.1;

static int numUnits = 20;    //The number of units in the boids
GLfloat boidsM[20][16];    //Matrix of all units
static GLfloat M[16] = { 0 };    //Matrix of a single unit for multiplication
static GLfloat leaderM[16] = { 0 };    //Matrix of the leading unit

//Velocity calculated by 4 rules
GLfloat v_rule1[3] = {0};
GLfloat v_rule2[3] = {0};
GLfloat v_rule3[3] = {0};
GLfloat v_rule4[3] = {0};

//Matrix for storing the velocity and position of the boids
GLfloat vBoids[20][3] = {};
GLfloat posBoids[20][3] = {};

//The coordinate of units' generation points in the boids
GLfloat genPoints[20][3] = { { 3.0f, 5.5f, 0.7f }, 
							 { -9.0f, 7.5f, 0.9f }, 
							 { -4.0f, 6.7f, 0.8f }, 
							 { 4.5f, 6.8f, 0.7f }, 
							 { -3.0f, 9.1f, 0.3f }, 
							 { -5.0f, 10.3f, 0.0f }, 
							 { 4.0f, 7.7f, 0.2f }, 
							 { -4.0f, 9.6f, 0.4f }, 
							 { 0.0f, 5.2f, 0.1f }, 
							 { -1.0f, 3.6f, 0.4f } ,
							 { 2.0f, 7.5f, 0.8f }, 
							 { -10.0f, 6.6f, 1.0f }, 
							 { -5.0f, 8.2f, 0.9f }, 
							 { 3.5f, 6.5f, 0.8f }, 
							 { -4.0f, 10.1f, 0.2f }, 
							 { -6.0f, 8.8f, 0.1f }, 
							 { 3.0f, 12.0f, 0.3f }, 
							 { -5.0f, 13.0f, 0.5f }, 
							 { -1.0f, 8.4f, 0.2f }, 
							 { -2.0f, 4.6f, 0.5f } };

//7 control points of the leading unit in Quaternion
static GLfloat leaderCPQuat[7][7] = { { 0, 0, 0, 1, 5, -8, -10 },
									  { 0, 0, 1, 0, 3, -10, -5 },
									  { 0, 1, 0, 0, -3, -14, -5 },
									  { 1, 0, 0, 0, 8, 2, -20 },
									  { 0, 1, 0, 0, -8, -2, -20 },
									  { 0, 0, 1, 0, -5, -6, -10 },
									  { 1, 0, 0, 0, 1, -18, -3 } };

//M marix for Catmul-Rom Spline
static GLfloat CRSplineM[16] = { -0.5f, 1.0f, -0.5f, 0.0f,
								 1.5f, -2.5f, 0.0f, 1.0f,
								 -1.5f, 2.0f, 0.5f, 0.0f,
								 0.5f, -0.5f, 0.0f, 0.0f };

//M marix for B Spline
static GLfloat BSplineM[16] = { -1.0f / 6.0f, 3.0f / 6.0f, -3.0f / 6.0f, 1.0f / 6.0f,
								3.0f / 6.0f, -6.0f / 6.0f, 0.0f / 6.0f, 4.0f / 6.0f,
								-3.0f / 6.0f, 3.0f / 6.0f, 3.0f / 6.0f, 1.0f / 6.0f,
								1.0f / 6.0f, 0.0f / 6.0f, 0.0f / 6.0f, 0.0f / 6.0f };

//Blending Function: Q(t) = T * M * G
GLfloat blend(GLfloat T[4], GLfloat MS[16], GLfloat G[4]) {
	GLfloat B[4] = { 0 };	//B[4] = T * M
	B[0] = T[0] * MS[0] + T[1] * MS[1] + T[2] * MS[2] + T[3] * MS[3];
	B[1] = T[0] * MS[4] + T[1] * MS[5] + T[2] * MS[6] + T[3] * MS[7];
	B[2] = T[0] * MS[8] + T[1] * MS[9] + T[2] * MS[10] + T[3] * MS[11];
	B[3] = T[0] * MS[12] + T[1] * MS[13] + T[2] * MS[14] + T[3] * MS[15];
	GLfloat Qt = B[0] * G[0] + B[1] * G[1] + B[2] * G[2] + B[3] * G[3];
	return Qt;
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

//Timer: 16ms interval, about 60 FPS
void timer(int value) {
	glutPostRedisplay();

	t = t + 0.01;    //time increases by 0.01 each time
	if (t >= 1)	{
		t = 0;
		if (points < numPoint - 1) {
			points++;
		} else {
			points = 0;
		}
	}
	// reset timer
	glutTimerFunc(16, timer, 0);
}

//Rotation matrix in quaternion
void QuaternionRoatationM(GLfloat matQuat[7], GLfloat R[16]) {
	GLfloat w = matQuat[0];
	GLfloat x = matQuat[1];
	GLfloat y = matQuat[2];
	GLfloat z = matQuat[3];
	R[0] = 1.0f - 2.0f*y*y - 2.0f*z*z;    //1, 1
	R[1] = 2.0f*x*y + 2.0f*w*z;           //2, 1
	R[2] = 2.0f*x*z - 2.0f*w*y;		      //3, 1
	R[3] = 0.0f;					      //4, 1
	R[4] = 2.0f*x*y - 2.0f*w*z;		      //1, 2
	R[5] = 1.0f - 2.0f*x*x - 2.0f*z*z;    //2, 2
	R[6] = 2.0f*y*z + 2.0f*w*x;		      //3, 2
	R[7] = 0.0f;					      //4, 2
	R[8] = 2.0f*x*z + 2.0f*w*y;		      //1, 3
	R[9] = 2.0f*y*z - 2.0f*w*x;		      //2, 3
	R[10] = 1.0f - 2.0f*x*x - 2.0f*y*y;   //3, 3
	R[11] = 0.0f;					      //4, 3
	R[12] = matQuat[4];				      //1, 4
	R[13] = matQuat[5];			          //2, 4
	R[14] = matQuat[6];			          //3, 4
	R[15] = 1.0f;					      //4, 4
}

//Interpolate using quaternion to generate the movement of the leader with given quaterion, postion and type of spline
void q_interpolate(GLfloat pQuat[7][7], GLfloat SplineM[16]) {
	GLfloat TMatrix_q[4] = { t*t*t, t*t, t, 1 };
	GLfloat tempM[7];    //A temporary matrix to keep the track of interpolation

	//Generate the track of interpolation based on 4 points
	for (int i = 0; i < 7; i++)	{
		GLfloat GMatrix_q[4] = { pQuat[points][i],
			pQuat[(points + 1) % numPoint][i],
			pQuat[(points + 2) % numPoint][i],
			pQuat[(points + 3) % numPoint][i] };

		tempM[i] = blend(TMatrix_q, SplineM, GMatrix_q);
	}
	normalization(tempM);
	QuaternionRoatationM(tempM, leaderM);
}

//Initialization of 4 rules
void rule_1(int index);
void rule_2(int index);
void rule_3(int index);
void rule_4(int index);

//Store the coordinates of generation point into boidsM and posBoids
void initializeMat() {
	for (int j = 0; j < numUnits; j++) {
		boidsM[j][0] = 1.0f;
		boidsM[j][5] = 1.0f;
		boidsM[j][10] = 1.0f;
		for (int i = 0; i < 3; i++) {
			boidsM[j][12 + i] = genPoints[j][i];
			posBoids[j][i] = boidsM[j][12 + i];
		}
		boidsM[j][15] = 1.0f;
	}
}

//Distances between two balls
GLfloat getDistance(GLfloat V1[3], GLfloat V2[3]){
	GLfloat Distance = sqrt((V1[0] - V2[0])*(V1[0] - V2[0]) + (V1[1] - V2[1])*(V1[1] - V2[1]) + (V1[2] - V2[2])*(V1[2] - V2[2]));
	return Distance;
}

//4 rules to set the principle of boids' behavior
//Rule 1: follow - units in the boids follow their leader
void rule_1(int index) {
	for (int i = 0; i < 3; i++) 	{
		v_rule1[i] = (leaderM[12 + i] - boidsM[index][12 + i]) / 3000;
	}
}

//Rule 2: group up - a unit sticks together with the center of the rest boids
void rule_2(int index) {
	GLfloat all[3] = {};
	GLfloat center[3] = {};
	//Sum up the positions of the whole boids and pick the average values as the center's position
	for (int i = 0; i < numUnits; i++) {
		all[0] += boidsM[i][12];
		all[1] += boidsM[i][13];
		all[2] += boidsM[i][14];
		center[0] = (all[0] - boidsM[index][12]) / (numUnits - 1);
		center[1] = (all[1] - boidsM[index][13]) / (numUnits - 1);
		center[2] = (all[2] - boidsM[index][14]) / (numUnits - 1);
	}

	//Calculate the volecity
	for (int i = 0; i < 3; i++)	{
		v_rule2[i] = (center[i] - boidsM[index][12 + i]) / 2000;
	}
}

//Rule 3: avoid collision - the distances between a unit and other units are always larger than a specific value
void rule_3(int index) {
	for (int i = 0; i < numUnits; i++) {
		if (i != index)	{
			if (getDistance(posBoids[index], posBoids[i])< 4) {
				for (int j = 0; j < 3; j++)	{
					v_rule3[j] = (v_rule2[j] - (posBoids[i][j] - posBoids[index][j])) / 2000;
				}
			}
		}
	}
}

//Rule 4: parallel - a unit moves with the same velocity as the others
void rule_4(int index) {
	GLfloat all[3] = {};
	GLfloat average[3] = {};
	//Sum up the velocity of every unit in the boids, and pick the average value as the velocity of the whole boids
	for (int i = 0; i < numUnits; i++) {
		all[0] += vBoids[i][0];
		all[1] += vBoids[i][1];
		all[2] += vBoids[i][2];
		average[0] = (all[0] - vBoids[index][0]) / (numUnits - 1);
		average[1] = (all[1] - vBoids[index][1]) / (numUnits - 1);
		average[2] = (all[2] - vBoids[index][2]) / (numUnits - 1);
	}

	//Calculate the velocity
	for (int i = 0; i < 3; i++)	{
		v_rule4[i] = (average[i] - vBoids[index][i]) / 2000;
	}
}

//Generate the movement of boids
void boidsMove() {
	//Temporary vectors for keeping velocity generated by 4 rules
	GLfloat v1[3] = { 0 }; 
	GLfloat v2[3] = { 0 };
	GLfloat v3[3] = { 0 };
	GLfloat v4[3] = { 0 };

	//Calculate the velocity of each unit according to the 4 rules
	for (int i = 0; i < numUnits; i++) {
		rule_1(i);
		for (int j = 0; j < 3; j++)	{
			v1[j] = v_rule1[j];
		}
		rule_2(i);
		for (int j = 0; j < 3; j++)	{
			v2[j] = v_rule2[j];
		}
		rule_3(i);
		for (int j = 0; j < 3; j++)	{
			v3[j] = v_rule3[j];
		}
		rule_4(i);
		for (int j = 0; j < 3; j++)	{
			v4[j] = v_rule4[j];
		}

		//Sum up the velocity generated and update the position in boidsM
		for (int j = 0; j < 3; j++)	{
			vBoids[i][j] = vBoids[i][j] + v1[j] + v2[j] + v3[j] + v4[j];
			posBoids[i][j] = posBoids[i][j] + vBoids[i][j] * 0.15;
			boidsM[i][12 + j] = posBoids[i][j];
		}
	}
}

//Animation
void animateUnit(int index) {
	glPushMatrix();
	for (int j = 0; j < 16; j++) {
		M[j] = boidsM[index][j];
	}
	glMultMatrixf(M);
	glutSolidSphere(0.3,20,20);
	glPopMatrix();
}

void animateLeader()
{
	glPushMatrix();
	q_interpolate(leaderCPQuat, BSplineM);
	glMultMatrixf(leaderM);
	//glutSolidDodecahedron();
	glutSolidIcosahedron();
	glPopMatrix();
}

void animateBoids() {    //including every units in the boids
	animateLeader();
	boidsMove();
	for (int i = 0; i < numUnits; i++) {
		animateUnit(i);
	}
}


void render(void) {
	//Clear buffer
	glClearColor(0.0, 0.0, 0.0, 1.0);
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
	gluLookAt(0, 15.0, 15.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

	//Animation	
	animateBoids();

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
	gluPerspective(65.0, (GLfloat)w / (GLfloat)h, 1.0, 50.0);
}


int main(int argc, char** argv) {
	//OpenGL window
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(800, 600);
	glutInitWindowPosition(100, 100);
	glutCreateWindow("Animation Lab 4 - Behavioral Motion Control System - Xie Wu");

	//Initialization
	initializeMat();

	//Callback functions
	glutDisplayFunc(render);
	glutReshapeFunc(reshape);
	glutTimerFunc(16, timer, 0);

	//Main loop
	glutMainLoop();

	return 0;
}