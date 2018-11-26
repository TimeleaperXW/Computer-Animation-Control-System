//Lab 3£ºPhysics-Based Motion Control System

#include "stdafx.h"
#include <assert.h>
#include <math.h>
#include <GL/glut.h>

#define pi = 3.14159265358979

//Screen size
int g_screenWidth = 0;
int g_screenHeight = 0;

//Number of balls
static int number = 20;

//Coordinates of balls' generation point
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

//Original velocity of balls
GLfloat velocity[20][3] = { { 0.7, 0, 0 }, 
							{ -1, 0, 0 },
							{ -0.6, 0, 0 },
							{ 0.6, 0, 0 },
							{ -0.8, 0, 0 }, 
							{ 1.6, 0, 0 }, 
							{ -1, 0, 0 }, 
							{ 0.5, 0, 0 }, 
							{ -1, 0, 0 }, 
							{ 0.7, 0, 0 },
							{ 0.6, 0, 0 }, 
							{ -1, 0, 0 },
							{ -1, 0, 0 },
							{ 0.3, 0, 0 },
							{ -1, 0, 0 }, 
							{ 0.4, 0, 0 }, 
							{ -1, 0, 0 }, 
							{ 0.6, 0, 0 }, 
							{ -1, 0, 0 }, 
							{ 0.7, 0, 0 } };

static GLfloat M[16] = { 0 };    //Matrix of a single ball for multiplication
GLfloat ballMatAll[20][16];    //Matrix of all balls

GLfloat velocityNew[20][3] = {0};    //Store balls' volecity as time increases
GLfloat posOrigin[20][3] = { 0 };    //Store balls' original positions
GLfloat posNew[20][3];    //Store balls' positions as time increases
GLfloat timeInterval = 0.03f;    //The amount that time increases each time
GLfloat acceleration[3] = { 0, -5.0, 0 };    //Gravity acceleration: y velocity increases by -2.0 as each time interval passes
GLfloat e = 0.8f;    //Coefficient of collision: the value of velocity remains 80% after collision

// Vector normalization
void normalization(GLfloat normalVec[3]) {
	GLfloat squareQuat = normalVec[0] * normalVec[0] + normalVec[1] * normalVec[1] + normalVec[2] * normalVec[2];
	if (squareQuat != 0) {
		GLfloat baseQuat = sqrt(squareQuat);
		normalVec[0] = normalVec[0] / baseQuat;
		normalVec[1] = normalVec[1] / baseQuat;
		normalVec[2] = normalVec[2] / baseQuat;
	}
}

// Vector multiplication - dot product
GLfloat VectorDotMult(GLfloat V1[3], GLfloat V2[3]) {
	GLfloat VResult = V1[0] * V2[0] + V1[1] * V2[1] + V1[2] * V2[2];
	return VResult;
}

//Distance between two balls
GLfloat distance(GLfloat V1[3], GLfloat V2[3]) {
	GLfloat distance = sqrt((V1[0] - V2[0]) * (V1[0] - V2[0]) + (V1[1] - V2[1]) * (V1[1] - V2[1]) + (V1[2] - V2[2]) * (V1[2] - V2[2]));
	return distance;
}

//Store the origninal position of balls into ballMatAll and posOrigin
void matInitialize() {
	for (int j = 0; j < number; j++){
		ballMatAll[j][0] = 1.0f;
		ballMatAll[j][5] = 1.0f;
		ballMatAll[j][10] = 1.0f;
		for (int i = 0; i < 3; i++){
			ballMatAll[j][12 + i] = genPoints[j][i];
			posOrigin[j][i] = ballMatAll[j][12 + i];
		}
		ballMatAll[j][15] = 1.0f;
	}
}

//Timer: 16ms interval, about 60 FPS
void timer(int value) {
	glutPostRedisplay();

	//Reset timer
	glutTimerFunc(16, timer, 0);
}

/* Collision detection between balls: collision takes place when the distance between two balls is less than 1 (the diameter of a ball)
* Suppose the staight line connecting two balls is x_axis, and the line perpendicular to x_axis is y_aixs
* The direction of velocity of two balls along x_axis turns around once a collision takes place, while the direction of velocity along y_axis remains unchanged */
void ballCollision(int index) {
	//Calculate the distance between a selected ball and every other ball
	for (int i = index + 1; i < number; i++) {
		if (distance(posOrigin[index], posOrigin[i])<1.0) {
			//Calculate the x_axis for the selected ball
			GLfloat x_axis[3];
			for (int j = 0; j < 3; j++)	{
				x_axis[j] = posOrigin[i][j] - posOrigin[index][j];
			}
			normalization(x_axis);
			//Calculate the velocity of ball 1 along x_axis and y_aixs
			GLfloat u1x[3], u1y[3];
			GLfloat tempU1 = VectorDotMult(x_axis, velocity[index]);
			for (int j = 0; j < 3; j++)
			{
				u1x[j] = tempU1*x_axis[j];
				u1y[j] = velocity[index][j] - u1x[j];
			}

			//Calculate the x_axis for the other ball
			for (int j = 0; j < 3; j++)
			{
				x_axis[j] = posOrigin[index][j] - posOrigin[i][j];
			}
			normalization(x_axis);
			//Calculate the velocity of ball 2 along x_axis and y_axis
			GLfloat u2x[3], u2y[3];
			GLfloat tempU2 = VectorDotMult(x_axis, velocity[i]);
			for (int j = 0; j < 3; j++)	{
				u2x[j] = tempU2*x_axis[j];
				u2y[j] = velocity[i][j] - u2x[j];
			}

			//Calculate the velocity
			GLfloat v1x[3], v2x[3];
			for (int j = 0; j < 3; j++)	{
				v1x[j] = (u1x[j] + u2x[j] - (u1x[j] - u2x[j]))*0.5;
				v2x[j] = (u1x[j] + u2x[j] - (u2x[j] - u1x[j]))*0.5;
				velocity[index][j] = v1x[j] + u1y[j];
				velocity[i][j] = v2x[j] + u2y[j];
			}
			continue;
		}
	}
}

//Collision detection between a ball and the floor: collision takes place when the distance between a ball and the floor is less than 0.5 (the radiant of a ball)
void floorCollision(int index) {
	if (posOrigin[index][1]<0.5) { 
	//The direction of velocity along Y axis turns around and the value is decreased by the coefficient of collision (e)
	velocity[index][1] = -e*velocity[index][1]; 
	//The velocity along X and Z axises remains unchanged
	velocity[index][0] = velocity[index][0];
	velocity[index][2] = velocity[index][2];
	}
}

//Calculate the movement of the selected ball after collision (if there is)
void ballMove(int index){
	floorCollision(index);
	ballCollision(index);
	for (int i = 0; i<3; i++){
		velocityNew[index][i] = velocity[index][i] + acceleration[i] * timeInterval;
		velocity[index][i] = velocityNew[index][i];
		posNew[index][i] = posOrigin[index][i] + velocityNew[index][i] * timeInterval;
		posOrigin[index][i] = posNew[index][i];
		ballMatAll[index][12 + i] = posNew[index][i];
	}
}

//Animation of the balls
void animateBall() {
	for (int i = 0; i < number; i++) {
		glPushMatrix();
		ballMove(i);
		for (int j = 0; j < 16; j++) {
			M[j] = ballMatAll[i][j];
		}
		glMultMatrixf(M);
		glutSolidSphere(0.5, 20, 20);
		glPopMatrix();
	}
}

//Animation of the ground
void animateGround() {
	glBegin(GL_LINES);
	for (GLfloat x = -100; x < 100; x += 5.0f) {
		glVertex3f(x, 0, -100); glVertex3f(x, 0, 100);
	}
	for (GLfloat z = -150; z < 100; z += 5.0f) {
		glVertex3f(-150, 0, z); glVertex3f(100, 0, z);
	}
	glEnd();
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
	gluLookAt(0, 15.0, 15.0,   0.0, 0.0, 0.0,      0.0, 1.0, 0.0);

	//Animation	
	animateGround();
	animateBall();

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
	gluPerspective(65.0, (GLfloat)w / (GLfloat)h, 1.0, 30.0);
}


int main(int argc, char** argv) {
	//OpenGL window
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(800, 600);
	glutInitWindowPosition(100, 100);
	glutCreateWindow("Animation Lab 3 - Physics-Based Motion Control System - Xie Wu");

	//Initialization
	matInitialize();

	//Callback functions
	glutDisplayFunc(render);
	glutReshapeFunc(reshape);
	glutTimerFunc(16, timer, 0);

	//Main loop
	glutMainLoop();

	return 0;
}