//Lab1 - Basic Key Framing Motion Control System

#include "stdafx.h"
#include <assert.h>
#include <math.h>
#include <GL/glut.h>

//screen size
int g_screenWidth = 0;
int g_screenHeight = 0;

//number of points on spline
static int points = 0;   //index of points
static int number = 7;   //number of points 

static GLfloat t = 0;    //time

//M matrix for glLoadMatrixf()
static GLfloat M[16] = {0};

//M Marix for Catmul-Rom Spline
static GLfloat CRSplineM[16] = {-0.5f, 1.0f, -0.5f, 0.0f,
								1.5f, -2.5f, 0.0f, 1.0f,
								-1.5f, 2.0f, 0.5f, 0.0f,
								0.5f, -0.5f, 0.0f, 0.0f};

//M Marix for B Spline
static GLfloat BSplineM[16]= {-1.0f/6.0f, 3.0f/6.0f, -3.0f/6.0f, 1.0f/6.0f,
							  3.0f/6.0f, -6.0f/6.0f, 0.0f/6.0f, 4.0f/6.0f,
							  -3.0f/6.0f, 3.0f/6.0f, 3.0f/6.0f, 1.0f/6.0f,
							  1.0f/6.0f, 0.0f/6.0f, 0.0f /6.0f, 0.0f/6.0f};

//7 points in Quternion. The 6 numbers are w, x, y, z values in quaternion and in world Cartisian System successively
static GLfloat point_quaternion[7][7] = { { 0, 0, 0, 1, 0, -5, -20 },
										  { 0, 0, 1, 0, -1, 1, -15 },
										  { 0, 1, 0, 0, -3, 3, -10 },
										  { 1, 0, 0, 0, -5, 0, -5 },
										  { 1, 0, 0, 0, 5, 0, -5 },
										  { 0, 1, 0, 0, 3, 3, -10 },
										  { 0, 0, 1, 0, 1, 1, -15 } };

//7 Pionts in Euler Angle. The 6 numbers are: x_angle, y_angle, z_angle in Euler angle, x, y, z in world Cartisian System succesively
static GLfloat point_euler[7][6] = { { 30, 60, 105, 0, -5, -20 },
									 { 50, 40, 85, -1, 1, -15 },
									 { 70, 20, 65, -3, 3, -10 },
									 { 90, 0, 45, -5, 0, -5 },
									 { 90, 0, 45, 5, 0, -5 },
									 { 70, 20, 65, 3, 3, -10 },
									 { 50, 40, 85, 1, 1, -15 } };

//Blending Function Q(t) = T*M*G
GLfloat blend(GLfloat T[4], GLfloat MS[16], GLfloat G[4]) {
	GLfloat B[4] = { 0 };    //B[4] = T*M
	B[0] = T[0] * MS[0] + T[1] * MS[1] + T[2] * MS[2] + T[3] * MS[3];
	B[1] = T[0] * MS[4] + T[1] * MS[5] + T[2] * MS[6] + T[3] * MS[7];
	B[2] = T[0] * MS[8] + T[1] * MS[9] + T[2] * MS[10] + T[3] * MS[11];
	B[3] = T[0] * MS[12] + T[1] * MS[13] + T[2] * MS[14] + T[3] * MS[15];
	GLfloat Qt = B[0] * G[0] + B[1] * G[1] + B[2] * G[2] + B[3] * G[3];
	return Qt;
}

//Generate unit quaternion with a given quaternion array
void normalization(GLfloat normalMat[7]) {
	GLfloat squareQuat = normalMat[0] * normalMat[0] + normalMat[1] * normalMat[1] + normalMat[2] * normalMat[2] + normalMat[3] * normalMat[3];
		if (squareQuat != 0) {
			GLfloat baseQuat = sqrt(squareQuat);
			normalMat[0] = normalMat[0] / baseQuat;
			normalMat[1] = normalMat[1] / baseQuat;
			normalMat[2] = normalMat[2] / baseQuat;
			normalMat[3] = normalMat[3] / baseQuat;
		}
}

//Rotation using Quaternions
void rotMatQ(GLfloat tempMatQ[7], GLfloat R[16]) {
	GLfloat w = tempMatQ[0];
	GLfloat x = tempMatQ[1];
	GLfloat y = tempMatQ[2];
	GLfloat z = tempMatQ[3];
	R[0] = 1.0f - 2.0f*y*y - 2.0f*z*z;     //1,1
	R[1] = 2.0f*x*y + 2.0f*w*z;            //2,1
	R[2] = 2.0f*x*z - 2.0f*w*y;		       //3,1
	R[3] = 0.0f;					       //4,1
	R[4] = 2.0f*x*y - 2.0f*w*z;		       //1,2
	R[5] = 1.0f - 2.0f*x*x - 2.0f*z*z;     //2,2
	R[6] = 2.0f*y*z + 2.0f*w*x;		       //3,2
	R[7] = 0.0f;					       //4,2
	R[8] = 2.0f*x*z + 2.0f*w*y;		       //1,3
	R[9] = 2.0f*y*z - 2.0f*w*x;		       //2,3
	R[10] = 1.0f - 2.0f*x*x - 2.0f*y*y;    //3,3
	R[11] = 0.0f;					       //4,3
	R[12] = tempMatQ[4];				   //1,4
	R[13] = tempMatQ[5];			       //2,4
	R[14] = tempMatQ[6];			       //3,4
	R[15] = 1.0f;					       //4,4
}

//Rotation using Euler Angles
void EulerToQuaternion(GLfloat tempMatE[7]) {
	GLfloat a = tempMatE[0] / 2;
	GLfloat b = tempMatE[1] / 2;
	GLfloat c = tempMatE[2] / 2;

	//Transformation from Euler Angle to Quaternion
	tempMatE[6] = tempMatE[5];
	tempMatE[5] = tempMatE[4];
	tempMatE[4] = tempMatE[3];
	tempMatE[0] = cos(c)*cos(b)*cos(c) + sin(c)*sin(b)*sin(a);    //w
	tempMatE[1] = sin(c)*cos(b)*cos(c) - cos(c)*sin(b)*sin(a);    //x
	tempMatE[2] = cos(c)*sin(b)*cos(c) + sin(c)*cos(b)*sin(a);    //y
	tempMatE[3] = cos(c)*cos(b)*sin(c) - sin(c)*sin(b)*cos(a);    //z
}

//Timer: 16ms interval, about 60 FPS
void timer(int value) {
	glutPostRedisplay();

	//As time increases by 0.01, the value of points changes from 0 to 2
	t = t + 0.01;
	if (t >= 1) {
		t = 0;
		if (points < number - 4) {
			points++;
		}
		else {
			points = 0;
		}
	}
	//Reset timer
	glutTimerFunc(16, timer, 0);
}

//Interpolate using Quaternions
void interpQ(GLfloat pQuat[6][7], GLfloat SplineM[16]) {    //Quaternion, type of spline
	GLfloat TMatrix_q[4] = {t*t*t, t*t, t, 1};
	GLfloat tempM[7];    //A temporary matrix to keep the track of interpolation

	//Generate the track of interpolation based on 4 points
	for (int i = 0; i < 7; i++) {
		GLfloat GMatrix_q[4] = {pQuat[points][i],
								pQuat[(points + 1)][i],
								pQuat[(points + 2)][i],
								pQuat[(points + 3)][i]};

		tempM[i] = blend(TMatrix_q, SplineM, GMatrix_q);
	}
	normalization(tempM);
	rotMatQ(tempM,M);
}

//Interpolate using Euler Angle
void interpE(GLfloat p_euler[7][6], GLfloat SplineM[16]) {    //Euler Angle, type of spline
	GLfloat TMatrix_e[4] = {t*t*t, t*t, t, 1};
	GLfloat tempM[7];
	for (int i = 0; i < 7; i++)	{
		GLfloat GMatrix_e[4] = {p_euler[points][i],
								p_euler[(points + 1)][i],
								p_euler[(points + 2)][i],
								p_euler[(points + 3)][i]};
		tempM[i] = blend(TMatrix_e, SplineM, GMatrix_e);
	}
	EulerToQuaternion(tempM);
	normalization(tempM);
	rotMatQ(tempM,M);
}

//Generate a teapot animation using 1 of the 4 options: Quaternion with Catmull-Rom Spline, Quaternion with B Spline, Euler Angle with Catmull-Rom Spline, Euler Angle with B Spline.
void Animate() {
	interpQ(point_quaternion, CRSplineM);
	//interpQ(point_quaternion,BSplineM);
	//interpE(point_euler, CRSplineM);
	//interpE(point_euler, BSplineM);
	glLoadMatrixf(M);
	glutSolidTeapot(1.0);

}

//Rendering
void render(void) {
	//clear buffer
	glClearColor(0.0, 0.0, 0.0, 1.0);
	glClearDepth(1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//render state
	glEnable(GL_DEPTH_TEST);
	glShadeModel(GL_SMOOTH);

	//lighting
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

	//attributes of the surface material 
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

	//modelview matrix
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	
	//animation
	Animate();	

	//disable lighting
	glDisable(GL_LIGHT0);
	glDisable(GL_LIGHTING);

	//swap back and front buffers
	glutSwapBuffers();
}

//Update the viewport and projection matrix when the size of window is changed
void reshape(int w, int h) {
	g_screenWidth = w;
	g_screenHeight = h;
	glViewport(0, 0, (GLsizei)w, (GLsizei)h);

	//projection matrix
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(95.0, (GLfloat)w / (GLfloat)h, 1.0, 2000.0);
}

int main(int argc, char** argv) {
	//create OpenGL window
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(1000, 800);
	glutInitWindowPosition(100, 100);
	glutCreateWindow("Animation Lab 1 - Basic Key Framing By Xie Wu");

	//callback functions
	glutDisplayFunc(render);
	glutReshapeFunc(reshape);
	glutTimerFunc(16, timer, 0);

	//main loop
	glutMainLoop();

	return 0;
}