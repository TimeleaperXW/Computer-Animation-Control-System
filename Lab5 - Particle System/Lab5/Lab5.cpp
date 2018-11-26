//Lab 5£ºFinal Project - Particle System

#include "stdafx.h"
#include <GL/glut.h>
#include <olectl.h>
#include "Texture.h"

#define MAX_PARTICLE_NUM 1000    //Set the maximum number of particles to 1000

GLuint texture[2];     //Stores the texture of particles
GLuint listname = 1;     //Stores the background

//Structure for particles
typedef struct {
	//Coordinate
	GLfloat x;
	GLfloat y;
	GLfloat z;
	//Velocity
	GLfloat vx;
	GLfloat vy;
	GLfloat vz;
	//Acceleration
	GLfloat ax;
	GLfloat ay;
	GLfloat az;
	//Lifespan
	GLfloat life;
	//Size
	GLfloat size;
}particle;
particle particles[MAX_PARTICLE_NUM];

//Attributes of the first generation of particles (presented as small light balls)
void particleFirst() {
	for (int i = 0; i < MAX_PARTICLE_NUM; i++) {
		particles[i].life = rand() % 450;
		particles[i].size = rand() % 5 + 1;
		particles[i].x = float(rand() % 500 -250);
		particles[i].y = float(rand() % 100 -250);
		particles[i].z = float(rand() % 180 - 90.0);
		particles[i].vx = float(rand() % 10 - 4) / 50; 
		particles[i].vy = float(rand() % 10) / 40;
		particles[i].vz = float(rand() % 10 - 4) / 50;
		particles[i].ax = (rand() % 3 + 1) / 10000;
		particles[i].ay = 3 / 10000;
		particles[i].az = (rand() % 3 + 1) / 10000;
	}
}

//Initialization. Small squares are created to carry the particles.
void initLight() {
	//Clear the background
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
	glShadeModel(GL_SMOOTH);
	glEnable(GL_DEPTH_TEST);

	//Color of squares
	glColor4f(1.0, 1.0, 1.0, 1.0f);

	glBlendFunc(GL_SRC_ALPHA, GL_ONE);

	particleFirst();

	//Load background and texture images into the texture array
	LoadTexture("Black.jpg", texture[0]);
	LoadTexture("Light.bmp", texture[1]);

	//Create a list to store the texture
	glNewList(listname, GL_COMPILE);
	glEnable(GL_TEXTURE_2D);
	glDisable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBindTexture(GL_TEXTURE_2D, texture[1]);

	//Create Squres
	glBegin(GL_QUADS);
	glTexCoord2f(0, 0); glVertex3f(-1, -1, 0.0);
	glTexCoord2f(1, 0); glVertex3f(1, -1, 0.0);
	glTexCoord2f(1, 1); glVertex3f(1, 1, 0.0);
	glTexCoord2f(0, 1); glVertex3f(-1, 1, 0.0);
	glEnd();

	glDisable(GL_TEXTURE_2D);
	glDisable(GL_BLEND);
	glEnable(GL_DEPTH_TEST);
	glEndList();
}


void animate(void) {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//Background
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, texture[0]);
	glBegin(GL_QUADS);
	glTexCoord2f(0, 0); glVertex3f(-130.0, -120.0, -1.0);
	glTexCoord2f(1, 0); glVertex3f(130.0, -120.0, -1.0);
	glTexCoord2f(1, 1); glVertex3f(130.0, 120.0, -1.0);
	glTexCoord2f(0, 1); glVertex3f(-130.0, 120.0, -1.0);
	glEnd();
	glDisable(GL_TEXTURE_2D);

	//Light balls (particles)
	for (int i = 0; i < MAX_PARTICLE_NUM; i++) {
		if (particles[i].life > 0) {
			glPushMatrix();
			glTranslatef(particles[i].x, particles[i].y, 0.0);
			glScalef(particles[i].size, particles[i].size, 1.0);
			glCallList(listname);    //Load texture
			glPopMatrix();

			//Movement of light balls, velocity and acceleration
			particles[i].life -= float(rand() % 100) / 1000.0f + 0.0003f;
			particles[i].vx += particles[i].ax;
			particles[i].vy += particles[i].ay;
			particles[i].vz += particles[i].az;
			particles[i].x += particles[i].vx;
			particles[i].y += particles[i].vy;
			particles[i].z += particles[i].vz;
		}
		//Generate a new light ball when one disappears
		if (particles[i].life <= 0)	{
			particles[i].life = rand() % 450;
			particles[i].size = rand() % 5 + 1;
			particles[i].x = float(rand() % 500 - 250);
			particles[i].y = float(rand() % 100 - 250);
			particles[i].z = float(rand() % 180 - 90.0);
			particles[i].vx = float(rand() % 10 - 4) / 50;
			particles[i].vy = float(rand() % 10) / 40;
			particles[i].vz = float(rand() % 10 - 4) / 50;
			particles[i].ax = (rand() % 3 + 1) / 10000;
			particles[i].ay = 3 / 10000;
			particles[i].az = (rand() % 3 + 1) / 10000;
		}
	}
	glFlush();
	glutSwapBuffers();
}

//Update the viewport and projection matrix when the size of window is changed
void reshape(GLsizei w, GLsizei h) {
	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60.0, (GLfloat)w / (GLfloat)h, 1.0, 210.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(0.0, 0.0, 200.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
}

int main(int argc, char ** argv) {
	//OpenGL window
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);
	glutInitWindowSize(500, 500);
	glutInitWindowPosition(100, 100);
	glutCreateWindow("Animation Lab 5 - Particle System - Xie Wu");

	//Initialization
	initLight();

	//Callback functions
	glutReshapeFunc(reshape);
	glutDisplayFunc(animate);
	glutIdleFunc(animate);

	//Main loop
	glutMainLoop();
	return 0;
}
