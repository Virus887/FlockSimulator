#include <vector_types.h>
#include <GL/glew.h>
#include <gl/freeglut_std.h>
#include <stdio.h>
#include <cmath>

#ifndef INTERACTIONS_H
#define INTERACTIONS_H

#define TITLE_STRING "Damian Bis: Boids simulator"

#define COEF_DELTA 0.05f
#define MIN_ALIGNMENT 0.0f
#define MAX_ALIGNMENT 1.0f
#define MIN_GROUP 0.0f
#define MAX_GROUP 3.0f
#define MIN_SEPARATION 0.00f
#define MAX_SEPARATION 2.00f

#define SPEED_DELTA 0.01f
#define MIN_SPEED 0.02f
#define MAX_SPEED 0.1f

#define LIMIT_ADD(x, delta, limit) (x+delta<=limit ? x+delta : x) 
#define LIMIT_SUB(x, delta, limit) (x-delta>=limit ? x-delta : x) 

//INITIAL DATA
#define HOW_MANY 10000
bool Arrows = true;
float boidSize = 0.8f;

float WIDTH = 2;
float alignCoef = 1.0f;
float groupCoef = .9f;
float separateCoef = 0.6f;

float timeElapsed = 0.05f;

bool MouseMode = false;
bool Paused = false;

unsigned int GridDim;
float2 mouseLocation = { 0.0f, 0.0f };
Flock myFlock;


//interaction handlers

void keyboard(unsigned char key, int x, int y) {
    if (key == 'm') MouseMode = !MouseMode;
    if (key == 'p') Paused = !Paused;

    if (key == '+') timeElapsed = LIMIT_ADD(timeElapsed, SPEED_DELTA, MAX_SPEED);
    if (key == '-') timeElapsed = LIMIT_SUB(timeElapsed, SPEED_DELTA, MIN_SPEED);

    if (key == 'a') alignCoef = LIMIT_ADD(alignCoef, COEF_DELTA, MAX_ALIGNMENT);
    if (key == 'z') alignCoef = LIMIT_SUB(alignCoef, COEF_DELTA, MIN_ALIGNMENT);
    if (key == 's') groupCoef = LIMIT_ADD(groupCoef, COEF_DELTA, MAX_GROUP);
    if (key == 'x') groupCoef = LIMIT_SUB(groupCoef, COEF_DELTA, MIN_GROUP);
    if (key == 'd') separateCoef = LIMIT_ADD(separateCoef, COEF_DELTA, MAX_SEPARATION);
    if (key == 'c') separateCoef = LIMIT_SUB(separateCoef, COEF_DELTA, MIN_SEPARATION);

    if (key == 'r')
    {
        alignCoef = 1.0f;
        groupCoef = .9f;
        separateCoef = 0.3f;
        timeElapsed = 0.05f;
    }

    if (key == 27)
    {
        exit(0);
    }      
}

void mouseMove(int x, int y) 
{
    double w = glutGet(GLUT_WINDOW_WIDTH);
    double h = glutGet(GLUT_WINDOW_HEIGHT);
    mouseLocation.x = (x / (float)w) * 2.0f*WIDTH - WIDTH;
    mouseLocation.y = -(y / (float)h) * 2.0f*HEIGHT + HEIGHT;
}

void printInstructions() {
   printf("Instructions:\n");
   printf("To manipulate boids speed use '+' and '-' keys, \n");
   printf("To manipulate boids alligment use 'A' and 'Z' keys, \n");
   printf("To manipulate boids cohesion use 'S' and 'X' keys, \n");
   printf("To manipulate boids separation use 'D' and 'C' keys, \n");
   printf("To pause application use 'P' key, \n");

   printf("To make boids to avoid mouse use 'M' key. \n");

   printf("To reset your setting click 'R' key. \n");
   printf("\n\nEnjoy :)\n");
  
}

#endif