#include "kernel.h"
#include "interactions.h"
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <cmath>
#ifdef _WIN32
#define WINDOWS_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#endif
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glew.h>
#include <GL/freeglut.h>
#endif
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>
#include <cuda_runtime.h>

using namespace std;

//FPS measurement variables
int counter = 0;
int frameCount = 0;
time_t initialTime = time(NULL);
time_t finalTime;
//

vector< float > boidGeom;

//Create boid shape
vector< float > generateBoidGeom()
{
    vector< float > buf;
    buf.push_back(0);
    buf.push_back(0.02f* boidSize);
    buf.push_back(0.01f* boidSize);
    buf.push_back(-0.02f* boidSize);
    buf.push_back(-0.01f* boidSize);
    buf.push_back(-0.02f* boidSize);
    if (Arrows)
    {
        buf.push_back(0);
        buf.push_back(-0.06f * boidSize);
        buf.push_back(0.005f * boidSize);
        buf.push_back(-0.02f * boidSize);
        buf.push_back(-0.005f * boidSize);
        buf.push_back(-0.02f * boidSize);
    }
    return buf;
}

//set appropiate grid dimention
void setGridDim(int howMany)
{
    int x = howMany / 1000;
    GridDim = 3 * (x+1);
    if (GridDim > 50) GridDim = 50;
}

//initialize flock
void init()
{
    srand(0);
    setGridDim(HOW_MANY);
    myFlock = Flock(HOW_MANY);
    boidGeom = generateBoidGeom();
}


void PrepareBoidsToDisplay()
{
    // to measure time of calculations

    //int beg = glutGet(GLUT_ELAPSED_TIME);
    boidsLauncher(&myFlock, mouseLocation, HOW_MANY, WIDTH, MouseMode, alignCoef,groupCoef,separateCoef,timeElapsed, GridDim);
    //int end = glutGet(GLUT_ELAPSED_TIME);
    //double elapsed = (double)(end - beg);
    //cout << elapsed << "ms" << endl;

    //display each boid
    for (int i = 0; i < HOW_MANY; i++)
    {
        glPushMatrix(); 
        glTranslatef(myFlock.p[i].x, myFlock.p[i].y, 0);
        glRotatef(myFlock.angle[i], 0.0f, 0.0f, 1.0f);
        glColor3ub(myFlock.color[i].x, myFlock.color[i].y, myFlock.color[i].z);
        glDrawArrays(GL_TRIANGLES, 0, boidGeom.size() / 2);
        glPopMatrix(); 
    }

}

//
void DisplayPause()
{
    char pausebuf[7] = "PAUSED";
    if (Paused)
    {
        glColor3f(1, 0, 0);
        glRasterPos2f(-.2f, 0);
        glutBitmapString(GLUT_BITMAP_TIMES_ROMAN_24, (unsigned const char*)pausebuf);
    }
}

void DisplayText()
{
    char* buf = (char*) malloc (200 * sizeof(char));  
    sprintf(buf, 
     "Boids Count: %d \n\nMouse Animation: %s \n\nBoids speed: %.2f\nAllign power: %.2f \nGroup power: %.2f\nSeparate power: %.2f"
        , HOW_MANY, MouseMode ? "ON" : "OFF", timeElapsed * 100, alignCoef , groupCoef, separateCoef);
    glColor3f(0.75, 0.25f, 0.25f);
    glRasterPos2f(-WIDTH + 0.03f, 1.9f);
    glutBitmapString(GLUT_BITMAP_TIMES_ROMAN_24, (unsigned const char*)buf);
    free(buf);
}



void display()
{
    if (Paused)
    {
        glLoadIdentity();
        DisplayPause();
        glutSwapBuffers();
        return;
    }
    glClear(GL_COLOR_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    //set relative width and height
    float w = glutGet(GLUT_WINDOW_WIDTH);
    float h = glutGet(GLUT_WINDOW_HEIGHT);
    WIDTH = (w / h) * HEIGHT;
    glOrtho(-WIDTH, WIDTH, -HEIGHT, HEIGHT, -1, 1);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(2, GL_FLOAT, 0, &boidGeom[0]);

    //launch kernel and display boids
    PrepareBoidsToDisplay();

    //print informations about boids
    DisplayText();

    glDisableClientState(GL_VERTEX_ARRAY);

    glutSwapBuffers();

    //measure FPS
    frameCount++;
    finalTime = time(NULL);
    if (finalTime - initialTime >= 1)
    {
        char title[256];
        sprintf(title, "%s , Fps: %d", TITLE_STRING, frameCount / (finalTime - initialTime));
        glutSetWindowTitle(title);
        frameCount = 0;
        initialTime = finalTime;
    }
}

//timer function posting redisplay
void timer(int extra)
{
    glutPostRedisplay();
    glutTimerFunc(1000/60, timer, 0);
}


int main(int argc, char** argv)
{
    printInstructions();

    //Initialize window
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
    glutInitWindowSize(1000, 800);
    glutCreateWindow(TITLE_STRING);
    init();

    //Set appropiate handlers
    glutKeyboardFunc(keyboard);
    glutPassiveMotionFunc(mouseMove);
    glutDisplayFunc(display);
    glutTimerFunc(0, timer, 0);
    glutMainLoop();

    return 0;
}