#ifndef KERNEL_H
#define KERNEL_H

#define MAXV 2
#define MOUSE_RADIUS 0.7f
#define MOUSE_FORCE  1.0f
#define HEIGHT 2.0f

#include <thrust/host_vector.h>

struct float2;
struct char3;

struct Flock
{
    int howMany;
    thrust::host_vector<unsigned int> index;
    thrust::host_vector<float2> p;
    thrust::host_vector<float2> v;
    thrust::host_vector<float> angle;
    thrust::host_vector<char3> color;
    Flock(int HowMany)
    {
        howMany = HowMany;
        index = thrust::host_vector<unsigned int>(howMany);
        p = thrust::host_vector<float2>(howMany);
        v = thrust::host_vector<float2>(howMany);
        angle = thrust::host_vector<float>(howMany);
        color = thrust::host_vector<char3>(howMany);
        for (int i = 0; i < howMany; i++)
        {
            index[i] = i;
            p[i].x = (rand() % 400 / 100.0f) - 2.0f;
            p[i].y = (rand() % 400 / 100.0f) - 2.0f;
            v[i].x = (rand() % 2 == 0 ? MAXV / 2.0f + (rand() % 100 / 200.0f) * MAXV : -(MAXV / 2.0f) - ((rand() % 100 / 200.0f) * MAXV)) / 10.0f;
            v[i].y = (rand() % 2 == 0 ? MAXV / 2.0f + (rand() % 100 / 200.0f) * MAXV : -MAXV / 2.0f - (rand() % 100 / 200.0f) * MAXV) / 10.0f;
            angle[i] = 0.0f;
            color[i].x = rand() % 127 + 128;
            color[i].y = rand() % 127 + 128;
            color[i].z = rand() % 127 + 128;
        }
    }
    Flock() {}
};

void boidsLauncher(Flock* flock, float2 pos, int howMany, float WIDTH, bool MouseMode, float alignCoef, float groupCoef, float separateCoef, float timeElapsed, unsigned int gridDim);

#endif