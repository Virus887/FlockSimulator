#include <thrust/device_vector.h>
#include <thrust/sort.h>
#include <thrust/execution_policy.h>
#include <thrust/binary_search.h>
#include <thrust/functional.h>

#include "stdio.h"
#include <cmath>
#include <cuda_runtime.h>

#include "kernel.h"


__device__
float angle(float x, float y) 
{ 
	float theta = std::atan2(x, y); 
	theta *= 180.0f / 3.14f; 
	return theta - 90;
}


__global__
void MyKernel1(float2* d_p, float2* d_v, float* d_angle,float2 mouseLocation, 
    unsigned int* indexes, unsigned int* begins, unsigned int* ends,
    int howMany, int GridDim, float WIDTH, bool MouseMode, float allignCoef, float groupCoef, float separateCoef, float timeElapsed ) {

    //give boid appropiate perception distances
    float allignDistance = 0.08f;
    float groupDistance = 0.08f;
    float separateDistance = 0.03f;
    if (howMany <= 2000)
    {
        allignDistance = 0.24f;
        groupDistance = 0.24f;
        separateDistance = 0.1f;
    }

    //calculate boid index 
    const int i = blockDim.x * blockIdx.x + threadIdx.x;
    if (i >= howMany) return;
    const int ii = indexes[i];

    //calculate boid bucket
    unsigned int x = ((d_p[ii].x + WIDTH) / (WIDTH * 2)) *GridDim;
    unsigned int y = ((d_p[ii].y + HEIGHT) / (HEIGHT * 2)) * GridDim;
    int bucket =  y * GridDim + x;

    
    //discard neighbour buckets that boid cannot see       //  __________
    bool b2 = (bucket / GridDim != GridDim - 1);           //  |b7 b8 b9|   
    bool b4 = (bucket % GridDim != 0);                     //  |b4    b6|
    bool b6 = (bucket % GridDim != GridDim - 1);           //  |b1 b2 b3|
    bool b8 = (bucket / GridDim != 0);                     //  ~~~~~~~~~~

    b6 = (b6 && ((x + 1) * (2 * WIDTH / (GridDim)) - (d_p[ii].x + WIDTH) <= allignDistance));
    b4 = (b4 && ((d_p[ii].x + WIDTH) - (x * (2 * WIDTH / GridDim)) <= allignDistance));
    b8 = (b8 && (((y + 1) * 2 * HEIGHT / (GridDim)) - (d_p[ii].y + HEIGHT) <= allignDistance));
    b2 = (b2 && ((d_p[ii].y + HEIGHT) - (y * (2 * HEIGHT / GridDim)) <= allignDistance));

    bool b1 = (b2 && b4);
    bool b3 = (b6 && b2);
    bool b7 = (b8 && b4);
    bool b9 = (b8 && b6);

#pragma region collect_data
    //collect neighbours data
    float vxAllign = 0, vyAllign = 0, dxGroup = 0, dyGroup = 0, dxSeparate = 0, dySeparate = 0, distance;
    int counterAllign = 0, counterGroup = 0, counterSeparate = 0;

        for (int j = begins[bucket]; j < ends[bucket]; ++j)
        {
            int jj = indexes[j];
            if (ii == jj) continue;
            distance = sqrtf(((d_p[jj].x - d_p[ii].x) * (d_p[jj].x - d_p[ii].x)) + ((d_p[jj].y - d_p[ii].y) * (d_p[jj].y - d_p[ii].y)));
            if (distance < allignDistance) 
            {
                vxAllign += (d_v[jj].x);
                vyAllign += (d_v[jj].y);
                counterAllign++;
            }
            if (distance < groupDistance) 
            {
                dxGroup += d_p[jj].x;
                dyGroup += d_p[jj].y;
                counterGroup++;
            }
            if (distance < separateDistance && distance > 0.0001f) 
            {
                dxSeparate -= ((d_p[jj].x - d_p[ii].x) / (distance));
                dySeparate -= ((d_p[jj].y - d_p[ii].y) / (distance));
                counterSeparate++;
            }       
        }

        if (b1) for (int j = begins[bucket - GridDim - 1]; j < ends[bucket - GridDim - 1]; ++j)
        {
            int jj = indexes[j];
            if (ii == jj) continue;
            distance = sqrtf(((d_p[jj].x - d_p[ii].x) * (d_p[jj].x - d_p[ii].x)) + ((d_p[jj].y - d_p[ii].y) * (d_p[jj].y - d_p[ii].y)));
            if (distance < allignDistance)
            {
                vxAllign += (d_v[jj].x);
                vyAllign += (d_v[jj].y);
                counterAllign++;
            }
            if (distance < groupDistance)
            {
                dxGroup += d_p[jj].x;
                dyGroup += d_p[jj].y;
                counterGroup++;
            }
            if (distance < separateDistance && distance > 0.0001f)
            {
                dxSeparate -= ((d_p[jj].x - d_p[ii].x) / (distance));
                dySeparate -= ((d_p[jj].y - d_p[ii].y) / (distance));
                counterSeparate++;
            }
        }

        if (b2) for (int j = begins[bucket - GridDim]; j < ends[bucket - GridDim]; ++j)
        {
            int jj = indexes[j];
            if (ii == jj) continue;
            distance = sqrtf(((d_p[jj].x - d_p[ii].x) * (d_p[jj].x - d_p[ii].x)) + ((d_p[jj].y - d_p[ii].y) * (d_p[jj].y - d_p[ii].y)));
            if (distance < allignDistance)
            {
                vxAllign += (d_v[jj].x);
                vyAllign += (d_v[jj].y);
                counterAllign++;
            }
            if (distance < groupDistance)
            {
                dxGroup += d_p[jj].x;
                dyGroup += d_p[jj].y;
                counterGroup++;
            }
            if (distance < separateDistance && distance > 0.0001f)
            {
                dxSeparate -= ((d_p[jj].x - d_p[ii].x) / (distance));
                dySeparate -= ((d_p[jj].y - d_p[ii].y) / (distance));
                counterSeparate++;
            }
        }

        if (b3) for (int j = begins[bucket - GridDim + 1]; j < ends[bucket - GridDim + 1]; ++j)
        {
            int jj = indexes[j];
            if (ii == jj) continue;
            distance = sqrtf(((d_p[jj].x - d_p[ii].x) * (d_p[jj].x - d_p[ii].x)) + ((d_p[jj].y - d_p[ii].y) * (d_p[jj].y - d_p[ii].y)));
            if (distance < allignDistance)
            {
                vxAllign += (d_v[jj].x);
                vyAllign += (d_v[jj].y);
                counterAllign++;
            }
            if (distance < groupDistance)
            {
                dxGroup += d_p[jj].x;
                dyGroup += d_p[jj].y;
                counterGroup++;
            }
            if (distance < separateDistance && distance > 0.0001f)
            {
                dxSeparate -= ((d_p[jj].x - d_p[ii].x) / (distance));
                dySeparate -= ((d_p[jj].y - d_p[ii].y) / (distance));
                counterSeparate++;
            }
        }

        if (b4) for (int j = begins[bucket - 1]; j < ends[bucket - 1]; ++j)
        {
            int jj = indexes[j];
            if (ii == jj) continue;
            distance = sqrtf(((d_p[jj].x - d_p[ii].x) * (d_p[jj].x - d_p[ii].x)) + ((d_p[jj].y - d_p[ii].y) * (d_p[jj].y - d_p[ii].y)));
            if (distance < allignDistance)
            {
                vxAllign += (d_v[jj].x);
                vyAllign += (d_v[jj].y);
                counterAllign++;
            }
            if (distance < groupDistance)
            {
                dxGroup += d_p[jj].x;
                dyGroup += d_p[jj].y;
                counterGroup++;
            }
            if (distance < separateDistance && distance > 0.0001f)
            {
                dxSeparate -= ((d_p[jj].x - d_p[ii].x) / (distance));
                dySeparate -= ((d_p[jj].y - d_p[ii].y) / (distance));
                counterSeparate++;
            }
        }

        if (b6) for (int j = begins[bucket + 1]; j < ends[bucket + 1]; ++j)
        {
            int jj = indexes[j];
            if (ii == jj) continue;
            distance = sqrtf(((d_p[jj].x - d_p[ii].x) * (d_p[jj].x - d_p[ii].x)) + ((d_p[jj].y - d_p[ii].y) * (d_p[jj].y - d_p[ii].y)));
            if (distance < allignDistance)
            {
                vxAllign += (d_v[jj].x);
                vyAllign += (d_v[jj].y);
                counterAllign++;
            }
            if (distance < groupDistance)
            {
                dxGroup += d_p[jj].x;
                dyGroup += d_p[jj].y;
                counterGroup++;
            }
            if (distance < separateDistance && distance > 0.0001f)
            {
                dxSeparate -= ((d_p[jj].x - d_p[ii].x) / (distance));
                dySeparate -= ((d_p[jj].y - d_p[ii].y) / (distance));
                counterSeparate++;
            }
        }

        if (b7) for (int j = begins[bucket + GridDim - 1]; j < ends[bucket + GridDim - 1]; ++j)
        {
            int jj = indexes[j];
            if (ii == jj) continue;
            distance = sqrtf(((d_p[jj].x - d_p[ii].x) * (d_p[jj].x - d_p[ii].x)) + ((d_p[jj].y - d_p[ii].y) * (d_p[jj].y - d_p[ii].y)));
            if (distance < allignDistance)
            {
                vxAllign += (d_v[jj].x);
                vyAllign += (d_v[jj].y);
                counterAllign++;
            }
            if (distance < groupDistance)
            {
                dxGroup += d_p[jj].x;
                dyGroup += d_p[jj].y;
                counterGroup++;
            }
            if (distance < separateDistance && distance > 0.0001f)
            {
                dxSeparate -= ((d_p[jj].x - d_p[ii].x) / (distance));
                dySeparate -= ((d_p[jj].y - d_p[ii].y) / (distance));
                counterSeparate++;
            }
        }

        if (b8) for (int j = begins[bucket + GridDim]; j < ends[bucket + GridDim]; ++j)
        {
            int jj = indexes[j];
            if (ii == jj) continue;
            distance = sqrtf(((d_p[jj].x - d_p[ii].x) * (d_p[jj].x - d_p[ii].x)) + ((d_p[jj].y - d_p[ii].y) * (d_p[jj].y - d_p[ii].y)));
            if (distance < allignDistance)
            {
                vxAllign += (d_v[jj].x);
                vyAllign += (d_v[jj].y);
                counterAllign++;
            }
            if (distance < groupDistance)
            {
                dxGroup += d_p[jj].x;
                dyGroup += d_p[jj].y;
                counterGroup++;
            }
            if (distance < separateDistance && distance > 0.0001f)
            {
                dxSeparate -= ((d_p[jj].x - d_p[ii].x) / (distance));
                dySeparate -= ((d_p[jj].y - d_p[ii].y) / (distance));
                counterSeparate++;
            }
        }

        if (b9) for (int j = begins[bucket + GridDim + 1]; j < ends[bucket + GridDim + 1]; ++j)
        {
            int jj = indexes[j];
            if (ii == jj) continue;
            distance = sqrtf(((d_p[jj].x - d_p[ii].x) * (d_p[jj].x - d_p[ii].x)) + ((d_p[jj].y - d_p[ii].y) * (d_p[jj].y - d_p[ii].y)));
            if (distance < allignDistance)
            {
                vxAllign += (d_v[jj].x);
                vyAllign += (d_v[jj].y);
                counterAllign++;
            }
            if (distance < groupDistance)
            {
                dxGroup += d_p[jj].x;
                dyGroup += d_p[jj].y;
                counterGroup++;
            }
            if (distance < separateDistance && distance > 0.0001f)
            {
                dxSeparate -= ((d_p[jj].x - d_p[ii].x) / (distance));
                dySeparate -= ((d_p[jj].y - d_p[ii].y) / (distance));
                counterSeparate++;
            }
        }

#pragma endregion collect_data

        float ax = 0;
        float ay = 0;

        // check if boid is near mouse cursor
        float dist = sqrtf(((mouseLocation.x - d_p[ii].x) * (mouseLocation.x - d_p[ii].x)) + ((mouseLocation.y - d_p[ii].y) * (mouseLocation.y - d_p[ii].y)));
        if (MouseMode && dist < MOUSE_RADIUS)
        {
            ax = ((d_p[ii].x - mouseLocation.x) / dist) * MOUSE_FORCE;
            ay = ((d_p[ii].y - mouseLocation.y) / dist) * MOUSE_FORCE;

            if (dist > 0.9f * MOUSE_RADIUS)
            {
                ax = -d_v[ii].x * 0.5f;
                ay = -d_v[ii].x * 0.5f;
            }
        }
        //Calculate acceleration
        else
        {
           if (counterAllign)
           {
               vxAllign /= counterAllign;
               vyAllign /= counterAllign;
               vxAllign *= allignCoef;
               vyAllign *= allignCoef;
               ax += (vxAllign - d_v[ii].x);
               ay += (vyAllign - d_v[ii].y);
           }
           if (counterGroup)
           {
               dxGroup /= (counterGroup);
               dyGroup /= (counterGroup);
               dxGroup -= d_p[ii].x;
               dyGroup -= d_p[ii].y;
               dxGroup *= groupCoef;
               dyGroup *= groupCoef;
               ax += (dxGroup);
               ay += (dyGroup);
           }
           if (counterSeparate)
           {
               dxSeparate /= counterSeparate;
               dySeparate /= counterSeparate;
               dxSeparate *= separateCoef;
               dySeparate *= separateCoef;
               ax += (dxSeparate);
               ay += (dySeparate);
           }   

        }

        //speed up slow boids
        if (fabs(d_v[ii].x) < 0.1f && fabs(d_v[ii].y) < 0.1f) 
        {
            d_v[ii].x *= 1.03f;
            d_v[ii].y *= 1.03f;
        }
        //water resistance slows fishes
        else 
        {
            d_v[ii].x *= 0.99f;
            d_v[ii].y *= 0.99f;
        }

        //speed up boids and change their positions
        d_v[ii].x += ax * timeElapsed;
        d_v[ii].y += ay * timeElapsed;

        d_p[ii].x += d_v[ii].x * timeElapsed;
        d_p[ii].y += d_v[ii].y * timeElapsed;

        //calculate boid swimming angle
        d_angle[ii] = angle(d_v[ii].y, d_v[ii].x);

        //keep boids in displaying area
        if (d_p[ii].x >= WIDTH) d_p[ii].x = -WIDTH + 0.002f;
        else if (d_p[ii].x <= -WIDTH) d_p[ii].x = WIDTH - 0.002f;
        if (d_p[ii].y >= HEIGHT) d_p[ii].y = -HEIGHT + 0.002f;
        else if (d_p[ii].y <= -HEIGHT) d_p[ii].y = HEIGHT - 0.002f;
}




struct calculate_bucket_index
{
    float w, h; 
    int gridDim;
    __host__ __device__
        calculate_bucket_index(float width, float height, int dim)
        :w(width), h(height), gridDim(dim) {}
    __host__ __device__
     unsigned int operator()(float2 p) const
    {
        // coordinates of the grid cell containing point p
        unsigned int x = ((p.x + w) / (w * 2)) * gridDim;                               
        unsigned int y = ((p.y + h) / (h * 2)) * gridDim;
        // return the bucket's linear index
        return y * gridDim + x;
    }
};


//Calculate new speeds and positions of each boid
void boidsLauncher(Flock* flock, float2 pos, int howMany,float WIDTH, bool MouseMode,float alignCoef,float groupCoef,float separateCoef,float timeElapsed, unsigned int gridDim)
{
    cudaError_t err = cudaSuccess;

    //Copy memory to device
    thrust::device_vector<float2> d_p = flock->p;
    thrust::device_vector<float2> d_v = flock->v;
    thrust::device_vector<float> d_angle = flock->angle;
    thrust::device_vector<unsigned int> sorted_indexes = flock->index;
 
    //bucket sort using thrust library
    //give each boid its bucket index 
    thrust::device_vector<unsigned int> bucket_indices(howMany);
    thrust::transform(d_p.begin(), d_p.end(), bucket_indices.begin(), calculate_bucket_index(WIDTH, HEIGHT, gridDim));

    //sort boid indexes by bucket
    thrust::sort_by_key(bucket_indices.begin(), bucket_indices.end(), sorted_indexes.begin());

    //return each bucket begin and end
    thrust::device_vector<unsigned int> bucket_start(gridDim * gridDim);
    thrust::device_vector<unsigned int> bucket_end(gridDim * gridDim);

    thrust::counting_iterator<unsigned int> search_begin(0);
    thrust::lower_bound(bucket_indices.begin(), bucket_indices.end(), search_begin, search_begin + gridDim * gridDim, bucket_start.begin());  //returns iterator on first boid with bucket index => i
    thrust::upper_bound(bucket_indices.begin(), bucket_indices.end(), search_begin, search_begin + gridDim * gridDim, bucket_end.begin());  //returns iterator on first boid with bucket index > i 


    //cast thrust vectors to raw_pointers to use them in kernel
    float2* ptr_p = thrust::raw_pointer_cast(&d_p[0]);
    float2* ptr_v = thrust::raw_pointer_cast(&d_v[0]);
    float* ptr_angle = thrust::raw_pointer_cast(&d_angle[0]);
    unsigned int* ptr_starts = thrust::raw_pointer_cast(&bucket_start[0]);
    unsigned int* ptr_ends = thrust::raw_pointer_cast(&bucket_end[0]);
    unsigned int* ptr_sorted_indexes = thrust::raw_pointer_cast(&sorted_indexes[0]);


    //run kernel 
    int threadsPerBlock = 256;
    int blocksPerGrid = (howMany + threadsPerBlock - 1) / threadsPerBlock;
    MyKernel1 <<<blocksPerGrid,  threadsPerBlock>> > (ptr_p, ptr_v, ptr_angle, pos, ptr_sorted_indexes, ptr_starts, ptr_ends, 
                                                        howMany, gridDim, WIDTH, MouseMode, alignCoef, groupCoef, separateCoef, timeElapsed);  

    //check errors
    err = cudaGetLastError();
    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to launch boids kernel (error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }

    //copy memory from device to host
    flock->p = d_p;
    flock->v = d_v;
    flock->angle = d_angle;
}

