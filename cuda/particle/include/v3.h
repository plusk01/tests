#pragma once

#include <cuda_runtime.h>

class v3
{
public:
    float x;
    float y;
    float z;

    v3();
    v3(float xIn, float yIn, float zIn);
    void randomize();
    __host__ __device__ void normalize();
    __host__ __device__ void scramble();
};