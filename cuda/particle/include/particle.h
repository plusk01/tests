#pragma once

#include "v3.h"

class particle
{
public:
    particle();

    __host__ __device__ void advance(float dist);
    const v3& getTotalDistance() const;

private:
    v3 position;
    v3 velocity;
    v3 totalDistance;
};
