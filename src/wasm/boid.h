#pragma once
#include "vec3.h"

struct Boid
{
    Vec3 position, velocity, acceleration;
    int id = 0;
    float stress = 0.0f;
    int speciesId = 0;
};

struct BoidStats
{
    Vec3 sumVelocity, sumPosition, separation;
    int count = 0;
};