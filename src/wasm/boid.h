#pragma once
#include "vec3.h"
#include <unordered_map>

static constexpr int COHESION_MEM_SIZE = 256;
struct CohesionEntry { int id = -1; float timer = 0.0f; };

struct Boid
{
    Vec3 position, velocity, acceleration;
    int id = 0;
    float stress = 0.0f;
    int speciesId = 0;
    CohesionEntry cohesionMemory[COHESION_MEM_SIZE];
    int cohesionMemHead = 0;
};

struct BoidStats
{
    Vec3 sumVelocity, sumPosition, separation;
    int count = 0;
};