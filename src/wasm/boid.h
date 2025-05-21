#pragma once
#include "vec3.h"
#include <unordered_map>

struct CohesionMemoryEntry {
    float timer; // 経過時間
};

struct Boid
{
    Vec3 position, velocity, acceleration;
    int id = 0;
    float stress = 0.0f;
    int speciesId = 0;
    std::unordered_map<int, CohesionMemoryEntry> cohesionMemory; // 近傍Boidのid→経過時間
};

struct BoidStats
{
    Vec3 sumVelocity, sumPosition, separation;
    int count = 0;
};