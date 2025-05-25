#pragma once

struct SpeciesParams
{
    float cohesion = 0.01f;
    float separation = 0.1f;
    float alignment = 0.05f;
    float maxSpeed = 1.0f;
    float minSpeed = 0.1f;
    float maxTurnAngle = 0.01f;
    float separationRange = 10.0f;
    float alignmentRange = 30.0f;
    float cohesionRange = 50.0f;
    int maxNeighbors = 7;
    float lambda = 0.05f;
    float tau = 1.0f;
    float horizontalTorque = 0.02f;
    float velocityEpsilon = 0.0001f;
    float torqueStrength = 0.1f;
};