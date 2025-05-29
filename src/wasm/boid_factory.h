#pragma once
#include <vector>
#include "boid.h"
#include <glm/glm.hpp>
#include <random>

class BoidFactory {
public:
    static std::vector<Boid> generateRandomBoids(int count, float posRange, float velRange);
};