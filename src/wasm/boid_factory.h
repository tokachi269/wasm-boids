#pragma once
#include <vector>
#include <glm/glm.hpp>
#include <random>

#include "boid.h"

class BoidFactory {
public:
    static std::vector<Boid> generateRandomBoids(int count, float posRange, float velRange);
};