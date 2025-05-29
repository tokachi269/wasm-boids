#pragma once
#define GLM_ENABLE_EXPERIMENTAL
#include <unordered_map>
#include <glm/glm.hpp>

struct Boid
{
    glm::vec3 position;
    glm::vec3 velocity;
    glm::vec3 acceleration;
    int id = 0;
    float stress = 0.0f;
    int speciesId = 0;

    // 近傍BoidのIDとその経過時間（τ管理）
    std::unordered_map<int, float> cohesionMemory;

    bool attractionEnabled = false;
    float attractionStartTime = -1.0f;
};