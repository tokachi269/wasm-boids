#include "boid_factory.h"

std::vector<Boid> BoidFactory::generateRandomBoids(int count, float posRange, float velRange) {
    std::vector<Boid> boids;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> posDist(-posRange, posRange);
    std::uniform_real_distribution<float> velDist(-velRange, velRange);

    for (int i = 0; i < count; ++i)
    {
        Boid b;
        b.position = glm::vec3(posDist(gen), posDist(gen), posDist(gen));
        b.velocity = glm::vec3(velDist(gen), velDist(gen), velDist(gen));
        b.acceleration = glm::vec3(0, 0, 0);
        b.id = i;
        b.stress = 0.0f;
        b.speciesId = 0;
        boids.push_back(b);
    }
    return boids;
}
