#include <vector>
#include "boid.h"
#include <glm/glm.hpp>
#include <boost/align/aligned_allocator.hpp>
template<typename T> using A16 = boost::alignment::aligned_allocator<T,16>;

struct SoABuffers {
    std::vector<glm::vec3, A16<glm::vec3>> positions;
    std::vector<glm::vec3, A16<glm::vec3>> velocities;
    std::vector<glm::vec3, A16<glm::vec3>> accelerations;

    std::vector<int>   ids;
    std::vector<float> stresses;
    std::vector<int>   speciesIds;

    std::unordered_map<int, std::unordered_map<int, float>> cohesionMemories;

    void reserveAll(std::size_t n) {
        positions .reserve(n);
        velocities.reserve(n);
        accelerations.reserve(n);
        ids.reserve(n);
        stresses.reserve(n);
        speciesIds.reserve(n);
    }
};