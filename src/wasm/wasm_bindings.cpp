#define GLM_ENABLE_EXPERIMENTAL
#include "boids_tree.h"
#include "boid.h"
#include "species_params.h"
#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include "scale_utils.h"
#include <stdexcept>
#include <emscripten/bind.h>

using namespace emscripten;

void setGlobalSpeciesParamsFromJS(const std::vector<SpeciesParams>& speciesParamsList, float spatialScale = 1.0f)
{
    for (const auto& params : speciesParamsList) {
        // スケール適用して登録
        BoidTree::instance().setGlobalSpeciesParams(scaledParams(params, spatialScale));
    }
}
void initBoidsFromJS(const std::vector<SpeciesParams>& speciesParamsList, float posRange, float velRange) {
    BoidTree::instance().initializeBoids(speciesParamsList, posRange, velRange);
}
EMSCRIPTEN_BINDINGS(my_module)
{
    value_object<glm::vec3>("Vec3")
        .field("x", &glm::vec3::x)
        .field("y", &glm::vec3::y)
        .field("z", &glm::vec3::z);

value_object<SpeciesParams>("SpeciesParams")
    .field("species", &SpeciesParams::species)
    .field("count", &SpeciesParams::count)
    .field("cohesion", &SpeciesParams::cohesion)
    .field("separation", &SpeciesParams::separation)
    .field("alignment", &SpeciesParams::alignment)
    .field("maxSpeed", &SpeciesParams::maxSpeed)
    .field("minSpeed", &SpeciesParams::minSpeed)
    .field("maxTurnAngle", &SpeciesParams::maxTurnAngle)
    .field("separationRange", &SpeciesParams::separationRange)
    .field("alignmentRange", &SpeciesParams::alignmentRange)
    .field("cohesionRange", &SpeciesParams::cohesionRange)
    .field("maxNeighbors", &SpeciesParams::maxNeighbors)
    .field("lambda", &SpeciesParams::lambda)
    .field("horizontalTorque", &SpeciesParams::horizontalTorque)
    .field("velocityEpsilon", &SpeciesParams::velocityEpsilon)
    .field("torqueStrength", &SpeciesParams::torqueStrength)
    .field("isPredator", &SpeciesParams::isPredator);

    class_<Boid>("Boid")
        .constructor<>()
        .property("position", &Boid::position)
        .property("velocity", &Boid::velocity)
        .property("acceleration", &Boid::acceleration)
        .property("id", &Boid::id)
        .property("stress", &Boid::stress)
        .property("speciesId", &Boid::speciesId);

    class_<BoidTree>("BoidTree")
        .constructor<>()
        .function("build", &BoidTree::build)
        .function("update", &BoidTree::update)
        .function("setFlockSize", &BoidTree::setFlockSize)
        .function("getPositionsPtr", &BoidTree::getPositionsPtr)
        .function("getVelocitiesPtr", &BoidTree::getVelocitiesPtr)
        .function("getBoidCount", &BoidTree::getBoidCount)
        .function("initializeBoids", &BoidTree::initializeBoids)
        .function("getGlobalSpeciesParams", &BoidTree::getGlobalSpeciesParams)
        .function("setGlobalSpeciesParams", &BoidTree::setGlobalSpeciesParams)
        .property("root", &BoidTree::root, allow_raw_pointers());

    class_<BoidUnit>("BoidUnit")
        .property("center", &BoidUnit::center)
        .property("radius", &BoidUnit::radius)
        .property("children", &BoidUnit::children, allow_raw_pointers());

    register_vector<Boid>("VectorBoid");
    register_vector<BoidUnit *>("VectorBoidUnit");
    register_vector<SpeciesParams>("VectorSpeciesParams");

    function("setGlobalSpeciesParamsFromJS", &setGlobalSpeciesParamsFromJS);
    function("callInitBoids", &initBoidsFromJS); // 新しい関数を登録
}

