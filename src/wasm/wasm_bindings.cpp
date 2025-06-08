#define GLM_ENABLE_EXPERIMENTAL
#include <emscripten/bind.h>
#include "boids_tree.h"
#include "boid.h"
#include "species_params.h"
#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include "scale_utils.h"
#include <stdexcept>

using namespace emscripten;

// JSオブジェクトからグローバルパラメータをセット（undefinedなら現状維持）
void setGlobalSpeciesParamsFromJS(val jsObj, float spatialScale = 1.0f)
{
    if (jsObj["species"].isUndefined()){
        throw std::invalid_argument("Species is required.");
    }

    SpeciesParams params =  BoidTree::instance().getGlobalSpeciesParams(jsObj["species"].as<std::string>());
    if (!jsObj["cohesion"].isUndefined())
        params.cohesion = jsObj["cohesion"].as<float>();
    if (!jsObj["separation"].isUndefined())
        params.separation = jsObj["separation"].as<float>();
    if (!jsObj["alignment"].isUndefined())
        params.alignment = jsObj["alignment"].as<float>();
    if (!jsObj["maxSpeed"].isUndefined())
        params.maxSpeed = jsObj["maxSpeed"].as<float>();
    if (!jsObj["minSpeed"].isUndefined())
        params.minSpeed = jsObj["minSpeed"].as<float>();
    if (!jsObj["maxTurnAngle"].isUndefined())
        params.maxTurnAngle = jsObj["maxTurnAngle"].as<float>();
    if (!jsObj["separationRange"].isUndefined())
        params.separationRange = jsObj["separationRange"].as<float>();
    if (!jsObj["alignmentRange"].isUndefined())
        params.alignmentRange = jsObj["alignmentRange"].as<float>();
    if (!jsObj["cohesionRange"].isUndefined())
        params.cohesionRange = jsObj["cohesionRange"].as<float>();
    if (!jsObj["maxNeighbors"].isUndefined())
        params.maxNeighbors = jsObj["maxNeighbors"].as<int>();
    if (!jsObj["lambda"].isUndefined())
        params.lambda = jsObj["lambda"].as<float>();
    if (!jsObj["horizontalTorque"].isUndefined())
        params.horizontalTorque = jsObj["horizontalTorque"].as<float>();
    if (!jsObj["velocityEpsilon"].isUndefined())
        params.velocityEpsilon = jsObj["velocityEpsilon"].as<float>();
    if (!jsObj["torqueStrength"].isUndefined())
        params.torqueStrength = jsObj["torqueStrength"].as<float>();
    BoidTree::instance().setGlobalSpeciesParams(scaledParams(params, spatialScale));
}

EMSCRIPTEN_BINDINGS(my_module)
{
    value_object<glm::vec3>("Vec3")
        .field("x", &glm::vec3::x)
        .field("y", &glm::vec3::y)
        .field("z", &glm::vec3::z);

    value_object<SpeciesParams>("SpeciesParams")
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
        .field("torqueStrength", &SpeciesParams::torqueStrength);

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


    function("setGlobalSpeciesParamsFromJS", &setGlobalSpeciesParamsFromJS);
}