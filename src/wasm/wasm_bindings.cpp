#define GLM_ENABLE_EXPERIMENTAL
#include "boids_simulation.h"
#include "boid.h"
#include "obstacle_field.h"
#include "species_params.h"
#include "simulation_tuning.h"
#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include "scale_utils.h"
#include <stdexcept>
#include <algorithm>
#include <emscripten/bind.h>

using namespace emscripten;

void setGlobalSpeciesParamsFromJS(const std::vector<SpeciesParams>& speciesParamsList, float spatialScale = 1.0f)
{
    for (const auto& params : speciesParamsList) {
        // スケール適用して登録
        BoidSimulation::instance().setGlobalSpeciesParams(scaledParams(params, spatialScale));
    }
}
void callInitBoids(const std::vector<SpeciesParams>& speciesParamsList, float spatialScale = 1.0f, float posRange = 1.0f, float velRange = 1.0f) {
    std::vector<SpeciesParams> scaledList;
    scaledList.reserve(speciesParamsList.size());
    for (const auto& params : speciesParamsList) {
        scaledList.push_back(scaledParams(params, spatialScale));
    }
    BoidSimulation::instance().initializeBoids(scaledList, posRange, velRange);
}

// JavaScript 側でグローバルチューニングパラメータを取得
SimulationTuningParams getSimulationTuningParams() {
    return gSimulationTuning;
}

// JavaScript 側からグローバルチューニングパラメータを設定
// maxEscapeWeight は 0〜1 にクランプ
void setSimulationTuningParams(const SimulationTuningParams &params) {
    gSimulationTuning = params;
    gSimulationTuning.maxEscapeWeight = std::clamp(gSimulationTuning.maxEscapeWeight, 0.0f, 1.0f);
    gSimulationTuning.schoolPullCoefficient = std::max(gSimulationTuning.schoolPullCoefficient, 0.0f);

    // ソフト境界は「無効化しやすさ」と「破綻防止」を優先してクランプ。
    gSimulationTuning.softBoundaryRadius = std::max(gSimulationTuning.softBoundaryRadius, 0.0f);
    gSimulationTuning.softBoundaryStart = std::max(gSimulationTuning.softBoundaryStart, 0.0f);
    if (gSimulationTuning.softBoundaryRadius > 0.0f) {
      gSimulationTuning.softBoundaryStart =
        std::min(gSimulationTuning.softBoundaryStart, gSimulationTuning.softBoundaryRadius);
    }
    gSimulationTuning.softBoundarySteer = std::max(gSimulationTuning.softBoundarySteer, 0.0f);
}

void configureGroundPlaneFromJS(bool enabled, float height, float blendDistance,
                                float stiffness, float damping) {
    obstacle_field::configureGroundPlane(enabled, height, blendDistance,
                                         stiffness, damping);
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
    .field("tau", &SpeciesParams::tau)
    .field("horizontalTorque", &SpeciesParams::horizontalTorque)
    .field("torqueStrength", &SpeciesParams::torqueStrength)
    .field("bodyHeadLength", &SpeciesParams::bodyHeadLength)
    .field("bodyTailLength", &SpeciesParams::bodyTailLength)
    .field("bodyRadius", &SpeciesParams::bodyRadius)
    .field("predatorAlertRadius", &SpeciesParams::predatorAlertRadius)
    .field("isPredator", &SpeciesParams::isPredator)
    .field("densityReturnStrength", &SpeciesParams::densityReturnStrength);

value_object<SimulationTuningParams>("SimulationTuningParams")
    .field("threatDecay", &SimulationTuningParams::threatDecay)
    .field("maxEscapeWeight", &SimulationTuningParams::maxEscapeWeight)
    .field("baseEscapeStrength", &SimulationTuningParams::baseEscapeStrength)
    .field("fastAttractStrength", &SimulationTuningParams::fastAttractStrength)
    .field("schoolPullCoefficient", &SimulationTuningParams::schoolPullCoefficient)
    .field("softBoundaryRadius", &SimulationTuningParams::softBoundaryRadius)
    .field("softBoundaryStart", &SimulationTuningParams::softBoundaryStart)
    .field("softBoundarySteer", &SimulationTuningParams::softBoundarySteer);

    class_<Boid>("Boid")
        .constructor<>()
        .property("position", &Boid::position)
        .property("velocity", &Boid::velocity)
        .property("acceleration", &Boid::acceleration)
        .property("id", &Boid::id)
        .property("stress", &Boid::stress)
        .property("speciesId", &Boid::speciesId);

    class_<BoidSimulation>("BoidSimulation")
        .constructor<>()
        .function("build", &BoidSimulation::build)
        .function("update", &BoidSimulation::update)
        .function("setFlockSize", &BoidSimulation::setFlockSize)
        .function("getPositionsPtr", &BoidSimulation::getPositionsPtr)
        .function("getVelocitiesPtr", &BoidSimulation::getVelocitiesPtr)
        .function("getBoidCount", &BoidSimulation::getBoidCount)
        .function("initializeBoids", &BoidSimulation::initializeBoids)
        .function("getGlobalSpeciesParams", &BoidSimulation::getGlobalSpeciesParams)
        .function("setGlobalSpeciesParams", &BoidSimulation::setGlobalSpeciesParams)
        .property("root", &BoidSimulation::root, allow_raw_pointers());

    class_<BoidUnit>("BoidUnit")
        .property("center", &BoidUnit::center)
        .property("radius", &BoidUnit::radius)
        .property("children", &BoidUnit::children, allow_raw_pointers());

    register_vector<Boid>("VectorBoid");
    register_vector<BoidUnit *>("VectorBoidUnit");
    register_vector<SpeciesParams>("VectorSpeciesParams");

    function("setGlobalSpeciesParamsFromJS", &setGlobalSpeciesParamsFromJS);
    function("getSimulationTuningParams", &getSimulationTuningParams);
    function("setSimulationTuningParams", &setSimulationTuningParams);
    function("callInitBoids", &callInitBoids); // 新しい関数を登録
    function("configureGroundPlane", &configureGroundPlaneFromJS);
}

