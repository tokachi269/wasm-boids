#pragma once
#include "species_params.h"

/**
 * spatialScale = 1.0 なら従来と同じ挙動。
 * 0.01 なら「モデルを 100 分の 1 に縮小」したのと同じ効果。
 */
inline SpeciesParams scaledParams(const SpeciesParams &src, float spatialScale)
{
    SpeciesParams dst = src;
    dst.maxSpeed        *= spatialScale;
    if(dst.minSpeed > 0.0f)
        dst.minSpeed    *= dst.maxSpeed * 0.4f;
    return dst;
}