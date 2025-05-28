#include "entry.h"
#include "boids_tree.h"
#include <iostream>

void Entry::run()
{
    std::cout << "WebAssembly entry point initialized!" << std::endl;
}

extern "C"
{

    void initBoids(int n, float pr, float vr)
    {
        BoidTree::instance().initializeBoids(n, pr, vr);
    }
    void build(int maxPerUnit = 16, int level = 0)
    {
        BoidTree::instance().build(maxPerUnit, level);
    }
    uintptr_t posPtr()
    {
        return BoidTree::instance().getPositionsPtr();
    }

    uintptr_t velPtr()
    {
        return BoidTree::instance().getVelocitiesPtr();
    }

    int boidCount()
    {
        return BoidTree::instance().getBoidCount();
    }
}
