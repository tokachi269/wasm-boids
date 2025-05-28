#include "entry.h"
#include "boids_tree.h"
#include <iostream>

void Entry::run() {
    std::cout << "WebAssembly entry point initialized!" << std::endl;
}

extern "C" {
    uintptr_t getPositionsPtr() {
        return boidTree.getPositionsPtr();
    }
}