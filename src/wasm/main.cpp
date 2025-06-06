#include "entry.h"
#include <emscripten.h>

int main() {
    Entry entry;
        // ランタイムを維持
    emscripten_exit_with_live_runtime();

    entry.run();
    return 0;
}