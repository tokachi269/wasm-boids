#include "entry.h"

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

int main(int argc, char **argv) {
#ifdef __EMSCRIPTEN__
  emscripten_exit_with_live_runtime();
#endif
  return Entry::run(argc, argv);
}
