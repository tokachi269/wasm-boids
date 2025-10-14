#include "entry.h"

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

int main() {
#ifdef __EMSCRIPTEN__
  emscripten_exit_with_live_runtime();
#endif
  Entry::run();
  return 0;
}