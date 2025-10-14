#pragma once

#include <iostream>
#include <string>

#ifdef __EMSCRIPTEN__
#include <emscripten/val.h>
#endif

// ブラウザ(WASM)とネイティブの出力先を吸収した軽量ログユーティリティ
namespace logger {
#ifdef __EMSCRIPTEN__
using Console = emscripten::val;

inline Console getConsole() {
  // ブラウザの console オブジェクトを取得
  return emscripten::val::global("console");
}

inline void log(const std::string &message) {
  getConsole().call<void>("log", message);
}
#else
struct Console {
  template <typename T> void call(const char *, const T &message) const {
    std::cout << message << std::endl;
  }
};

inline Console getConsole() {
  // ネイティブ環境ではダミーの Console を返す
  return {};
}

inline void log(const std::string &message) {
  std::cout << message << std::endl;
}
#endif

inline void log(const char *message) { log(std::string(message)); }

} // namespace logger
