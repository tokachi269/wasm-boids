# CMakeの最低バージョンを指定
cmake_minimum_required(VERSION 3.20)

# プロジェクト名と使用する言語を指定
project(wasm_boids LANGUAGES CXX)

# Emscriptenを使用するための設定
set(CMAKE_C_COMPILER "D:/GitHub/emsdk/upstream/emscripten/emcc")
set(CMAKE_CXX_COMPILER "D:/GitHub/emsdk/upstream/emscripten/em++")
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options("$<$<C_COMPILER_ID:MSVC>:/utf-8>")
add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")

# ビルドタイプを設定
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

# ソースファイルを指定
set(SOURCES
    src/wasm/main.cpp
    src/wasm/boid_factory.cpp
    src/wasm/entry.cpp
    src/wasm/boid_unit.cpp
    src/wasm/boids_tree.cpp
    src/wasm/wasm_bindings.cpp
)

# インクルードディレクトリを指定
include_directories(
    D:/GitHub/vcpkg/installed/x64-windows/include
    D:/GitHub/emsdk/upstream/emscripten/system/include
    D:/GitHub/emsdk/upstream/emscripten/cache/sysroot/include
)

# emcc用のオプションを設定
set(EMCC_FLAGS
    "-s WASM=1"
    "-s MODULARIZE=1"
    "-s EXPORT_NAME=createBoidsModule"
    "-s ENVIRONMENT=web,worker"
    "-s ALLOW_MEMORY_GROWTH=1"
    "-s EXPORTED_FUNCTIONS=_main,_setSpeciesParams,_setFlockSize,_update,_build,_initBoids,_posPtr,_velPtr,_boidCount,_malloc,_free"
    "-s EXPORTED_RUNTIME_METHODS=ccall,cwrap,HEAPF32"
    "--bind"
)

# デバッグビルド用のフラグ
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    list(APPEND EMCC_FLAGS
        "-O0"
        "-g"
        "-gsource-map"
        "--profiling-funcs"
        "--source-map-base=/static/js/"
        "-s ASSERTIONS=2"
        "-sSTACK_OVERFLOW_CHECK=2"
    )
endif()

# リリースビルド用のフラグ
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    list(APPEND EMCC_FLAGS
        "-O3"
        "-s EXPORT_ES6=1"
    )
endif()

# 実際のターゲットを設定
add_executable(wasm_boids ${SOURCES})
set_target_properties(wasm_boids PROPERTIES
    OUTPUT_NAME "wasm_boids"
    SUFFIX ".js"
    LINK_FLAGS "${EMCC_FLAGS}"
)