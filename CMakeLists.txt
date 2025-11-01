# CMakeの最低バージョンを指定
cmake_minimum_required(VERSION 3.20)

# プロジェクト名と使用する言語を指定
project(wasm_boids LANGUAGES CXX)

# Emscriptenを使用するための設定
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# WASM成果物の出力先ディレクトリ（dev/prod切替可能）
set(WASM_RUNTIME_OUTPUT_DIR "${CMAKE_SOURCE_DIR}/src/wasm/build" CACHE PATH "Output directory for generated WebAssembly artifacts")
get_filename_component(WASM_RUNTIME_OUTPUT_DIR "${WASM_RUNTIME_OUTPUT_DIR}" ABSOLUTE BASE_DIR "${CMAKE_SOURCE_DIR}")
file(MAKE_DIRECTORY "${WASM_RUNTIME_OUTPUT_DIR}")

# ビルドタイプを設定
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

# emcc用のコンパイルオプションを設定
set(EMCC_COMPILE_FLAGS
    "-O0"          # 最大最適化
    "-g"
    "-pthread" # スレッドを有効化
    "-msimd128" # SIMDを有効化（必要に応じて）
    "-matomics" # Atomicsを有効化
    "-mbulk-memory" # Bulk Memoryを有効化
)

# emcc用のリリース（Release）向けコンパイルオプション
set(EMCC_COMPILE_FLAGS_RELEASE
    "-O3"          # 最大最適化
    "-DNDEBUG"     # assert 無効化
    "-msimd128"
    "-matomics"
    "-mbulk-memory"
)

# emcc用のリンクオプションを設定
set(EMCC_LINK_FLAGS
    "-sWASM=1"
    "-sEXPORT_ES6=1"
    "-sMODULARIZE=1"
    "-sEXPORT_NAME=createBoidsModule"
    "-sENVIRONMENT=web,worker,node"
    "-sALLOW_MEMORY_GROWTH=1"
    "-sEXPORTED_FUNCTIONS=['_main','_boidUnitMappingPtr','_speciesIdsPtr','_syncReadToWriteBuffers','_oriPtr','_setSpeciesParams','_setFlockSize','_update','_build','_posPtr','_velPtr','_boidCount','_stepSimulation','_currentFirstBoidX','_malloc','_free']"
    "-sEXPORTED_RUNTIME_METHODS=['ccall','cwrap','HEAPF32','HEAP32','HEAPU8']"
    "--bind"
    "-pthread" # スレッドを有効化
    "-sUSE_PTHREADS=1" # POSIXスレッドを有効化
    "-sPROXY_TO_PTHREAD=1" # main関数をスレッドに移動
    "-sPTHREAD_POOL_SIZE=navigator.hardwareConcurrency" # CPUコア数に基づくスレッドプールサイズ
    "-msimd128" # SIMDを有効化（必要に応じて）
    "-matomics" # Atomicsを有効化
    "-mbulk-memory" # Bulk Memoryを有効化
    "-sINCOMING_MODULE_JS_API=['locateFile']"
    "-sINITIAL_MEMORY=67108864" # 初期メモリサイズ（64MB）
    "-sMAXIMUM_MEMORY=2147483648"   # 最大メモリサイズ（2GB）
    "-sALLOW_MEMORY_GROWTH=1"   # メモリの動的拡張を許可
    "-sSTACK_SIZE=2097152" # スタックサイズを1MBに設定
    "-sEXCEPTION_CATCHING_ALLOWED=['*']"
    "-sASSERTIONS=1"
    "-gsource-map"
    "--profiling-funcs"
    "--source-map-base=/static/js/"
)

# emcc用のリリース（Release）向けリンクオプション
set(EMCC_LINK_FLAGS_RELEASE
    "-sWASM=1"
    "-sEXPORT_ES6=1"
    "-sMODULARIZE=1"
    "-sEXPORT_NAME=createBoidsModule"
    "-sENVIRONMENT=web,worker,node"
    "-sALLOW_MEMORY_GROWTH=1"
    "-sEXPORTED_FUNCTIONS=['_main','_boidUnitMappingPtr','_speciesIdsPtr','_syncReadToWriteBuffers','_oriPtr','_setSpeciesParams','_setFlockSize','_update','_build','_posPtr','_velPtr','_boidCount','_stepSimulation','_currentFirstBoidX','_malloc','_free']"
    "-sEXPORTED_RUNTIME_METHODS=['ccall','cwrap','HEAPF32','HEAP32','HEAPU8']"
    "--bind"
    "-pthread"
    "-sUSE_PTHREADS=1"
    "-sPROXY_TO_PTHREAD=1"
    "-sPTHREAD_POOL_SIZE=navigator.hardwareConcurrency"
    "-msimd128"
    "-matomics"
    "-mbulk-memory"
	"-sINCOMING_MODULE_JS_API=['locateFile']"
    "-sINITIAL_MEMORY=33554432"
    "-sMAXIMUM_MEMORY=2147483648"
	"-sALLOW_MEMORY_GROWTH=1"
    "-O3"
    "-sASSERTIONS=1"
)


# 共通ソース
set(COMMON_SOURCES
    src/wasm/boid_factory.cpp
    src/wasm/entry.cpp
    src/wasm/boid_unit.cpp
    src/wasm/boids_tree.cpp
    src/wasm/lbvh_index.cpp
    src/wasm/main.cpp
    src/wasm/simulation_tuning.cpp
)

# インクルードディレクトリを指定
include_directories(
    D:/GitHub/vcpkg/installed/x64-windows/include
)

if (CMAKE_SYSTEM_NAME STREQUAL "Emscripten")
    add_executable(wasm_boids
        ${COMMON_SOURCES}
        src/wasm/wasm_bindings.cpp
    )

    set_target_properties(wasm_boids PROPERTIES
        OUTPUT_NAME "wasm_boids"
        SUFFIX ".js"
        RUNTIME_OUTPUT_DIRECTORY "${WASM_RUNTIME_OUTPUT_DIR}"
    )

    target_compile_options(wasm_boids PRIVATE
        $<$<CONFIG:Debug>:${EMCC_COMPILE_FLAGS}>
        $<$<CONFIG:Release>:${EMCC_COMPILE_FLAGS_RELEASE}>
    )

    # Thread pooling handled in source; no compile-time toggle required

    target_link_options(wasm_boids PRIVATE
        $<$<CONFIG:Debug>:${EMCC_LINK_FLAGS}>
        $<$<CONFIG:Release>:${EMCC_LINK_FLAGS_RELEASE}>
    )
else()
    add_executable(wasm_boids_native
        ${COMMON_SOURCES}
        src/wasm/native_simulation.cpp
    )

    target_compile_definitions(wasm_boids_native PRIVATE
        $<$<CONFIG:Debug>:DEBUG>
    )

    target_compile_options(wasm_boids_native PRIVATE
        $<$<CXX_COMPILER_ID:MSVC>:/W4 /permissive->
        $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>
    )

    set_target_properties(wasm_boids_native PROPERTIES
        OUTPUT_NAME "wasm_boids_native"
    )

    find_package(Threads REQUIRED)
    target_link_libraries(wasm_boids_native PRIVATE Threads::Threads)
endif()