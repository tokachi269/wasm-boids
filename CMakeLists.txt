# CMakeの最低バージョンを指定
cmake_minimum_required(VERSION 3.20)

# プロジェクト名と使用する言語を指定
project(wasm_boids LANGUAGES CXX)

# Emscriptenを使用するための設定
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ビルドタイプを設定
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

# emcc用のコンパイルオプションを設定
set(EMCC_COMPILE_FLAGS
    "-pthread" # スレッドを有効化
    "-msimd128" # SIMDを有効化（必要に応じて）
    "-matomics" # Atomicsを有効化
    "-mbulk-memory" # Bulk Memoryを有効化
)

# emcc用のリンクオプションを設定
set(EMCC_LINK_FLAGS
    "-sWASM=1"
    "-sEXPORT_ES6=1"
    "-sMODULARIZE=1"
    "-sEXPORT_NAME=createBoidsModule"
    "-sENVIRONMENT=web,worker"
    "-sALLOW_MEMORY_GROWTH=1"
    "-sEXPORTED_FUNCTIONS=['_main','__emscripten_thread_crashed','_setSpeciesParams','_setFlockSize','_update','_build','_initBoids','_posPtr','_velPtr','_boidCount','_malloc','_free']"
    "-sEXPORTED_RUNTIME_METHODS=['ccall','cwrap','HEAPF32']"
    "--bind"
    "-pthread" # スレッドを有効化
    "-sUSE_PTHREADS=1" # POSIXスレッドを有効化
    "-sPROXY_TO_PTHREAD=1" # main関数をスレッドに移動
    "-sPTHREAD_POOL_SIZE=1" # CPUコア数に基づくスレッドプールサイズ navigator.hardwareConcurrency
    "-msimd128" # SIMDを有効化（必要に応じて）
    "-matomics" # Atomicsを有効化
    "-mbulk-memory" # Bulk Memoryを有効化
    "-sINCOMING_MODULE_JS_API=['locateFile']"
    "-sDISABLE_EXCEPTION_CATCHING=0"
    "-sINITIAL_MEMORY=67108864" # 初期メモリサイズ（16MB）
    "-sMAXIMUM_MEMORY=2147483648"   # 最大メモリサイズ（2GB）
    "-sALLOW_MEMORY_GROWTH=1"   # メモリの動的拡張を許可
    "-sSTACK_SIZE=2097152" # スタックサイズを1MBに設定
    "-sEXCEPTION_CATCHING_ALLOWED=['*']"
    "-sASSERTIONS=1"
    "-sSTACK_OVERFLOW_CHECK=2"
    "-sEXIT_RUNTIME=0"
)

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
)

# 実際のターゲットを設定
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/src/wasm/build")
add_executable(wasm_boids ${SOURCES})

# コンパイルオプションを設定
target_compile_options(wasm_boids PRIVATE ${EMCC_COMPILE_FLAGS})

# 出力形式の設定
set_target_properties(wasm_boids PROPERTIES
    OUTPUT_NAME "wasm_boids"
    SUFFIX ".js"
)

# リンクオプションを設定
target_link_options(wasm_boids PRIVATE ${EMCC_LINK_FLAGS})