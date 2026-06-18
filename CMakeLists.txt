cmake_minimum_required(VERSION 3.24)
project(jomon_period VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(JOMON_BUILD_VIEWER "Build Open3D-based 3D viewer" OFF)
option(JOMON_BUILD_TESTS  "Build unit tests"              ON)

# ---------------------------------------------------------------------------
# happly (header-only PLY I/O, not in vcpkg)
# ---------------------------------------------------------------------------
include(FetchContent)
FetchContent_Declare(
    happly
    GIT_REPOSITORY https://github.com/nmwsharp/happly.git
    GIT_TAG        master
    GIT_SHALLOW    TRUE
)
FetchContent_GetProperties(happly)
if(NOT happly_POPULATED)
    FetchContent_Populate(happly)
endif()

# ---------------------------------------------------------------------------
# vcpkg-managed dependencies
# ---------------------------------------------------------------------------
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(fmt    REQUIRED)

# ---------------------------------------------------------------------------
# Compiler flags
# ---------------------------------------------------------------------------
if(MSVC)
    add_compile_options(/utf-8 /W4 /wd4127 /wd4244 /wd4267)
    add_compile_definitions(_USE_MATH_DEFINES)   # expose M_PI on MSVC
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------------------------------
# Core static library (M1-M4 pipeline; no GUI dependency)
# ---------------------------------------------------------------------------
add_library(jomon_core STATIC
    src/ply_io.cpp
    src/face_extract.cpp
    src/height_map.cpp
    src/detrend.cpp
    src/autocorr2d.cpp
    src/period_estimator.cpp
)
target_include_directories(jomon_core PUBLIC
    include
    ${happly_SOURCE_DIR}
)
target_link_libraries(jomon_core PUBLIC
    Eigen3::Eigen
    fmt::fmt
)

# ---------------------------------------------------------------------------
# CLI: jomon_period (M4 – final goal)
# ---------------------------------------------------------------------------
add_executable(jomon_period apps/jomon_period.cpp)
target_link_libraries(jomon_period PRIVATE jomon_core)

# ---------------------------------------------------------------------------
# Optional: Open3D viewer (M1 visualisation, JOMON_BUILD_VIEWER=ON)
# ---------------------------------------------------------------------------
if(JOMON_BUILD_VIEWER)
    FetchContent_Declare(
        polyscope
        GIT_REPOSITORY https://github.com/nmwsharp/polyscope.git
        GIT_TAG        v2.3.0
    )
    FetchContent_MakeAvailable(polyscope)

    add_executable(jomon_viewer apps/jomon_viewer.cpp)
    target_link_libraries(jomon_viewer PRIVATE jomon_core polyscope)
endif()

# ---------------------------------------------------------------------------
# Tests (M3 validation with synthetic data)
# ---------------------------------------------------------------------------
if(JOMON_BUILD_TESTS)
    enable_testing()
    add_executable(test_synthetic tests/synthetic_periodic.cpp)
    target_link_libraries(test_synthetic PRIVATE jomon_core)
    add_test(NAME synthetic_periodic COMMAND test_synthetic)
endif()
