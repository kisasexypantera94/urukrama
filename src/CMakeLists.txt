cmake_minimum_required(VERSION 3.28.3)

project(urukrama)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_FLAGS_RELEASE "-Ofast")

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/../externals/sanitizers-cmake/cmake" ${CMAKE_MODULE_PATH})
# set(CMAKE_CXX_CLANG_TIDY "clang-tidy")

# Conan
find_package(Eigen3 REQUIRED)
find_package(tsl-robin-map REQUIRED)
find_package(Boost REQUIRED)
find_package(spdlog REQUIRED)

# apt
find_package(BLAS REQUIRED)
find_package(OpenMP REQUIRED)

# submodules
find_package(Sanitizers REQUIRED)

add_executable(urukrama
    main.cpp
    utils.cpp
    graph.cpp
    faiss.cpp
)

target_link_libraries(urukrama
    spdlog::spdlog_header_only
    Eigen3::Eigen
    tsl::robin_map
    boost::boost
    faiss
    OpenMP::OpenMP_CXX
    ${BLAS_LIBRARIES}
)

target_link_directories(urukrama
    PUBLIC
    ${CMAKE_LIBRARY_PATH}
)

target_include_directories(urukrama
    PUBLIC
    ${CMAKE_INCLUDE_PATH}
)

add_sanitizers(urukrama)