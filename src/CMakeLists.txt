cmake_minimum_required(VERSION 3.17.3)

project(urukrama)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Eigen3 REQUIRED)
find_package(tsl-robin-map REQUIRED)

add_executable(urukrama
    main.cpp
    utils.cpp
)

target_link_libraries(urukrama Eigen3::Eigen tsl::robin_map)