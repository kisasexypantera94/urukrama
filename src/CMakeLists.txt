cmake_minimum_required(VERSION 3.17.3)

project(urukrama)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-O3)

add_executable(urukrama
    main.cpp
    utils.cpp
)
