cmake_minimum_required(VERSION 3.0)
project(MAIN)

add_definitions(-std=c++11)

set(CMAKE_BUILD_TYPE "Debug")

add_executable(main main.cpp)