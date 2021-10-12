cmake_minimum_required(VERSION 3.0)

project(MAIN)

set(CMAKE_BUILD_TYPE Debug)     # 要启用F5调试一定要用这一条

# add_executable(main main.cpp)

# add_subdirectory(VRP)   # 构建动态链接库

link_directories(lib)

add_executable(main main.cpp)

target_link_libraries(main libVRP.so)


