cmake_minimum_required(VERSION 3.0.0)
project(MAIN)

set(CMAKE_BUILD_TYPE Debug)
set(OpenCV_DIR "~/Library/opencv-3.3.1/build/installed/share/OpenCV/")
find_package(OpenCV REQUIRED)

message("${OpenCV_INCLUDE_DIRS}")
# /usr/include

message("${OpenCV_LIBRARY_DIRS}")
# /usr/include/opencv

# message("${OpenCV_LIBS}")

add_executable(main main.cpp)

target_link_libraries(main ${OpenCV_LIBS})