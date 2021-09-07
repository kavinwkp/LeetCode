cmake_minimum_required(VERSION 3.0.0)
project(MAIN)

set(CMAKE_BUILD_TYPE Debug)
set(OpenCV_DIR "~/Library/opencv-3.3.1/build/installed/share/OpenCV/")
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)

# message("${OpenCV_INCLUDE_DIRS}")
# message("${OpenCV_LIBRARY_DIRS}")
# message("${OpenCV_LIBS}")

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${realsense2_INCLUDE_DIRS})

add_executable(main main.cpp)

target_link_libraries(main ${OpenCV_LIBS} ${realsense2_LIBRARY})