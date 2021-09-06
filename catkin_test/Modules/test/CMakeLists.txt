cmake_minimum_required(VERSION 3.0.2)
project(test)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(OpenCV_DIR "~/Library/opencv-3.3.1/build/installed/share/OpenCV")
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  test_msgs
  image_transport
  geometry_msgs
  # sensor_msgs
  cv_bridge
  OpenCV
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


################################################
## Declare ROS messages, services and actions ##
################################################


## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )


###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/test.cpp
# )
add_library(simple
  include/test/simple.h
  src/simple.cpp
)



add_executable(talker src/talker.cpp)
target_link_libraries(talker simple ${catkin_LIBRARIES})
add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_executable(att src/att.cpp)
target_link_libraries(att ${catkin_LIBRARIES})

add_executable(image_pub src/image_pub.cpp)
target_link_libraries(image_pub ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_executable(image_sub src/image_sub.cpp)
target_link_libraries(image_sub ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_executable(web_cam src/web_cam.cpp)
target_link_libraries(web_cam ${catkin_LIBRARIES} ${OpenCV_LIBS})


#############
## Install ##
#############



install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)