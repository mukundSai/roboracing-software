cmake_minimum_required(VERSION 2.8.3)
project(iarrc)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  rr_platform
  cv_bridge
  pcl_ros
  sick_tim
  laser_geometry
  image_transport
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED filters)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs rr_platform
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

add_library(iarrc src/color_detector/color_detector.cpp)
target_link_libraries(iarrc ${catkin_LIBRARIES})

add_subdirectory(src/stoplight_watcher)
add_subdirectory(src/navigation_controller)
add_subdirectory(src/finish_line_watcher)
add_subdirectory(src/qualify)
add_subdirectory(src/ol_reliable)
