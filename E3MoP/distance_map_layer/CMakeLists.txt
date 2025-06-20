cmake_minimum_required(VERSION 3.0.2)
project(distance_map_layer)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  dynamic_reconfigure
  roscpp
)

# OpenCV
find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
  message(STATUS ${OpenCV_INCLUDE_DIRS})
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES distance_map_layer
  CATKIN_DEPENDS costmap_2d dynamic_reconfigure roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(distance_map_layer src/distance_map_layer.cpp)
target_link_libraries(distance_map_layer ${catkin_LIBRARIES} ${OpenCV_LIBS})
