cmake_minimum_required(VERSION 3.0.2)
project(path_smoother_base)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  geometry_msgs
  nav_core
  nav_msgs
  roscpp
  tf2
)

# Glog
# find_package(glog REQUIRED)

catkin_package(
  INCLUDE_DIRS include third_party
  LIBRARIES path_smoother_base
  CATKIN_DEPENDS
    costmap_2d
    geometry_msgs
    nav_core
    nav_msgs
    roscpp
    tf2
)

include_directories(
  include
  third_party
  ${catkin_INCLUDE_DIRS}
)

add_library(path_smoother_base
  src/local_planner_util.cpp
  src/path_smoother_base.cpp
  src/spline_interpolation.cpp
  src/utils.cpp
)
target_link_libraries(path_smoother_base ${catkin_LIBRARIES})
