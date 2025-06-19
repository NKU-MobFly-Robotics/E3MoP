cmake_minimum_required(VERSION 3.0.2)
project(ceres_path_smoother)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  base_local_planner
  costmap_2d
  distance_map_layer
  geometry_msgs
  lattice_path_planner
  nav_core
  nav_msgs
  path_smoother_base
  roscpp
  teb_local_planner
  tf
  tf2
)

# Ceres
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ceres_path_smoother
  CATKIN_DEPENDS
    base_local_planner
    costmap_2d
    distance_map_layer
    geometry_msgs
    lattice_path_planner
    nav_core
    nav_msgs
    path_smoother_base
    roscpp
    teb_local_planner
    tf
    tf2
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS} 
)

add_library(ceres_path_smoother src/ceres_local_planner_ros.cpp src/ceres_path_smoother_ros.cpp src/ceres_path_smoother.cpp)
target_link_libraries(ceres_path_smoother ${catkin_LIBRARIES} ${CERES_LIBRARIES} ${OpenCV_LIBS})
