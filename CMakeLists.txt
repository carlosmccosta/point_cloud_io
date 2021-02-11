cmake_minimum_required(VERSION 3.0.2)
project(point_cloud_io)

## Use C++14
add_definitions(--std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    geometry_msgs
    pcl_ros
    roscpp
    sensor_msgs
    tf2
    tf2_ros
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    include
#  LIBRARIES
  CATKIN_DEPENDS
    geometry_msgs
    pcl_ros
    roscpp
    sensor_msgs
    tf2
    tf2_ros
#  DEPENDS
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

## Libraries
add_library(
  read_lib
  src/Read.cpp
)

add_library(
  write_lib
  src/Write.cpp
)

## Declare cpp executables
add_executable(
  read
  src/read_node.cpp
)
add_executable(
  write
  src/write_node.cpp
)

## Specify libraries to link executable targets against
target_link_libraries(
  read_lib
  write_lib
  ${catkin_LIBRARIES}
)
target_link_libraries(
  write_lib
  ${catkin_LIBRARIES}
)

target_link_libraries(
  read
  read_lib
  write_lib
  ${catkin_LIBRARIES}
)
target_link_libraries(
  write
  write_lib
  ${catkin_LIBRARIES}
)
