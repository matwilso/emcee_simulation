cmake_minimum_required(VERSION 2.8.3)
project(emcee_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
roscpp
joy
geometry_msgs
sensor_msgs
control_msgs
rosconsole
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)



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
#  INCLUDE_DIRS include
#  LIBRARIES emcee_control
#  CATKIN_DEPENDS ros_control ros_controllers
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(emcee_control
#   src/${PROJECT_NAME}/emcee_control.cpp
# )

## Declare a C++ executable
add_executable(joy_teleop src/joy_teleop.cpp)
target_link_libraries(joy_teleop ${catkin_LIBRARIES})
