cmake_minimum_required(VERSION 3.0.2)
project(unit_tests)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  cv_bridge
  image_transport
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  actionlib
  control_msgs
  tf
  angles
  message_generation

)

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY
  robotNavigationTest/msg   
  FILES
  Goal.msg
#   Message1.msg
#   Message2.msg
)

add_service_files(
  DIRECTORY
  robotNavigationTest/srv
  FILES
  set_goal.srv
  # service_file.srv
)


## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  geometry_msgs
  nav_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS robotNavigationTest/include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp roslib std_msgs nav_msgs geometry_msgs sensor_msgs tf message_runtime
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp roslib std_msgs nav_msgs geometry_msgs sensor_msgs image_transport cv_bridge tf angles message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  robotNavigationTest/include
)

add_subdirectory(robotNavigationTest)
