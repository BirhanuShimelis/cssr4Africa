cmake_minimum_required(VERSION 3.0.2)
project(cssr_system)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  image_transport
  cv_bridge
  tf
  angles
  actionlib
  control_msgs
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
add_service_files(
  DIRECTORY
  robotLocalization/srv
  FILES
  SetPose.srv
  
)

add_service_files(
  DIRECTORY
  robotNavigation/srv
  FILES
  set_goal.srv
  
)


# Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY
  robotNavigation/msg
  FILES
  Goal.msg
  # Message1.
  # Message2.msg
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
)

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
  INCLUDE_DIRS robotLocalization robotNavigation
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
  robotLocalization/include
  robotNavigation/include
)

## Add Subdirectories for robotLocalization and robotNavigation
add_subdirectory(robotLocalization)
add_subdirectory(robotNavigation)


###########
## Install ##
########### 

# ## Install data files for robotNavigation
# install(DIRECTORY robotNavigation/data/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/robotNavigation/data
# )


