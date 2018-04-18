cmake_minimum_required(VERSION 2.8.3)
project(thu_kinect2)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp std_msgs genmsg shm_transport tzc_transport
)
find_package(freenect2 REQUIRED HINTS "$ENV{HOME}/freenect2")
find_package(OpenCV REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES thu_kinect2
  CATKIN_DEPENDS roscpp
  DEPENDS rt
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${freenect2_INCLUDE_DIRS}
  include
)

add_executable(thu_kinect2 src/kinect2_bridge.cpp)
target_link_libraries(thu_kinect2
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${freenect2_LIBRARY}
  -lrt
)


#############
## Install ##
#############


#############
## Testing ##
#############


