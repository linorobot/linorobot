cmake_minimum_required(VERSION 2.8.3)
project(linorobot)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp 
    rospy 
    tf
    nav_msgs 
    lino_msgs 
    geometry_msgs 
    sensor_msgs 
    std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES linorobot
  CATKIN_DEPENDS roscpp rospy tf nav_msgs lino_msgs geometry_msgs sensor_msgs std_msgs 
  DEPENDS system_lib
)
###########
## Build ###include "lino_base.h"

###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(lino_base src/lino_base.cpp)
add_executable(lino_base_node src/lino_base_node.cpp)
target_link_libraries(lino_base_node lino_base ${catkin_LIBRARIES})
