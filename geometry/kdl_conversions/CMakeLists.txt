cmake_minimum_required(VERSION 3.0.2)
project(kdl_conversions)

find_package(catkin REQUIRED geometry_msgs)
find_package(orocos_kdl REQUIRED)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS geometry_msgs
  DEPENDS orocos_kdl
)

include_directories(include ${catkin_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})

add_library(${PROJECT_NAME}
  src/kdl_msg.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
