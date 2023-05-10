cmake_minimum_required(VERSION 2.8.3)
project(lino_pid)

find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure message_generation roscpp std_msgs lino_msgs)

#add_message_files(
#  FILES
#)

generate_messages(
    DEPENDENCIES
    std_msgs
)

generate_dynamic_reconfigure_options(
    cfg/linoPID.cfg
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS dynamic_reconfigure message_runtime roscpp std_msgs lino_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

# Set the name to use for the executable.
set (BINNAME1 pid_configure)
set (BINNAME2 pid_listen)

# Set the source files to use with the executable.
set (SRCS1 ${SRCS1} src/lino_pid_core.cpp)
set (SRCS1 ${SRCS1} src/pid_configure.cpp)
set (SRCS2 ${SRCS2} src/lino_pid_core.cpp)
set (SRCS2 ${SRCS2} src/pid_listen.cpp)

# Build the executable that will be used to run this node.
add_executable (${BINNAME1} ${SRCS1})
target_link_libraries(${BINNAME1} ${catkin_LIBRARIES})
add_dependencies(${BINNAME1} ${PROJECT_NAME}_gencfg)
add_executable (${BINNAME2} ${SRCS2})
target_link_libraries(${BINNAME2} ${catkin_LIBRARIES})
add_dependencies(${BINNAME2} ${PROJECT_NAME}_gencfg)


install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
)

    install(TARGETS ${BINNAME1}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS ${BINNAME2}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
