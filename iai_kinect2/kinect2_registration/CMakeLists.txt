cmake_minimum_required(VERSION 2.8.3)
project(kinect2_registration CXX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBT_USE_DOUBLE_PRECISION -Wall")
# Unused warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wuninitialized -Winit-self -Wunused-function -Wunused-label -Wunused-variable -Wunused-but-set-variable -Wunused-but-set-parameter")
# Additional warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Warray-bounds -Wtype-limits -Wreturn-type -Wsequence-point -Wparentheses -Wmissing-braces -Wchar-subscripts -Wswitch -Wwrite-strings -Wenum-compare -Wempty-body")# -Wlogical-op")

# Check for c++11 support
INCLUDE(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
IF(COMPILER_SUPPORTS_CXX11)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ELSEIF(COMPILER_SUPPORTS_CXX0X)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
  MESSAGE(ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
ENDIF()

# additional cmake modules
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(cmake_modules QUIET)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)
find_package(OpenMP)
find_package(Eigen3)
find_package(OpenCL)

if(OPENMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

############################################################
## Check for modules being build based on found libraries ##
############################################################

if(EIGEN3_FOUND)
  message(STATUS "CPU based depth registration enabled")
  include_directories(${EIGEN3_INCLUDE_DIR})
  set(DEPTH_REG_CPU ON)
  add_definitions(-DDEPTH_REG_CPU)
else()
  message(STATUS "CPU based depth registration disabled")
  set(DEPTH_REG_CPU OFF)
endif()

if(OpenCL_FOUND)
  message(STATUS "OpenCL based depth registration enabled")
  set(EXPORTED_DEPENDENCIES OpenCL)

  if(UNIX AND NOT APPLE)
    include(CheckOpenCLICDLoader)
    if(OpenCL_C_WORKS AND NOT OpenCL_CXX_WORKS)
      set(KINECT2_OPENCL_ICD_LOADER_IS_OLD 1)
      message(WARNING "Your libOpenCL.so is incompatible with CL/cl.h. Install ocl-icd-opencl-dev to update libOpenCL.so?")
    endif()
  endif()
  include_directories(${OpenCL_INCLUDE_DIRS})

  set(DEPTH_REG_OPENCL ON)
  add_definitions(-DDEPTH_REG_OPENCL)
else()
  message(STATUS "OpenCL based depth registration disabled")
  set(DEPTH_REG_OPENCL OFF)
endif()

if(NOT DEPTH_REG_CPU AND NOT DEPTH_REG_OPENCL)
  message(FATAL_ERROR "No registration method available!")
endif()

################################################
## Declare ROS messages, services and actions ##
################################################

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
  INCLUDE_DIRS include
  LIBRARIES kinect2_registration
#  CATKIN_DEPENDS other_catkin_pkg
  DEPENDS ${EXPORTED_DEPENDENCIES}
  CFG_EXTRAS kinect2_registration.cmake
)

###########
## Build ##
###########

# Display additional files in qtcreator
execute_process(COMMAND find ${PROJECT_SOURCE_DIR} -type f OUTPUT_VARIABLE FILES_STRING)
string(REPLACE "\n" ";" FILES_LIST ${FILES_STRING})
add_custom_target(additional_files_${PROJECT_NAME}
  SOURCES
  ${FILES_LIST}
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  "${CMAKE_CURRENT_SOURCE_DIR}/include/internal"
)

if(DEPTH_REG_CPU)
  set(MODULES ${MODULES} src/depth_registration_cpu.cpp)
endif()

if(DEPTH_REG_OPENCL)
  add_definitions(-DREG_OPENCL_FILE="${PROJECT_SOURCE_DIR}/src/depth_registration.cl")
  set(MODULES ${MODULES} src/depth_registration_opencl.cpp)
  set(MODULE_LIBS ${MODULE_LIBS} ${OpenCL_LIBRARIES})
endif()

add_library(kinect2_registration SHARED src/kinect2_registration.cpp ${MODULES})
target_link_libraries(kinect2_registration
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${MODULE_LIBS}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS kinect2_registration
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_kinect2_bridge.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
