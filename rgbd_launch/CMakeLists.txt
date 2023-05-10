cmake_minimum_required(VERSION 3.0.2)
project(rgbd_launch)

find_package(catkin)

catkin_package()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)
  roslaunch_add_file_check(launch)
  roslaunch_add_file_check(test)
  ## This test requires hardware, which is not available on CI, so commented out.
  ## Comment this line in when you're developing with a physical sensor attached.
  #add_rostest(test/camera.test)
endif()
