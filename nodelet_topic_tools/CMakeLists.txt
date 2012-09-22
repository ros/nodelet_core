project(nodelet_topic_tools)
catkin_package()

find_package(catkin REQUIRED dynamic_reconfigure)

# Dynamic reconfigure
if (FALSE)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()
endif()

add_library(test_nodelet_topic_tools test/string_nodelet_throttle.cpp)

#rosbuild_add_rostest(test/test_nodelet_throttle.launch)
