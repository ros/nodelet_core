cmake_minimum_required(VERSION 2.8)
project(nodelet_topic_tools)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED dynamic_reconfigure message_filters nodelet pluginlib roscpp)
catkin_package(INCLUDE_DIRS include
               DEPENDS dynamic_reconfigure message_filters nodelet pluginlib roscpp
)

find_package(catkin REQUIRED dynamic_reconfigure nodelet rostest)

# Dynamic reconfigure
generate_cfg(cfg/NodeletThrottle.cfg)

# build the tool
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
include_directories(include)
add_library(test_nodelet_topic_tools test/string_nodelet_throttle.cpp)
add_dependencies(test_nodelet_topic_tools ${PROJECT_NAME}_gencfg)

# add test
add_rostest(test/test_nodelet_throttle.launch)

# install the useful stuff
install(DIRECTORY include/{PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
