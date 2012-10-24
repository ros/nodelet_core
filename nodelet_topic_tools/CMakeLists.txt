cmake_minimum_required(VERSION 2.8)
project(nodelet_topic_tools)

find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure message_filters nodelet pluginlib roscpp rostest)
catkin_package(INCLUDE_DIRS include
               DEPENDS dynamic_reconfigure message_filters nodelet pluginlib roscpp
)

# Dynamic reconfigure
generate_dynamic_reconfigure_options(cfg/NodeletThrottle.cfg)

# build the tool
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
include_directories(include)
add_library(test_nodelet_topic_tools test/string_nodelet_throttle.cpp)
add_dependencies(test_nodelet_topic_tools ${PROJECT_NAME}_gencfg)

# add test
add_rostest(test/test_nodelet_throttle.launch)

# install the useful stuff
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
