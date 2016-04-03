cmake_minimum_required(VERSION 2.8.3)

project(test_nodelet_topic_tools)

find_package(catkin REQUIRED COMPONENTS message_filters nodelet nodelet_topic_tools pluginlib roscpp rostest)

catkin_package(
  CATKIN_DEPENDS message_filters nodelet nodelet_topic_tools pluginlib roscpp
)

if(CATKIN_ENABLE_TESTING)
  include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
  add_library(test_nodelet_topic_tools test/string_nodelet_throttle.cpp)
  target_link_libraries(test_nodelet_topic_tools ${catkin_LIBRARIES})
  add_dependencies(test_nodelet_topic_tools ${catkin_EXPORTED_TARGETS})

  add_rostest(test/test_nodelet_throttle.launch)
endif()
