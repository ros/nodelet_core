cmake_minimum_required(VERSION 2.8)
project(nodelet)

find_package(catkin REQUIRED)

find_package(Boost REQUIRED)
find_package(catkin REQUIRED bondcpp rosconsole roscpp)
catkin_package(INCLUDE_DIRS include
               LIBRARIES nodeletlib
               DEPENDS Boost bondcpp rosconsole roscpp
)

# Nodelet is known not to build and run correctly on 
# OS X Lion 10.7.x using the llvm.  Use gcc-4.2 instead (if available).
# See: https://code.ros.org/trac/ros-pkg/ticket/5144
if(${CMAKE_SYSTEM} MATCHES "Darwin-11.*")
  if(EXISTS "/usr/bin/g++-4.2" AND EXISTS "/usr/bin/gcc-4.2")
    set(CMAKE_CXX_COMPILER /usr/bin/g++-4.2)
    set(CMAKE_C_COMPILER /usr/bin/gcc-4.2)
  else()
    # If there is no g++-4.2 or gcc-4.2 use clang++ and clang
    set(CMAKE_CXX_COMPILER /usr/bin/clang++)
    set(CMAKE_C_COMPILER /usr/bin/clang)
  endif()
endif()

# Find common packages
find_package(Boost)
find_package(catkin REQUIRED bondcpp genmsg pluginlib rosconsole roscpp std_msgs)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS}
                           ${BOOST_INCLUDE_DIRS}
)
include_directories(include)

# Deal with the services
add_service_files(DIRECTORY srv FILES NodeletList.srv  NodeletLoad.srv  NodeletUnload.srv)

generate_messages(DEPENDENCIES std_msgs)

# Debug only, collects stats on how callbacks are doled out to worker threads
#add_definitions(-DNODELET_QUEUE_DEBUG)

add_library(nodeletlib src/nodelet_class.cpp src/loader.cpp src/callback_queue.cpp src/callback_queue_manager.cpp)
target_link_libraries(nodeletlib ${BOOST_LIBRARIES}
                                 ${catkin_LIBRARIES}
)
add_dependencies(nodeletlib ${PROJECT_NAME}_gencpp)

add_executable(nodelet src/nodelet.cpp)
target_link_libraries(nodelet ${BOOST_LIBRARIES}
                              ${catkin_LIBRARIES}
                              nodeletlib
)

# See: https://code.ros.org/trac/ros-pkg/ticket/5144
if(APPLE)
  set_target_properties(nodeletlib PROPERTIES LINK_FLAGS "-undefined dynamic_lookup")
endif(APPLE)

# install
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(TARGETS nodeletlib
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(TARGETS nodelet
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
