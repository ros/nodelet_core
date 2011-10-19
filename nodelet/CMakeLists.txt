cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

# Nodelet is known not to build and run correctly on 
# OS X Lion 10.7.x using the llvm.  Use gcc-4.2 instead.
# See: https://code.ros.org/trac/ros-pkg/ticket/5144
IF(${CMAKE_SYSTEM} MATCHES "Darwin-11.*")
  set(CMAKE_CXX_COMPILER "/usr/bin/g++-4.2")
  set(CMAKE_C_COMPILER "/usr/bin/gcc-4.2")
ENDIF(${CMAKE_SYSTEM} MATCHES "Darwin-11.*")

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

rosbuild_add_boost_directories()

rosbuild_add_executable(nodelet src/nodelet.cpp)
target_link_libraries (nodelet nodeletlib uuid)
rosbuild_link_boost (nodelet filesystem system)

rosbuild_add_library(nodeletlib src/nodelet_class.cpp src/loader.cpp src/callback_queue.cpp src/callback_queue_manager.cpp)
rosbuild_link_boost(nodeletlib thread)

# See: https://code.ros.org/trac/ros-pkg/ticket/5144
if(APPLE)
  set_target_properties(nodeletlib PROPERTIES LINK_FLAGS "-undefined dynamic_lookup")
endif(APPLE)

target_link_libraries (nodeletlib)
