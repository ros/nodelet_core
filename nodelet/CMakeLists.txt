project(nodelet)
catkin_package()
catkin_package_export(INCLUDE_DIRS include)

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

#uncomment if you have defined services
find_package(catkin REQUIRED std_msgs)
find_package(Boost)
add_service_files(DIRECTORY srv FILES NodeletList.srv  NodeletLoad.srv  NodeletUnload.srv)

generate_messages(DEPENDENCIES std_msgs)

include_directories(SYSTEM ${BOOST_INCLUDE_DIRS})
include_directories(include)

# Debug only, collects stats on how callbacks are doled out to worker threads
#add_definitions(-DNODELET_QUEUE_DEBUG)

add_executable(nodelet src/nodelet.cpp)
target_link_libraries(nodelet ${BOOST_LIBRARIES}
                              nodeletlib
                              uuid
)

add_library(nodeletlib src/nodelet_class.cpp src/loader.cpp src/callback_queue.cpp src/callback_queue_manager.cpp)
target_link_libraries(nodeletlib ${BOOST_LIBRARIES})

# See: https://code.ros.org/trac/ros-pkg/ticket/5144
if(APPLE)
  set_target_properties(nodeletlib PROPERTIES LINK_FLAGS "-undefined dynamic_lookup")
endif(APPLE)
