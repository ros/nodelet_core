cmake_minimum_required(VERSION 3.1)
project(nodelet)

## Find catkin dependencies
find_package(catkin REQUIRED
  bondcpp
  cmake_modules
  message_generation
  pluginlib
  rosconsole
  roscpp
  std_msgs
)

catkin_package_xml()
# split version in parts and pass to extra file
string(REPLACE "." ";" nodelet_VERSION_LIST "${nodelet_VERSION}")
list(LENGTH nodelet_VERSION_LIST _count)
if(NOT _count EQUAL 3)
  message(FATAL_ERROR "nodelet version '${nodelet_VERSION}' does not match 'MAJOR.MINOR.PATCH' pattern")
endif()
list(GET nodelet_VERSION_LIST 0 nodelet_VERSION_MAJOR)
list(GET nodelet_VERSION_LIST 1 nodelet_VERSION_MINOR)
list(GET nodelet_VERSION_LIST 2 nodelet_VERSION_PATCH)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/include/nodelet/version.h.in ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/nodelet/version.h)

## Find Boost (only headers)
find_package(Boost REQUIRED)

## Find UUID libraries
find_package(UUID REQUIRED)

## Add service files to be generated
add_service_files(DIRECTORY srv FILES NodeletList.srv  NodeletLoad.srv  NodeletUnload.srv)

## Generate servics
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES nodeletlib
  CATKIN_DEPENDS bondcpp message_runtime pluginlib rosconsole roscpp std_msgs
  DEPENDS Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${BOOST_INCLUDE_DIRS}
  ${UUID_INCLUDE_DIRS})

# Debug only, collects stats on how callbacks are doled out to worker threads
#add_definitions(-DNODELET_QUEUE_DEBUG)

add_library(nodeletlib src/nodelet_class.cpp src/loader.cpp src/callback_queue.cpp src/callback_queue_manager.cpp)
target_link_libraries(nodeletlib ${catkin_LIBRARIES} ${BOOST_LIBRARIES})
add_dependencies(nodeletlib ${nodelet_EXPORTED_TARGETS})

add_executable(nodelet src/nodelet.cpp)
target_link_libraries(nodelet nodeletlib ${UUID_LIBRARIES} ${catkin_LIBRARIES} ${BOOST_LIBRARIES})

# install
catkin_install_python(PROGRAMS scripts/declared_nodelets scripts/list_nodelets
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(TARGETS nodeletlib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(TARGETS nodelet
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

