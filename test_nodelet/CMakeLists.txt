project(test_nodelet)
catkin_package()

find_package(Boost)
include_directories(SYSTEM {$BOOST_INCLUDE_DIRS})

#common commands for building c++ executables and libraries
add_library(${PROJECT_NAME} src/plus.cpp src/console_tests.cpp)

#rosbuild_add_rostest(test_local.launch)
#rosbuild_add_rostest(test_loader.launch)

catkin_add_gtest(test_callback_queue_manager src/test_callback_queue_manager.cpp)
target_link_libraries(test_callback_queue_manager ${BOOST_LIBRARIES})
catkin_add_gtest(test_console test/test_console.cpp)

#rosbuild_add_rostest(test/test_console.launch)

# Not a real test. Tries to measure overhead of CallbackQueueManager.
add_executable(benchmark src/benchmark.cpp)
target_link_libraries(benchmark ${BOOST_LIBRARIES})
