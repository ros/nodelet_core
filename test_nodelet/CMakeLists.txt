cmake_minimum_required(VERSION 2.8)
project(test_nodelet)

find_package(catkin REQUIRED)
catkin_package()

find_package(Boost REQUIRED thread)
find_package(catkin REQUIRED nodelet pluginlib rostest)
include_directories(SYSTEM ${BOOST_INCLUDE_DIRS}
                           ${catkin_INCLUDE_DIRS}
)

#common commands for building c++ executables and libraries
add_library(${PROJECT_NAME} src/plus.cpp src/console_tests.cpp)
target_link_libraries(${PROJECT_NAME} ${BOOST_LIBRARIES}
                                      ${catkin_LIBRARIES}
)

add_rostest(test_local.launch)
add_rostest(test_loader.launch)

catkin_add_gtest(test_callback_queue_manager src/test_callback_queue_manager.cpp)
target_link_libraries(test_callback_queue_manager ${BOOST_LIBRARIES} ${catkin_LIBRARIES})

add_executable(test_console EXCLUDE_FROM_ALL test/test_console.cpp)
target_link_libraries(test_console ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
add_dependencies(tests test_console)

add_rostest(test/test_console.launch)

# Not a real test. Tries to measure overhead of CallbackQueueManager.
add_executable(benchmark src/benchmark.cpp)
target_link_libraries(benchmark ${BOOST_LIBRARIES}
                                ${PROJECT_NAME}
)
