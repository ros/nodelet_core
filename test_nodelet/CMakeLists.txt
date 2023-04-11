cmake_minimum_required(VERSION 3.0.2)

project(test_nodelet)

find_package(catkin REQUIRED)
catkin_package()

if(CATKIN_ENABLE_TESTING)
  find_package(Boost REQUIRED thread)
  find_package(catkin REQUIRED bond nodelet pluginlib rostest)
  include_directories(SYSTEM ${BOOST_INCLUDE_DIRS}
                             ${catkin_INCLUDE_DIRS}
  )

  # find pthread and provide it as Threads::Threads imported target
  set(THREADS_PREFER_PTHREAD_FLAG ON)
  find_package(Threads REQUIRED)

  #common commands for building c++ executables and libraries
  add_library(${PROJECT_NAME} src/plus.cpp src/console_tests.cpp src/failing_nodelet.cpp src/nodehandles.cpp src/request_stop.cpp)
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

  # TODO(mjcarroll):Remove these tests until
  # https://github.com/ros/rosconsole/issues/30 is resolved
  # add_rostest(test/test_bond_break_on_shutdown.launch)
  # add_rostest(test/test_unload_called_twice.launch)

  add_rostest_gtest(test_nodehandles_different_namespaces
                    test/test_nodehandles_different_namespaces.test
                    test/test_nodehandles_different_namespaces.cpp)
  target_link_libraries(test_nodehandles_different_namespaces ${catkin_LIBRARIES})

  add_rostest_gtest(test_nodehandles_same_namespaces
                    test/test_nodehandles_same_namespaces.test
                    test/test_nodehandles_same_namespaces.cpp)
  target_link_libraries(test_nodehandles_same_namespaces ${catkin_LIBRARIES})

  add_rostest_gtest(test_nodehandles_manager_namespaced
                    test/test_nodehandles_manager_namespaced.test
                    test/test_nodehandles_manager_namespaced.cpp)
  target_link_libraries(test_nodehandles_manager_namespaced ${catkin_LIBRARIES})

  # Not a real test. Tries to measure overhead of CallbackQueueManager.
  add_executable(benchmark src/benchmark.cpp)
  target_link_libraries(benchmark ${BOOST_LIBRARIES}
                                  ${PROJECT_NAME}
  )

  add_executable(create_instance_cb_error src/create_instance_cb_error.cpp)
  target_link_libraries(create_instance_cb_error ${catkin_LIBRARIES})

  add_rostest_gtest(test_nodelet_request_stop test/test_nodelet_request_stop.test test/test_nodelet_request_stop.cpp)
  target_link_libraries(test_nodelet_request_stop ${BOOST_LIBRARIES} ${catkin_LIBRARIES} Threads::Threads)

  add_rostest(test/test_request_stop_unload.test DEPENDENCIES ${PROJECT_NAME})
  add_rostest(test/test_request_stop_unload_standalone.test DEPENDENCIES ${PROJECT_NAME})
  add_rostest(test/test_request_stop_unload_kill_manager.test DEPENDENCIES ${PROJECT_NAME})
endif()
