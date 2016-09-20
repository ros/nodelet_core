^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package test_nodelet
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.9.6 (2016-09-20)
------------------

1.9.5 (2016-06-22)
------------------

1.9.4 (2016-03-15)
------------------
* add test dependency on rosbash
  http://build.ros.org/job/Jdev__nodelet_core__ubuntu_trusty_amd64/3/
  should fix this
* update maintainer
* Contributors: Mikael Arguedas

1.9.3 (2015-08-05)
------------------
* adding support for named nodelet loggers
* Contributors: Tully Foote

1.9.2 (2014-10-30)
------------------

1.9.1 (2014-10-29)
------------------
* test_nodelet: test for `#23 <https://github.com/ros/nodelet_core/issues/23>`_ (display helpful error msg)
* Contributors: Max Schwarz

1.9.0 (2014-06-16)
------------------

1.8.3 (2014-05-08)
------------------
* update changelogs
* Update maintainer field
* Contributors: Dirk Thomas, Esteve Fernandez

1.8.2 (2014-01-07)
------------------

1.8.0 (2013-07-11)
------------------
* check for CATKIN_ENABLE_TESTING
* update email in package.xml

1.7.15 (2013-03-12)
-------------------

1.7.14 (2013-01-13)
-------------------

1.7.13 (2012-12-27)
-------------------
* move nodelet_topic_tools to separate package, fix unit tests

1.7.12 (2012-12-19 01:34)
-------------------------

1.7.11 (2012-12-19 00:58)
-------------------------

1.7.10 (2012-12-14)
-------------------
* add missing dep to catkin

1.7.9 (2012-12-13)
------------------

1.7.8 (2012-12-06)
------------------
* fix test registration

1.7.7 (2012-11-01)
------------------

1.7.6 (2012-10-30)
------------------
* clean up package.xml files

1.7.5 (2012-10-23)
------------------
* fixed compiling tests

1.7.4 (2012-10-08)
------------------
* fixed cmake to find dependencies correctly

1.7.3 (2012-10-04)
------------------

1.7.2 (2012-10-03)
------------------

1.7.1 (2012-10-02)
------------------
* adding nodelet_core metapackage and reving to 1.7.1

1.7.0 (2012-10-01)
------------------
* make it compile locally
* first pass at catkinizing the stack
* add explicit boost link
* Added benchmark for CallbackQueueManager performance.
* adding support for once, throttle, and filter features.  With unit tests for all but the filters `#4681 <https://github.com/ros/nodelet_core/issues/4681>`_
* fix test on machines with only 1 core (`#4082 <https://github.com/ros/nodelet_core/issues/4082>`_)
* fix hanging tests and a hang on nodelet CallbackQueueManager destruction (`#4082 <https://github.com/ros/nodelet_core/issues/4082>`_)
* backing out my testing changes accidentally committed
* reving for release 1.1.6
* Added Ubuntu platform tags to manifest
* Hopefully fix what is probably a load-related test problem (http://build.willowgarage.com/job/ros-latest-all-test-ubuntu-jaunty-x86_64/36/testReport/test_nodelet.test_callback_queue_manager/CallbackQueueManager/test_nodelet_test_callback_queue_manager_multipleSingleThreaded/)
* Fix && that should have been || (`#4048 <https://github.com/ros/nodelet_core/issues/4048>`_)
* add wait_for_service calls
* merging josh's branch from ticket `#3875 <https://github.com/ros/nodelet_core/issues/3875>`_
* moving topic tools out of nodelet proper, removing rospy and message_filters dependencies from nodelet
* review status
* removing extra launch files and adding test to CMake
* coverage on nodelet commands in tests, coverage of parameter and remapping passing
* two unit tests passing
* first unittest working
* creating test package for nodelet
