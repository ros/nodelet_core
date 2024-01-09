^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package test_nodelet_topic_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.11.0 (2024-01-09)
-------------------
* Update package maintainers (`#114 <https://github.com/ros/nodelet_core/issues/114>`_)
* Contributors: Geoffrey Biggs

1.10.2 (2021-10-07)
-------------------

1.10.1 (2021-01-25)
-------------------

1.10.0 (2020-03-09)
-------------------
* Update maintainer to Michael Carroll (`#104 <https://github.com/ros/nodelet_core/issues/104>`_)
* Fix nodelet tests for focal (`#103 <https://github.com/ros/nodelet_core/issues/103>`_)
* Bump CMake version to avoid CMP0048 warning (`#102 <https://github.com/ros/nodelet_core/issues/102>`_)
* Temporarily disable tests to appease noetic CI (`#97 <https://github.com/ros/nodelet_core/issues/97>`_)
* Contributors: Michael Carroll, Shane Loretz

1.9.16 (2018-04-27)
-------------------

1.9.15 (2018-03-16)
-------------------
* use new pluginlib headers (`#73 <https://github.com/ros/nodelet_core/issues/73>`_)
* Contributors: Mikael Arguedas

1.9.14 (2017-11-15)
-------------------
* update to use non deprecated pluginlib macro (`#69 <https://github.com/ros/nodelet_core/issues/69>`_)
* Contributors: Mikael Arguedas

1.9.13 (2017-10-27)
-------------------

1.9.12 (2017-08-04)
-------------------

1.9.11 (2017-07-27)
-------------------
* remove trailing whitespaces (`#62 <https://github.com/ros/nodelet_core/issues/62>`_)
* switch to package format 2 (`#63 <https://github.com/ros/nodelet_core/issues/63>`_)
* Test laziness after disconnection (`#60 <https://github.com/ros/nodelet_core/issues/60>`_)
* Contributors: Kentaro Wada, Mikael Arguedas

1.9.10 (2017-03-27)
-------------------

1.9.9 (2017-02-17)
------------------

1.9.8 (2016-11-15)
------------------

1.9.7 (2016-10-24)
------------------

1.9.6 (2016-09-20)
------------------
* Add NodeletLazy abstract class for lazy transport (`#45 <https://github.com/ros/nodelet_core/issues/45>`_)
  * Add NodeletLazy abstract class for lazy transport
  * Add test for NodeletLazy with checking its lazy-ness
  * Fix ROS logging format supporting `ros/ros_comm#875 <https://github.com/ros/ros_comm/issues/875>`_
  * Fix ever_subscribed\_ flag setting location
  * Clearfy the comment describing advertise method
  * Parameterize the duration to warn the no connection
  User can disable this feature by setting -1 to the param.
* Contributors: Kentaro Wada

1.9.5 (2016-06-22)
------------------
* fix non-isolated build
* Contributors: Dirk Thomas

1.9.4 (2016-03-15)
------------------
* update maintainer
* Contributors: Mikael Arguedas

1.9.3 (2015-08-05)
------------------

1.9.2 (2014-10-30)
------------------

1.9.1 (2014-10-29)
------------------

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
* use EXPORTED_TARGETS variable instead of explicit target names
* update email in package.xml

1.7.15 (2013-03-12)
-------------------

1.7.14 (2013-01-13)
-------------------
* remove duplicate dep

1.7.13 (2012-12-27)
-------------------
* move nodelet_topic_tools to separate package, fix unit tests

1.7.12 (2012-12-19 01:34)
-------------------------

1.7.11 (2012-12-19 00:58)
-------------------------

1.7.10 (2012-12-14)
-------------------

1.7.9 (2012-12-13)
------------------

1.7.8 (2012-12-06)
------------------

1.7.7 (2012-11-01)
------------------

1.7.6 (2012-10-30)
------------------

1.7.5 (2012-10-23)
------------------

1.7.4 (2012-10-08)
------------------

1.7.3 (2012-10-04)
------------------

1.7.2 (2012-10-03)
------------------

1.7.1 (2012-10-02)
------------------

1.7.0 (2012-10-01)
------------------
