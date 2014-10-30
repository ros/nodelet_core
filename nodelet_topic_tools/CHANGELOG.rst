^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package nodelet_topic_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* fix missing boost dependency
* Contributors: Dirk Thomas, Esteve Fernandez

* fix missing boost dependency

1.8.2 (2014-01-07)
------------------

1.8.0 (2013-07-11)
------------------
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
* remove obsolete imports

1.7.11 (2012-12-19 00:58)
-------------------------
* reordering dynamic_reconfigure in CMakeLists.txt

1.7.10 (2012-12-14)
-------------------
* add missing dep to catkin

1.7.9 (2012-12-13)
------------------

1.7.8 (2012-12-06)
------------------
* updated catkin_package(DEPENDS)

1.7.7 (2012-11-01)
------------------
* no need to export the plugin as it is for testing only

1.7.6 (2012-10-30)
------------------
* clean up package.xml files

1.7.5 (2012-10-23)
------------------
* comply to the new dynamic_reconfigure API

1.7.4 (2012-10-08)
------------------
* fixed cmake to find dependencies correctly

1.7.3 (2012-10-04)
------------------
* fix typo

1.7.2 (2012-10-03)
------------------
* add rostest as a dependency

1.7.1 (2012-10-02)
------------------
* adding nodelet_core metapackage and reving to 1.7.1

1.7.0 (2012-10-01)
------------------
* make it compile locally
* first pass at catkinizing the stack
* Adding nodelet throttle, `#5295 <https://github.com/ros/nodelet_core/issues/5295>`_
* fixed a grave bug where nullfilters were not working correctly
* added missing dependency
* MUX simplified by using a 8-connected null filters
  DeMUX has a specialization for message type (uses ros::Subscriber internally by default)
  Added rosdep for nodelet (uuid)
* Added Ubuntu platform tags to manifest
* fixed the tools (broken, did not compile)
* removed the transport for now
* moving topic tools out of nodelet proper, removing rospy and message_filters dependencies from nodelet
