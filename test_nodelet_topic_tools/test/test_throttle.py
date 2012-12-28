#!/usr/bin/env python

PKG = 'nodelet_topic_tools'
import roslib; roslib.load_manifest(PKG)

import unittest
import threading

import rospy

from std_msgs.msg import String

class TestNodeletThrottle(unittest.TestCase):
    def __init__(self, *args):
        super(TestNodeletThrottle, self).__init__(*args)

        self._lock = threading.RLock()

        self._msgs_rec = 0

        self._pub = rospy.Publisher('string_in', String)
        self._sub = rospy.Subscriber('string_out', String, self._cb)

    def _cb(self, msg):
        with self._lock:
            self._msgs_rec += 1

    def test_nodelet_throttle(self):
        for i in range(0,10):
            self._pub.publish(String('hello, world'))
            rospy.sleep(1.0)        

        self.assert_(self._msgs_rec > 0, "No messages received from nodelet throttle on topic \"string_out\"")

if __name__ == '__main__':
    rospy.init_node('test_nodelet_throttle')

    import rostest
    rostest.rosrun(PKG, 'test_nodelet_throttle', TestNodeletThrottle)

