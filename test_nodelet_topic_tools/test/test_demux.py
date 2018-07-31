#!/usr/bin/env python

import rospy
import threading
import unittest

from std_msgs.msg import String


class TestNodeletDemux(unittest.TestCase):
    def __init__(self, *args):
        super(TestNodeletDemux, self).__init__(*args)

        self._lock = threading.RLock()

        self._msgs_rec = {}

        self._pub = rospy.Publisher('input', String, queue_size=4)
        self.output_topics = rospy.get_param('~output_topics', [])
        rospy.loginfo(self.output_topics)
        self._sub = {}
        for key in self.output_topics:
            self._sub[key] = rospy.Subscriber(key, String, self._cb, (key))

    def _cb(self, msg, key):
        with self._lock:
            if key not in self._msgs_rec.keys():
                self._msgs_rec[key] = 0
            self._msgs_rec[key] += 1

    def test_nodelet_demux(self):
        self._msgs_rec = {}
        num_each_to_receive = 4
        num_to_send = len(self.output_topics) * num_each_to_receive
        self.assert_(num_to_send > 0, "No output topics to test with")

        # TODO(lucasw) is this a really brittle test, maybe a few messags will be missed
        # and the assert below should be within 10-20% of expected?
        rospy.sleep(1.0)
        for i in range(num_to_send):
            self._pub.publish(String('hello, world'))
            rospy.sleep(0.2)
        rospy.sleep(1.0)

        rospy.loginfo(self._msgs_rec)
        for key in self.output_topics:
            self.assert_(key in self._msgs_rec.keys(), "No messages received on " +
                         key + "\" " + str(num_each_to_receive))
            if key in self._msgs_rec.keys():
                self.assert_(self._msgs_rec[key] == num_each_to_receive,
                             "Wrong messages received from nodelet demux on topic \"" +
                             key + "\" " + str(self._msgs_rec[key]) + " != " + str(num_each_to_receive))

if __name__ == '__main__':
    rospy.init_node('test_nodelet_demux')

    import rostest
    rostest.rosrun('test_nodelet_topic_tools', 'test_nodelet_demux', TestNodeletDemux)
