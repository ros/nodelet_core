#!/usr/bin/env python
###############################################################################
# Software License Agreement (BSD License)
#
#  Copyright (c) 2016, JSK Lab, University of Tokyo.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/o2r other materials provided
#     with the distribution.
#   * Neither the name of the JSK Lab nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
###############################################################################

__author__ = 'Kentaro Wada <www.kentaro.wada@gmail.com>'

import os
import sys

import unittest

import rosgraph
import rospy
import rosmsg
import roslib


PKG = 'test_nodelet_topic_tools'
NAME = 'test_lazy'


class TestConnection(unittest.TestCase):

    def __init__(self, *args):
        super(TestConnection, self).__init__(*args)
        rospy.init_node(NAME)
        self.master = rosgraph.Master(NAME)

    def test_no_subscribers(self):
        check_connected_topics = rospy.get_param('~check_connected_topics')
        # Check assumed topics are not there
        _, subscriptions, _ = self.master.getSystemState()
        for check_topic in check_connected_topics:
            for topic, sub_node in subscriptions:
                if topic == rospy.get_namespace() + check_topic:
                    raise ValueError('Found topic: {}'.format(check_topic))

    def test_subscriber_appears(self):
        topic_type = rospy.get_param('~input_topic_type')
        check_connected_topics = rospy.get_param('~check_connected_topics')
        wait_time = rospy.get_param('~wait_for_connection', 0)
        msg_class = roslib.message.get_message_class(topic_type)
        # Subscribe topic and bond connection
        sub = rospy.Subscriber('~input', msg_class,
                               self._cb_test_subscriber_appears)
        print('Waiting for connection for {} sec.'.format(wait_time))
        rospy.sleep(wait_time)
        # Check assumed topics are there
        _, subscriptions, _ = self.master.getSystemState()
        for check_topic in check_connected_topics:
            for topic, sub_node in subscriptions:
                if topic == rospy.get_namespace() + check_topic:
                    break
            else:
                raise ValueError('Not found topic: {}'.format(check_topic))

    def _cb_test_subscriber_appears(self, msg):
        pass


if __name__ == "__main__":
    import rostest
    rostest.rosrun(PKG, NAME, TestConnection)
