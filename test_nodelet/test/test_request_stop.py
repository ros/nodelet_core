#!/usr/bin/env python

# Copyright (c) 2023, Open Source Robotics Foundation
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import rosnode
import rospy
import rostest
import sys
from time import sleep
import unittest

from bond.msg import Status
from nodelet.srv import NodeletUnload, NodeletUnloadRequest
from std_msgs.msg import String


class TestRequestStop(unittest.TestCase):
    def __init__(self, methodName='runTest'):
        super(TestRequestStop, self).__init__(methodName)
        self._last_ready_msg = None
        self._num_received = 0
        self._stop_method = rospy.get_param("~stop_method", "unload")
        self._manager_name = rospy.get_param("~manager_name", "")
        self._wait_for_bond = rospy.get_param("~wait_for_bond", False)
        if self._wait_for_bond:
            self._num_bond_msgs = 0
            self._bond_sub = rospy.Subscriber(
                rospy.names.ns_join(self._manager_name, "bond"), Status, self.bond_cb, queue_size=4)

    def bond_cb(self, msg):
        self._num_bond_msgs += 1

    def ready_cb(self, msg):
        self._last_ready_msg = msg
        self._num_received += 1

    def test_unload(self):
        # The nodelet runs an infinite loop that can only be broken by calling `requestStop()` on the nodelet.
        # Test this by unloading the nodelet, which should automatically call `requestStop()`.
        # The nodelet publishes one message on /ready topic before entering the loop and one message after it is broken.
        # However, if the nodelet is killed instead of unloaded, the `ros::shutdown()` call that happens during the kill
        # will make it impossible to send the second message. So the kill tests just tests whether the node has stopped
        # responding.

        if self._stop_method == "unload":
            unload_srv = rospy.names.ns_join(self._manager_name, "unload_nodelet")
            unload = rospy.ServiceProxy(unload_srv, NodeletUnload)
            unload.wait_for_service()

        self._last_ready_msg = None
        self._num_received = 0

        sub = rospy.Subscriber("ready", String, self.ready_cb, queue_size=2)

        while not rospy.is_shutdown() and self._num_received < 1:
            rospy.logdebug("Waiting for ready message")
            sleep(0.1)

        self.assertGreaterEqual(self._num_received, 1)
        node_name = self._last_ready_msg.data

        # Give the bonds time to start. If we let one bond end to send exactly one heartbeat,
        # we run into https://github.com/ros/bond_core/pull/93 which puts the other end into
        # infinite wait. We want to make sure at least two heartbeats are sent. Each end sends
        # one message per second to the bond topic, so we expect to see at least 4 messages.
        while self._wait_for_bond and not rospy.is_shutdown() and self._num_bond_msgs < 4:
            rospy.loginfo("Waiting for bonds. Have %i messages so far." % (self._num_bond_msgs,))
            sleep(0.1)

        if self._stop_method == "unload":
            req = NodeletUnloadRequest()
            req.name = node_name
            res = unload.call(req)
            self.assertTrue(res.success)

            while not rospy.is_shutdown() and self._num_received < 2:
                rospy.loginfo("Waiting for interrupted message")
                sleep(0.1)

            self.assertEqual(self._num_received, 2)
            self.assertEqual(self._last_ready_msg.data, "interrupted")

        elif self._stop_method.startswith("kill"):
            name_to_kill = node_name if self._stop_method == "kill" else self._manager_name
            success, fail = rosnode.kill_nodes((name_to_kill,))
            self.assertEqual(success, [name_to_kill])
            self.assertEqual(fail, [])

            while rosnode.rosnode_ping(node_name, max_count=1):
                sleep(0.1)
            self.assertFalse(rosnode.rosnode_ping(node_name, max_count=1))


if __name__ == '__main__':
    rospy.init_node('test_request_stop')
    rostest.rosrun('test_request_stop', 'test_request_stop', TestRequestStop)
