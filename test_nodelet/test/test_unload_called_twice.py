#!/usr/bin/env python

import roslib; roslib.load_manifest('test_nodelet')
import rospy
import unittest
import rostest
import signal
import subprocess
import time

from nodelet.srv import *

class TestUnloadCalledTwice(unittest.TestCase):
    def test_unload_called_twice(self):
        '''
        Test that when a nodelet loader is stopped and requests unloading,
        the unload() call in LoaderROS is not run twice (#50).
        '''

        # start nodelet manager
        proc_manager = subprocess.Popen(["rosrun", "nodelet", "nodelet", "manager", "__name:=nodelet_manager"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=-1
        )

        # wait for nodelet manager to be ready
        try:
            rospy.wait_for_service('/nodelet_manager/load_nodelet', timeout=2)
        except:
            self.fail("Could not determine that nodelet manager has started")

        # load nodelet
        proc_nodelet = subprocess.Popen(["rosrun", "nodelet", "nodelet", "load", "test_nodelet/Plus", "nodelet_manager", "__name:=test"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )

        # wait for nodelet to be ready
        try:
            rospy.wait_for_service('test/get_loggers', timeout=2)
        except:
            self.fail("Could not determine that nodelet has started")
        time.sleep(1)

        # stop the nodelet loader via signal (similar to roslaunch killing it)
        proc_nodelet.send_signal(signal.SIGINT)
        (n_out, n_err) = proc_nodelet.communicate()

        # stop the nodelet manager, too
        proc_manager.send_signal(signal.SIGINT)
        (m_out, m_err) = proc_manager.communicate()

        # check that nodelet unloaded and that LoaderROS::unload() does not
        # complain about nodelet not being found (an indication that it was called
        # again after the nodelet was already unloaded)
        self.assertIn('Unloading nodelet /test from manager nodelet_manager', n_out)
        self.assertNotIn('Failed to find nodelet with name', m_err)

if __name__ == '__main__':
    rospy.init_node('test_unload_called_twice')
    rostest.rosrun('test_unload_called_twice', 'test_unload_called_twice', TestUnloadCalledTwice)
