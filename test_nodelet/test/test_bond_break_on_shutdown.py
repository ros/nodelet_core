#!/usr/bin/env python

import roslib; roslib.load_manifest('test_nodelet')
import rospy
import unittest
import rostest
import signal
import subprocess
import time

from nodelet.srv import *

class TestBondBreakOnShutdown(unittest.TestCase):
    def test_bond_break_on_shutdown(self):
        '''
        Test that the bond is broken cleanly when closing a nodelet loader (#50).
        This relies on a debug message printed by the bondcpp package in case
        of error.
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

        # load a nodelet
        proc_nodelet = subprocess.Popen(["rosrun", "nodelet", "nodelet", "load", "test_nodelet/Plus", "nodelet_manager", "__name:=test"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=-1
        )

        # wait for it to be ready
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

        rospy.loginfo(n_out)
        rospy.loginfo(m_out)

        # check that nodelet unloaded and there was no error with bond breaking
        self.assertIn('Unloading nodelet /test from manager nodelet_manager', n_out)
        self.assertNotIn('Bond failed to break on destruction', m_out)
        self.assertNotIn('Bond failed to break on destruction', n_out)

if __name__ == '__main__':
    rospy.init_node('test_bond_break_on_shutdown')
    rostest.unitrun('test_bond_break_on_shutdown', 'test_bond_break_on_shutdown', TestBondBreakOnShutdown)

