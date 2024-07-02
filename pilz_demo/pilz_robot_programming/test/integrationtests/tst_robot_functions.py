#!/usr/bin/env python
# Copyright (c) 2019 Pilz GmbH & Co. KG
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import unittest
import rospy
import sys
from pilz_robot_programming.robot import *
from pilz_robot_programming.commands import Ptp
from tst_api_utils import setOverrideParam
import roslib

API_VERSION = "1"
PKG = 'pilz_robot_programming'
roslib.load_manifest(PKG)  # This line is not needed with Catkin.


class TestSpeedOverride(unittest.TestCase):
    """ Checks the basic behaviour of the speed override.
        More in-depth checks are done withing the tst_api_cmd_conversion.py
    """
    def setUp(self):
        rospy.loginfo("SetUp called...")
        self.robot = Robot(API_VERSION)

    def tearDown(self):
        if hasattr(self, 'robot'):
            self.robot._release()
            self.robot = None

    def testSettingOverride(self):
        setOverrideParam(0.3)
        self.assertEqual(self.robot._speed_override, 0.3)

    def testInvariantMoveOnCommand(self):
        """ Test that the command itself remains untouched to the speed override"""
        setOverrideParam(0.3)
        ptp = Ptp(goal=[0, 0.5, 0.5, 0, 0, 0], vel_scale=0.3, acc_scale=0.4)
        self.robot.move(ptp)
        self.assertEqual(ptp._vel_scale, 0.3)
        self.assertEqual(ptp._acc_scale, 0.4)


if __name__ == '__main__':
    import rostest
    rospy.init_node('tst_robot_functions')
    rostest.rosrun('pilz_robot_programming',
                   'tst_robot_functions', TestSpeedOverride)
