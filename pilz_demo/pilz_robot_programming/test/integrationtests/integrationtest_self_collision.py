#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (c) 2018 Pilz GmbH & Co. KG
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
from geometry_msgs.msg import Point
from pilz_robot_programming import *
from pilz_industrial_motion_testutils.integration_test_utils import *

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

API_VERSION = "1"
COLLISION_TOPIC = "/move_group/display_contacts"


class TestSelfCollision(unittest.TestCase):
    """
    Test if Execution is prohibited if during the execution a collision would occur.
    """

    _SLEEP_TIME_S = 0.01
    _TOLERANCE_FOR_MOTION_DETECTION_RAD = 0.01

    def setUp(self):
        rospy.loginfo("SetUp called...")
        self.robot = Robot(API_VERSION)
        self.message_received = False
        self.sub_ = rospy.Subscriber(COLLISION_TOPIC, MarkerArray, self.collisionCallback)
        self.condition = threading.Condition()

    def tearDown(self):
        rospy.loginfo("TearDown called...")
        self.robot._release()
        self.robot = None

    def collisionCallback(self, data):
        self.message_received = True
        with self.condition:
            self.condition.notifyAll()

    def test_self_collision(self):
        """ Tests that self collision is detected.
            Test sequence:
                1. Move out of zero position to a position where the gripper brackets almost touch the robot.
                2. Make a lin movement that would scratch the robot.

            Test Results:
                1. -
                2. No movement is executed. Message is send on COLLISION_TOPIC.
        """

        move_thread = MoveThread(self.robot, Ptp(goal=[0, 0.185, 2.27, 0, 1.25, 0]), RobotMoveFailed)
        move_thread.start()
        move_thread.join()
        rospy.loginfo("Ptp Movement done.")

        pos_before_lin = self.robot.get_current_joint_states()
        rospy.loginfo(pos_before_lin)

        move_thread_lin = MoveThread(self.robot,
                                     Lin(goal=Pose(position=Point(0.15, 0, 0.12)), relative=True, vel_scale=0.5,
                                         acc_scale=0.1),
                                     RobotMoveFailed)
        move_thread_lin.start()

        rospy.loginfo("Waiting for notification...")
        with self.condition:
            self.condition.wait(timeout=5.0)

        move_thread_lin.join()
        pos_after_lin = self.robot.get_current_joint_states()

        self.assertEqual(pos_before_lin, pos_after_lin)
        self.assertTrue(move_thread_lin.exception_thrown)
        self.assertTrue(self.message_received)


if __name__ == '__main__':
    import rostest

    rospy.init_node('test_self_collision')
    rostest.rosrun('pilz_industrial_motion_planner', 'test_self_collision', TestSelfCollision)
