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
from math import radians

from geometry_msgs.msg import Point

from pilz_robot_programming import *

from pilz_industrial_motion_testutils.integration_test_utils import *

API_VERSION = "1"

_JOINT_LIMITS_DEGREE = {
    'prbt_joint_1': 170,
    'prbt_joint_2': 145,
    'prbt_joint_3': 135,
    'prbt_joint_4': 170,
    'prbt_joint_5': 170,
    'prbt_joint_6': 179
}
_JOINT_POSITIONS_TOLERANCE = 0.001
_JOINT_LIMIT_OVERSTEP = 0.1


class TestJointPositionLimits(unittest.TestCase):

    def setUp(self):
        self.robot = Robot(API_VERSION)
        self.joint_names = _JOINT_LIMITS_DEGREE.keys()
        self.joint_names = sorted(self.joint_names)

    def tearDown(self):
        if hasattr(self, 'robot'):
            self.robot._release()
            self.robot = None

    def _check_joint_limits(self):
        """ Check if current joint positions are within the limits.
        """
        positions = self.robot.get_current_joint_states()

        for i in range(len(self.joint_names)):
            name = self.joint_names[i]
            position = positions[i]
            limit = radians(_JOINT_LIMITS_DEGREE[name])+_JOINT_POSITIONS_TOLERANCE

            self.assertGreater(position, -limit, 'Joint ' + name + ' violates lower limit. Position: ' + str(position))
            self.assertLess(position, limit, 'Joint ' + name + ' violates upper limit. Position: ' + str(position))

    def _joint_limit_reaching_test(self, joint_name):
        """ Test if the robot can be commanded to move exactly to the limits

            Test Sequence:
              1. Command a movement to the lower limit.
              2. Command a movement to the upper limit.

            Expected Results:
              1. Trajectory is executed successfully.
              2. Trajectory is executed successfully.
        """
        index = self.joint_names.index(joint_name)
        limit = _JOINT_LIMITS_DEGREE[joint_name]

        lower_positions = [0] * len(self.joint_names)
        lower_positions[index] = -radians(limit)

        try:
            self.robot.move(Ptp(goal=lower_positions))
        except RobotMoveFailed:
            self.fail('Failed moving exactly to lower limit')

        upper_positions = [0] * len(self.joint_names)
        upper_positions[index] = radians(limit)

        try:
            self.robot.move(Ptp(goal=upper_positions))
        except RobotMoveFailed:
            self.fail('Failed moving exactly to upper limit')

    def _joint_limit_overstepping_test(self, joint_name):
        """ Test if the robot does not overstep the limits

            Test Sequence:
              1. Command a movement to the home position.
              2. Command a movement overstepping the lower limit.
              3. Command a movement overstepping the upper limit.

            Expected Results:
              1. Trajectory is executed successfully.
              2. Trajectory execution is aborted and the robot does not overstep the limits.
              3. Trajectory execution is aborted and the robot does not overstep the limits.
        """
        index = self.joint_names.index(joint_name)
        limit = _JOINT_LIMITS_DEGREE[joint_name]

        lower_positions = [0] * len(self.joint_names)
        lower_positions[index] = -(radians(limit) + _JOINT_LIMIT_OVERSTEP)

        self.assertRaises(RobotMoveFailed, self.robot.move, Ptp(goal=lower_positions))
        self._check_joint_limits()

        upper_positions = [0] * len(self.joint_names)
        upper_positions[index] = radians(limit) + _JOINT_LIMIT_OVERSTEP

        self.assertRaises(RobotMoveFailed, self.robot.move, Ptp(goal=upper_positions))
        self._check_joint_limits()

    def test_joint_joint_limits_reaching(self):
        """ Perform all reaching tests.
        """
        for name in self.joint_names:
            self._joint_limit_reaching_test(name)

    def test_joint_joint_limits_overstepping(self):
        """ Perform all overstepping tests.
        """
        for name in self.joint_names:
            self._joint_limit_overstepping_test(name)


if __name__ == '__main__':
    import rostest
    rospy.init_node('test_joint_position_limits')
    rostest.rosrun('pilz_robot_programming', 'test_joint_position_limits', TestJointPositionLimits)
