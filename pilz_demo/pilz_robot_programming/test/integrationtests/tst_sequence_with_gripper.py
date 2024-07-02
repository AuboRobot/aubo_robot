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
import numpy

from geometry_msgs.msg import Point

from pilz_robot_programming import *

from pilz_industrial_motion_testutils.integration_test_utils import *

API_VERSION = "1"


class TestSequenceWithGripper(unittest.TestCase):

    _GRIPPER_GROUP_NAME = "gripper"
    _GRIPPER_POSE_CLOSED = 0.001
    _GRIPPER_POSE_OPEN = 0.02

    _TOLERANCE_POSITION_COMPARE = 1e-3

    def setUp(self):
        if 'ros_init_done' not in globals():
            rospy.init_node('tst_sequence_with_gripper')
            global ros_init_done

        self.robot = Robot(API_VERSION)
        self.robot.move(Gripper(goal=self._GRIPPER_POSE_CLOSED))

    def tearDown(self):
        if hasattr(self, 'robot'):
            self.robot._release()
            self.robot = None

    def test_only_gripper_in_sequence(self):
        """ Tests a sequence with only gripper commands.
        """
        start_joint_values = [0, 0.5, 0.5, 0, 0, 0]
        self.robot.move(Ptp(goal=start_joint_values))

        seq = Sequence()
        seq.append(Gripper(goal=self._GRIPPER_POSE_OPEN), blend_radius=0.1)
        seq.append(Gripper(goal=self._GRIPPER_POSE_CLOSED), blend_radius=0.1)
        seq.append(Gripper(goal=self._GRIPPER_POSE_OPEN))
        self.robot.move(seq)

        # Check that gripper has moved
        current_joints = self.robot.get_current_joint_states(self._GRIPPER_GROUP_NAME)
        self.assertEqual(1, len(current_joints))
        self.assertAlmostEqual(self._GRIPPER_POSE_OPEN, current_joints[0])

        # Check that robot has NOT moved
        self.assertTrue(numpy.allclose(self.robot.get_current_joint_states(),
                                       start_joint_values, atol=self._TOLERANCE_POSITION_COMPARE))

    def test_invalid_gripper_execution(self):
        """ Tests a sequence containing an invalid gripper command.
        """
        start_joint_values = [0, 0.5, 0.5, 0, 0, 0]
        self.robot.move(Ptp(goal=start_joint_values))

        gripper_invalid_pos = 1

        seq = Sequence()
        seq.append(Gripper(goal=self._GRIPPER_POSE_OPEN))

        seq.append(Lin(goal=Pose(position=Point(0.2, 0, 0.7))), blend_radius=0.01)
        seq.append(Lin(goal=Pose(position=Point(0.2, 0.1, 0.7))))

        seq.append(Gripper(goal=self._GRIPPER_POSE_CLOSED))
        seq.append(Gripper(goal=gripper_invalid_pos))

        self.assertRaises(RobotMoveFailed, self.robot.move, seq)

        # Check that robot has NOT moved
        self.assertTrue(numpy.allclose(self.robot.get_current_joint_states(),
                                       start_joint_values, atol=self._TOLERANCE_POSITION_COMPARE))

    def test_mixed_cmds_in_sequence(self):
        """ Tests a sequence which contains several gripper and non-gripper commands.
        Invalid blend radii are set to test if execution of sequence is still possible.
        """
        start_joint_values = [0, 0.5, 0.5, 0, 0, 0]
        self.robot.move(Ptp(goal=start_joint_values))

        gripper_totally_open = 0.029

        seq = Sequence()
        seq.append(Gripper(goal=self._GRIPPER_POSE_OPEN), blend_radius=1.0)

        seq.append(Lin(goal=Pose(position=Point(0.2, 0, 0.7))), blend_radius=0.01)
        expected_pos = Point(0.2, 0.1, 0.7)
        seq.append(Lin(goal=Pose(position=expected_pos)), blend_radius=1.0)

        seq.append(Gripper(goal=self._GRIPPER_POSE_CLOSED), blend_radius=1.0)
        seq.append(Gripper(goal=gripper_totally_open))

        self.robot.move(seq)

        # Check that gripper has opened
        curr_gripper_joints = self.robot.get_current_joint_states(self._GRIPPER_GROUP_NAME)
        self.assertEqual(1, len(curr_gripper_joints))
        self.assertAlmostEqual(gripper_totally_open, curr_gripper_joints[0])

        # Check that robot has moved
        curr_position = self.robot.get_current_pose().position
        self.assertTrue(numpy.allclose(curr_position.x,
                                       expected_pos.x, atol=self._TOLERANCE_POSITION_COMPARE))
        self.assertTrue(numpy.allclose(curr_position.y,
                                       expected_pos.y, atol=self._TOLERANCE_POSITION_COMPARE))
        self.assertTrue(numpy.allclose(curr_position.z,
                                       expected_pos.z, atol=self._TOLERANCE_POSITION_COMPARE))


if __name__ == '__main__':
    import rostest
    if 'ros_init_done' not in globals():
        rospy.init_node('tst_sequence_with_gripper')
        global ros_init_done
    rostest.rosrun('pilz_robot_programming', 'tst_sequence_with_gripper', TestSequenceWithGripper)
