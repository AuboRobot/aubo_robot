#!/usr/bin/env python
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
from pilz_robot_programming import *
from pilz_industrial_motion_testutils.xml_testdata_loader import *
from pilz_industrial_motion_testutils.integration_test_utils import *
from pilz_industrial_motion_testutils.robot_motion_observer import RobotMotionObserver
from tst_api_utils import setOverrideParam
from pathlib import Path

_TEST_DATA_FILE_NAME = Path(__file__).parent.parent.absolute() / Path("test_data/test_data.xml")
API_VERSION = "1"


class TestAPIGripper(unittest.TestCase):
    """ Test the gripper command"""

    _SLEEP_TIME_S = 0.01
    _TOLERANCE_FOR_MOTION_DETECTION_RAD = 0.01

    def setUp(self):
        self.robot = Robot(API_VERSION)
        self.robot_motion_observer = RobotMotionObserver(group_name="gripper")
        self.test_data = XmlTestdataLoader(_TEST_DATA_FILE_NAME)

    def tearDown(self):
        if hasattr(self, 'robot'):
            self.robot._release()
            self.robot = None
        if hasattr(self, 'test_data'):
            del self.test_data

    def test_gripper_cmd_convert_invalid_goal(self):
        """ Test the gripper convert function with invalid goal.

            Test sequence:
                1. Call gripper convert function with a list.

            Test Results:
                1. raises Exception.
        """

        gripper_cmd = Gripper(goal=[1, 2])
        self.assertRaises(RuntimeError, gripper_cmd._cmd_to_request, self.robot)

    def test_gripper_cmd_convert(self):
        """ Test the gripper convert function works correctly.

            Test sequence:
                1. Call gripper convert function with a valid floating point goal.

            Test Results:
                1. Correct motion plan request is returned.
        """

        gripper_cmd = Gripper(goal=0.02, vel_scale=0.1)
        req = gripper_cmd._cmd_to_request(self.robot)
        self.assertIsNotNone(req)
        self.assertEqual("gripper", req.group_name)
        # gripper command is internal a ptp command
        self.assertEqual("PTP", req.planner_id)
        self.assertEqual(0.02, req.goal_constraints[0].joint_constraints[0].position)
        self.assertEqual(0.1, req.max_velocity_scaling_factor)

    def test_gripper_cmd_convert_int(self):
        """ Test the gripper convert function works correctly.

            Test sequence:
                1. Call gripper convert function with a valid integer goal.

            Test Results:
                1. Correct motion plan request is returned.
        """

        gripper_cmd = Gripper(goal=0, vel_scale=0.1)
        req = gripper_cmd._cmd_to_request(self.robot)
        self.assertIsNotNone(req)
        self.assertEqual("gripper", req.group_name)
        # gripper command is internal a ptp command
        self.assertEqual("PTP", req.planner_id)
        self.assertEqual(0.0, req.goal_constraints[0].joint_constraints[0].position)
        self.assertEqual(0.1, req.max_velocity_scaling_factor)

    def test_gripper_cmd_convert_ignore_speed_override(self):
        """ Test the gripper convert function ignores the speed override.

            Test sequence:
                1. Call gripper convert function with a valid floating point goal and set a SpeedOverride

            Test Results:
                1. The max_velocity_scaling_factor and max_acceleration_scaling_factor is not changed.
        """

        gripper_cmd = Gripper(goal=0.02, vel_scale=0.1, acc_scale=0.2)
        req = gripper_cmd._cmd_to_request(self.robot)
        setOverrideParam(0.1)

        # gripper command is internal a ptp command
        self.assertEqual(0.1, req.max_velocity_scaling_factor)
        self.assertEqual(0.2, req.max_acceleration_scaling_factor)

    def test_gripper_execution(self):
        """ Test execution of valid gripper command works successfully.

            Test sequence:
                1. Execute a valid gripper command

            Test results:
                1. Move function returns without throwing an exception.
        """
        self.robot.move(Gripper(goal=0.01))

    def test_invalid_gripper_execution(self):
        """ Test execution of invalid gripper command works successfully.

            Test sequence:
                1. Execute an invalid gripper command with out of bounds joints

            Test results:
                1. Move function throws exception.
        """
        self.assertRaises(RobotMoveFailed, self.robot.move, Gripper(goal=1))

    def test_gripper_stop_during_command(self):
        """Test stop during gripper command execution.

            Test sequence:
                1. Move the gripper to zero position
                2. Start given command in separate thread.
                3. Wait till robot is moving then execute stop function in main test thread.

            Test Results:
                1. Robot's gripper closes.
                2. Robot's gripper starts moving.
                3. Execute methods ends and throws correct exception.
        """
        # 1
        self.robot.move(Gripper(goal=0.00))

        # 2
        move_thread = MoveThread(self.robot, Gripper(goal=0.02), RobotMoveFailed)
        move_thread.start()

        # 3
        # Wait till movement started
        self.assertTrue(self.robot_motion_observer.wait_motion_start(
            move_tolerance=self._TOLERANCE_FOR_MOTION_DETECTION_RAD, sleep_interval=self._SLEEP_TIME_S))

        rospy.loginfo("Call stop function.")
        self.robot.stop()

        rospy.loginfo("Wait till thread/command finished to evaluate result...")
        move_thread.join()

        # Check return value of execute method
        self.assertTrue(move_thread.exception_thrown)

    def test_get_current_gripper_joint(self):
        """ Check if the current joints can be retrieved correctly

            Test sequence:
                1. Move robot to a goal position via ptp.
                2. Check the current pose by robot.get_current_joints().

            Test Results:
                1. Move function returns without throwing an exception.
                2. The current joints are same as the goal joints.
        """
        # 1
        goal_joint = 0.03
        self.robot.move(Gripper(goal=goal_joint))

        # 2
        current_joints = self.robot.get_current_joint_states("gripper")
        self.assertEqual(1, len(current_joints))
        self.assertAlmostEqual(goal_joint, current_joints[0])


if __name__ == '__main__':
    import rostest
    rospy.init_node('tst_api_gripper')
    rostest.rosrun('pilz_robot_programming', 'tst_api_gripper', TestAPIGripper)
