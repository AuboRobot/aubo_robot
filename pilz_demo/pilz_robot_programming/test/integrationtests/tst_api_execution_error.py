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
from pilz_robot_programming.robot import *
from pilz_industrial_motion_testutils.xml_testdata_loader import *
from pilz_robot_programming.commands import *
from pathlib import Path

import robot_commander_mock

_TEST_DATA_FILE_NAME = Path(__file__).parent.parent.absolute() / Path("test_data/test_data.xml")
PLANNING_GROUP_NAME = "manipulator"
API_VERSION = "1"

MOVEIT_MOCK_STATUS_SERVICE_NAME = "moveit_mock_status"


class TestAPIExecutionError(unittest.TestCase):
    """Tests the behavior of the Robot.move(...) function for the different commands in case an error occurs
    during command execution."""

    def setUp(self):
        # Wait till MoveItMock is up and running (signaled via ROS-Service).
        rospy.wait_for_service(MOVEIT_MOCK_STATUS_SERVICE_NAME)

        self.test_data = XmlTestdataLoader(_TEST_DATA_FILE_NAME)

        self.robot = Robot(API_VERSION)
        # Inject RobotCommanderMock into test object: Robot.
        self.robot._robot_commander = robot_commander_mock.RobotCommanderMock()

    def tearDown(self):
        if hasattr(self, 'robot'):
            self.robot._release()
            self.robot = None
        if hasattr(self, 'test_data'):
            del self.test_data

    def test_gripper_error_during_command(self):
        """Test what happens if error occurs during command execution.

            Test sequence:
                1. Trigger execution of command + Cause executed command to fail (done via MoveitMock).

            Test Results:
                1. Move function throws exception.
        """
        cmd_for_testing = Gripper(goal=0.01)
        self.assertRaises(RobotMoveFailed, self.robot.move, cmd_for_testing)

    def test_ptp_error_during_command(self):
        """Test what happens if error occurs during command execution.

            Test sequence:
                1. Trigger execution of command + Cause executed command to fail (done via MoveitMock).

            Test Results:
                1. Move function throws exception.
        """
        cmd_for_testing = Ptp(goal=self.test_data.get_joints("PTPJointValid", PLANNING_GROUP_NAME))
        self.assertRaises(RobotMoveFailed, self.robot.move, cmd_for_testing)

    def test_lin_error_during_command(self):
        """Test what happens if error occurs during command execution.

            Test sequence:
                1. Trigger execution of command + Cause executed command to fail (done via MoveitMock).

            Test Results:
                1. Move function finishes with status: FAILURE.
        """
        cmd_for_testing = Lin(goal=self.test_data.get_pose("CartValid", PLANNING_GROUP_NAME), vel_scale=0.1)
        self.assertRaises(RobotMoveFailed, self.robot.move, cmd_for_testing)

    def test_circ_stop_during_command(self):
        """Test what happens if error occurs during command execution.

            Test sequence:
                1. Trigger execution of command + Cause executed command to fail (done via MoveitMock).

            Test Results:
                1. Move function throws exception.
        """
        cmd_for_testing = Circ(goal=self.test_data.get_pose("CIRCGoalPose", PLANNING_GROUP_NAME),
                               center=self.test_data.get_pose("CIRCCenterPose", PLANNING_GROUP_NAME).position,
                               vel_scale=0.1, acc_scale=0.1)
        self.assertRaises(RobotMoveFailed, self.robot.move, cmd_for_testing)

    def test_blend_stop_during_command(self):
        """Test what happens if error occurs during command execution.

            Test sequence:
                1. Trigger execution of command + Cause executed command to fail (done via MoveitMock).

            Test Results:
                1. Move function throws exception.
        """
        cmd_for_testing = Sequence()
        cmd_for_testing.append(Lin(goal=self.test_data.get_pose("Blend_1_Mid", PLANNING_GROUP_NAME), vel_scale=0.1,
                               acc_scale=0.1), blend_radius=0.12)
        cmd_for_testing.append(Lin(goal=self.test_data.get_pose("Blend_1_End", PLANNING_GROUP_NAME), vel_scale=0.1,
                               acc_scale=0.1), blend_radius=0.1)
        cmd_for_testing.append(Lin(goal=self.test_data.get_pose("Blend_1_Start", PLANNING_GROUP_NAME), vel_scale=0.04,
                               acc_scale=0.1), blend_radius=0)
        self.assertRaises(RobotMoveFailed, self.robot.move, cmd_for_testing)

    def test_gripper_cmd_incorrect_num_joints(self):
        """Test what happens if RobotCommander returns incorrect number of joints for gripper planning-group.

            Test sequence:
                1. Trigger execution of gripper command.

            Test Results:
                1. Move function throws exception.
        """
        cmd_for_testing = Gripper(goal=0.01)
        # Cause RobotCommander-Mock to return incorrect number of joints by setting incorrect planning-group name.
        cmd_for_testing._planning_group = "manipulator"

        self.assertRaises(IndexError, cmd_for_testing._cmd_to_request, self.robot)


if __name__ == '__main__':
    import rostest
    rospy.init_node('test_api_execution_error')
    rostest.rosrun('pilz_robot_programming', 'test_api_execution_error', TestAPIExecutionError)
