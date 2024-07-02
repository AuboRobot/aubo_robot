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
from unittest.mock import patch

import rospy

from pilz_robot_programming.exceptions import RobotBrakeTestException
from pilz_robot_programming.robot import Robot
from brake_test_mock import BrakeTestMock
from pilz_msgs.msg import IsBrakeTestRequiredResult


API_VERSION = "1"


class TestAPIBrakeTest(unittest.TestCase):
    """
    Test brake test support in the python api.
    """

    def setUp(self):
        self.robot = Robot(API_VERSION)

    def tearDown(self):
        if hasattr(self, 'robot'):
            self.robot._release()
            self.robot = None

    def test_required(self):
        """
        We expect the method to return true, if the brake test is required.
        """
        mock = BrakeTestMock()
        mock.start()
        mock.advertise_brake_test_required_service()

        mock.set_is_brake_test_required_state(IsBrakeTestRequiredResult.REQUIRED)

        res = self.robot.is_brake_test_required()

        self.assertTrue(res)

        mock.stop()
        mock.join()

    def test_not_required(self):
        """
        We expect the method to return false, if the brake test is not required.
        """
        mock = BrakeTestMock()
        mock.start()
        mock.advertise_brake_test_required_service()

        mock.set_is_brake_test_required_state(IsBrakeTestRequiredResult.NOT_REQUIRED)

        res = self.robot.is_brake_test_required()

        self.assertFalse(res)

        mock.stop()
        mock.join()

    def test_unknown_if_required(self):
        """
        We expect the method to return false, if the brake test is not required.
        """
        mock = BrakeTestMock()
        mock.start()
        mock.advertise_brake_test_required_service()

        mock.set_is_brake_test_required_state(IsBrakeTestRequiredResult.UNKNOWN)

        self.assertRaises(
            rospy.ROSException,
            Robot.is_brake_test_required,
            self.robot
        )

        mock.stop()
        mock.join()

    def test_execute_success(self):
        """Execute a successful brake test"""
        mock = BrakeTestMock()
        mock.start()
        mock.advertise_brake_test_execute_service()

        mock.set_brake_test_execute_success()
        self.robot.execute_brake_test()

        mock.stop()
        mock.join()

    def test_execute_failure(self):
        """Execute a failing brake test"""
        mock = BrakeTestMock()
        mock.start()
        mock.advertise_brake_test_execute_service()

        mock.set_brake_test_execute_failure()

        try:
            self.robot.execute_brake_test()
        except Exception as e:
            # Testing wether it is the right exception
            self.assertIsInstance(e, RobotBrakeTestException)

        mock.stop()
        mock.join()

    def test_service_call_fail(self):
        """Checking calling the api methods with no service available
        produces the expected Exception"""
        self.assertRaises(
            rospy.ROSException,
            Robot.is_brake_test_required,
            self.robot
        )

        self.assertRaises(
            rospy.ROSException,
            Robot.execute_brake_test,
            self.robot
        )

    def fake_get_execute_brake_test_service(obj):
        """Fake method for mock a service call raising a ROSException"""
        def fake_callable():
            raise rospy.exceptions.ROSException("MOCK FAIL")
        return fake_callable

    @patch.object(Robot, '_get_execute_brake_test_service', fake_get_execute_brake_test_service)
    def test_execute_service_call_failure(self):
        """Execute a failing brake test"""
        mock = BrakeTestMock()
        mock.start()
        mock.advertise_brake_test_execute_service()

        try:
            self.robot.execute_brake_test()
        except Exception as e:
            # Testing wether it is the right exception
            self.assertIsInstance(e, rospy.ROSException)
            # Checking that the Exception was caused by the mock
            self.assertIn("MOCK FAIL", str(e))

        mock.stop()
        mock.join()


if __name__ == '__main__':
    import rostest
    rospy.init_node('test_api_brake_test')
    rostest.rosrun('pilz_robot_programming', 'test_api_brake_test', TestAPIBrakeTest)
