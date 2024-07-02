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

import rospy
import unittest
import subprocess
import signal
from pilz_industrial_motion_testutils.xml_testdata_loader import *
from pilz_industrial_motion_testutils.robot_motion_observer import RobotMotionObserver
from pilz_robot_programming import *
from pathlib import Path

_TEST_DATA_FILE_NAME = Path(__file__).parent.parent.absolute() / Path("test_data/test_data.xml")
_GROUP_NAME = "manipulator"
_API_VERSION = "1"


class TestAPIProgramTermination(unittest.TestCase):
    """
    Test the API behaviour when the python program is terminated.
    """

    _PTP_TEST_NAME = "PTPJointValid"
    _WAIT_TIME_FOR_MOTION_DETECTION_SEC = 8.0

    def setUp(self):
        rospy.loginfo("SetUp called...")
        self.test_data = XmlTestdataLoader(_TEST_DATA_FILE_NAME)
        self.robot_motion_observer = RobotMotionObserver(_GROUP_NAME)

    def tearDown(self):
        rospy.loginfo("TearDown called...")
        if hasattr(self, 'test_data'):
            del self.test_data

    def _get_robot_move_command(self):
        """Build command to move the robot (to be called as subprocess)

        :returns list containing executable with path and appropriate arguments
        """
        ptp_goal = [str(x) for x in self.test_data.get_joints(self._PTP_TEST_NAME, _GROUP_NAME)]
        movecmd = str(Path(__file__).parent.absolute() / Path('movecmd.py')) + ' ptp joint'

        return movecmd.split(" ") + ptp_goal

    def test01_stop_at_program_interrupt(self):
        """
        Test if robot movement is stopped when program is interrupted (Ctrl-c).

        Test Sequence:
            1. Request robot movement in a subprocess.
            2. Send interrupt signal.

        Expected Results:
            1. -
            2. Robot does not move after subprocess has terminated.
        """

        # 1. Start robot movement
        proc = subprocess.Popen(self._get_robot_move_command())

        # Wait until movement is detected
        self.assertTrue(
            self.robot_motion_observer.wait_motion_start(wait_time_out=self._WAIT_TIME_FOR_MOTION_DETECTION_SEC))

        # 2. Send interrupt signal
        proc.send_signal(signal.SIGINT)

        # Wait until process has terminated.
        proc.wait()

        self.assertFalse(self.robot_motion_observer.is_robot_moving())

    def test09_instantiation_after_program_kill(self):
        """
        Test if robot can be instantiated after a program was killed. In case of a failure, the robot can no longer be
        instantiated and a robot movement may still be executed. So this should be the last test case.

        Test Sequence:
            1. Request robot movement in a subprocess.
            2. Send kill signal and wait for termination of the subprocess.
            3. Try to create an instance of Robot.

        Expected Results:
            1. -
            2. -
            3. No exception is thrown
        """

        # 1. Start robot movement
        proc = subprocess.Popen(self._get_robot_move_command())

        # Wait until movement is detected
        self.assertTrue(
            self.robot_motion_observer.wait_motion_start(wait_time_out=self._WAIT_TIME_FOR_MOTION_DETECTION_SEC))

        # 2. Send kill signal
        proc.send_signal(signal.SIGKILL)

        # Wait until process has terminated.
        proc.wait()

        try:
            r = Robot(_API_VERSION)
        except RobotMultiInstancesError:
            self.fail('Instantiation after program kill does throw exception.')
        else:
            del r


if __name__ == '__main__':
    import rostest
    rospy.init_node('test_api_program_termination')
    rostest.rosrun('pilz_robot_programming', 'test_api_program_termination', TestAPIProgramTermination)
