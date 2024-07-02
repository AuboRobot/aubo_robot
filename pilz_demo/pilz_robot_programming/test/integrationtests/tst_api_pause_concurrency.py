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
from pathlib import Path

_TEST_DATA_FILE_NAME = Path(__file__).parent.parent.absolute() / Path("test_data/test_data.xml")
PLANNING_GROUP_NAME = "manipulator"
API_VERSION = "1"


class TestAPIPauseConcurrency(unittest.TestCase):

    _SLEEP_TIME_S = 0.01
    _TOLERANCE_FOR_MOTION_DETECTION_RAD = 0.01
    _WAIT_CMD_STOP_TIME_OUT_S = 3

    def setUp(self):
        self.test_data = XmlTestdataLoader(_TEST_DATA_FILE_NAME)
        self.robot = Robot(API_VERSION)
        self.robot_motion_observer = RobotMotionObserver(PLANNING_GROUP_NAME)
        self.robot.move(Ptp(goal=self.test_data.get_joints("ZeroPose", PLANNING_GROUP_NAME)))
        self.ptp = Ptp(goal=self.test_data.get_joints("PTPJointLarge", PLANNING_GROUP_NAME))

    def tearDown(self):
        if hasattr(self, 'robot'):
            self.robot._release()
            self.robot = None
        if hasattr(self, 'test_data'):
            del self.test_data

    def move_ptp(self):
        self.robot.move(self.ptp)

    def test_pause_move_concurrency(self):
        """ Test the concurrent situation of pause and move

            Test sequence:.
                1. Trigger the pause and move at the same time
                2. Trigger resume

            Test Results:
                1. Robot does not move.
                2. Robot moves successfully
        """

        lock = threading.Lock()
        cv = threading.Condition(lock)

        thread_pause = threading.Thread(target=waited_trigger, args=(cv, self.robot.pause))
        thread_pause.start()
        thread_move = threading.Thread(target=waited_trigger, args=(cv, self.move_ptp))
        thread_move.start()

        rospy.sleep(1)

        with cv:
            cv.notify_all()

        self.assertFalse(self.robot_motion_observer.is_robot_moving(self._SLEEP_TIME_S,
                                                                    self._TOLERANCE_FOR_MOTION_DETECTION_RAD))

        self.robot.resume()

        thread_pause.join()
        thread_move.join()

    def test_resume_stop_concurrency_when_motion_is_paused(self):
        """ Test the concurrent situation of resume and stop when motion is paused

            Test sequence:.
                1. Trigger Move and wait until move starts
                2. Trigger Pause and wait until move stops
                3. Trigger resume and stop at the same time

            Test Results:
                1. Robot starts to move.
                2. Robot move stops
                3. Robot does not move, exception throw
        """

        # 1. start the robot motion
        move_thread = MoveThread(self.robot, self.ptp, RobotMoveFailed)
        move_thread.start()
        self.assertTrue(self.robot_motion_observer.wait_motion_start(
            move_tolerance=self._TOLERANCE_FOR_MOTION_DETECTION_RAD, sleep_interval=self._SLEEP_TIME_S))

        # 2. trigger pause
        self.robot.pause()
        self.assertTrue(self.robot_motion_observer.wait_motion_stop(self._WAIT_CMD_STOP_TIME_OUT_S,
                                                                    self._TOLERANCE_FOR_MOTION_DETECTION_RAD))

        # 3. trigger stop/resume at the same time
        lock = threading.Lock()
        cv = threading.Condition(lock)

        thread_stop = threading.Thread(target=waited_trigger, args=(cv, self.robot.stop))
        thread_stop.start()
        thread_resume = threading.Thread(target=waited_trigger, args=(cv, self.robot.resume))
        thread_resume.start()

        rospy.sleep(1)

        with cv:
            cv.notify_all()

        thread_stop.join()
        thread_resume.join()

        move_thread.join()
        self.assertTrue(move_thread.exception_thrown)

    def test_resume_move_concurrency_when_paused_without_move(self):
        """ Test the concurrent situation of resume and move pause is requested

            Test sequence:.
                1. Trigger pause
                2. Trigger resume and move at the same time

            Test Results:
                1. -
                2. Robot moves successfully
        """
        # 1. trigger pause
        self.robot.pause()

        # 2. trigger stop/resume at the same time
        lock = threading.Lock()
        cv = threading.Condition(lock)

        thread_move = threading.Thread(target=waited_trigger, args=(cv, self.move_ptp))
        thread_move.start()
        thread_resume = threading.Thread(target=waited_trigger, args=(cv, self.robot.resume))
        thread_resume.start()

        rospy.sleep(1)

        with cv:
            cv.notify_all()

        thread_move.join()
        thread_resume.join()


if __name__ == '__main__':
    import rostest
    rospy.init_node('test_api_pause_concurrency')
    rostest.rosrun('pilz_robot_programming', 'test_api_pause_concurrency', TestAPIPauseConcurrency)
