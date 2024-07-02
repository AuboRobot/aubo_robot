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
import rospy
from pilz_robot_programming import *
from pilz_industrial_motion_testutils.xml_testdata_loader import *
from pilz_industrial_motion_testutils.integration_test_utils import *
from pilz_industrial_motion_testutils.robot_motion_observer import RobotMotionObserver
from pathlib import Path

_TEST_DATA_FILE_NAME = Path(__file__).parent.parent.absolute() / Path("test_data/test_data.xml")
PLANNING_GROUP_NAME = "manipulator"
API_VERSION = "1"


class TestAPIPause(unittest.TestCase):

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

    def test_pause_move_resume_succeed(self):
        """ Test the sequence of pause, move, resume, move succeed

            Test sequence:
                1. Trigger pause.
                2. Start robot motion.
                3. Trigger resume.

            Test Results:
                1. -
                2. Robot does not move.
                3. Robot starts motion and moves to goal.
        """

        # 1. trigger pause
        self.robot.pause()

        # 2. start the robot motion
        move_thread = MoveThread(self.robot, self.ptp, RobotMoveFailed)
        move_thread.start()
        # no movement
        self.assertFalse(self.robot_motion_observer.is_robot_moving(self._SLEEP_TIME_S,
                                                                    self._TOLERANCE_FOR_MOTION_DETECTION_RAD))

        # 3. trigger resume
        self.robot.resume()
        self.assertTrue(self.robot_motion_observer.wait_motion_start(
            move_tolerance=self._TOLERANCE_FOR_MOTION_DETECTION_RAD, sleep_interval=self._SLEEP_TIME_S))

        rospy.loginfo("Wait until thread/command finished to evaluate result...")
        move_thread.join()
        self.assertFalse(move_thread.exception_thrown)

    def test_move_pause_resume_succeed(self):
        """ Test the sequence of move, pause, resume, move succeed

            Test sequence:
                1. Start robot motion and wait for the motion to start.
                2. Trigger pause and wait for the motion to stop.
                3. Trigger resume.

            Test Results:
                1. Robot moves.
                2. Robot pauses motion.
                3. Robot starts motion again and moves to goal.
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

        # 3. trigger resume
        self.robot.resume()

        rospy.loginfo("Wait until thread/command finished to evaluate result...")
        move_thread.join()
        self.assertFalse(move_thread.exception_thrown)

    def test_move_pause_stop_resume_move_succeed(self):
        """ Test the sequence of move, pause, stop, resume, move, succeed.

            Test sequence:
                1. Start robot motion and wait until motion starts.
                2. Trigger pause.
                3. Trigger stop.
                4. Trigger resume.
                5. Start robot motion again

            Test Results:
                1. Robot moves.
                2. Robot pauses motion.
                3. -
                4. Robot does not move
                5. Robot moves again
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

        # 3. trigger stop
        self.robot.stop()

        # 4. trigger resume
        self.robot.resume()
        # no movement
        self.assertFalse(self.robot_motion_observer.is_robot_moving(self._SLEEP_TIME_S*3,
                                                                    self._TOLERANCE_FOR_MOTION_DETECTION_RAD))
        # wait thread ends
        move_thread.join()
        # move stopped results in exception
        self.assertTrue(move_thread.exception_thrown)

        # 5. start the robot motion again
        move_thread2 = MoveThread(self.robot, self.ptp, RobotMoveFailed)
        move_thread2.start()
        self.assertTrue(self.robot_motion_observer.wait_motion_start(
            move_tolerance=self._TOLERANCE_FOR_MOTION_DETECTION_RAD, sleep_interval=self._SLEEP_TIME_S))
        move_thread2.join()
        self.assertFalse(move_thread2.exception_thrown)

    def test_pause_resume_move_succeed(self):
        """ Test the sequence of pause, resume, move, succeed.

            Test sequence:
                1. Trigger pause.
                2. Trigger resume.
                3. Start robot motion and wait until motion starts.

            Test Results:
                1. -
                2. -
                3. Robot moves again
        """
        # 1 trigger pause
        self.robot.pause()

        # 2 trigger resume
        self.robot.resume()

        # 3 start the robot motion
        move_thread = MoveThread(self.robot, self.ptp, RobotMoveFailed)
        move_thread.start()
        self.assertTrue(self.robot_motion_observer.wait_motion_start(
            move_tolerance=self._TOLERANCE_FOR_MOTION_DETECTION_RAD, sleep_interval=self._SLEEP_TIME_S))
        # wait thread ends
        move_thread.join()
        # move stopped results in exception
        self.assertFalse(move_thread.exception_thrown)

    def test_pause_move_stop(self):
        """ Test the sequence of pause, move, stop.

            Test sequence:
                1. Trigger pause.
                2. Start robot motion.
                3. Trigger stop.

            Test Results:
                1. -
                2. Robot wait for resume, not moving
                3. Robot stop moving, exception thrown
        """
        # 1 trigger pause
        self.robot.pause()

        # 2 start the robot motion, robot not moving
        move_thread = MoveThread(self.robot, self.ptp, RobotMoveFailed)
        move_thread.start()
        self.assertFalse(self.robot_motion_observer.is_robot_moving(self._SLEEP_TIME_S,
                                                                    self._TOLERANCE_FOR_MOTION_DETECTION_RAD))

        # 3 trigger stop
        self.robot.stop()

        # wait thread ends
        move_thread.join()
        # move stopped results in exception
        self.assertTrue(move_thread.exception_thrown)

    def test_pause_pause_move_resume_succeed(self):
        """ Test the sequence of pause, pause, move, resume, succeed.
        Only one pause is buffered.

            Test sequence:
                1. Trigger pause two times.
                2. Start robot motion.
                3. Trigger resume.

            Test Results:
                1. -
                2. Robot wait for resume, not moving.
                3. Robot moves successfully.
        """
        # 1. trigger pause twice
        self.robot.pause()
        self.robot.pause()

        # 2. start the robot motion
        move_thread = MoveThread(self.robot, self.ptp, RobotMoveFailed)
        move_thread.start()
        # no movement
        self.assertFalse(self.robot_motion_observer.is_robot_moving(self._SLEEP_TIME_S,
                                                                    self._TOLERANCE_FOR_MOTION_DETECTION_RAD))

        # 3. trigger resume
        self.robot.resume()
        self.assertTrue(self.robot_motion_observer.wait_motion_start(
            move_tolerance=self._TOLERANCE_FOR_MOTION_DETECTION_RAD, sleep_interval=self._SLEEP_TIME_S))

        move_thread.join()
        self.assertFalse(move_thread.exception_thrown)

    def test_move_resume_succeed(self):
        """ Test the sequence of move, resume, succeed.
        Resume during motion has no effect.

            Test sequence:.
                1. Start robot motion.
                2. Trigger resume.

            Test Results:
                1. Robot moves.
                2. Robot moves successfully.
        """
        # 1. start the robot motion
        move_thread = MoveThread(self.robot, self.ptp, RobotMoveFailed)
        move_thread.start()
        self.assertTrue(self.robot_motion_observer.wait_motion_start(
            move_tolerance=self._TOLERANCE_FOR_MOTION_DETECTION_RAD, sleep_interval=self._SLEEP_TIME_S))

        # 2. trigger resume
        self.robot.resume()

        move_thread.join()
        self.assertFalse(move_thread.exception_thrown)

    def test_pause_move_pause_resume_pause_resume_succeed(self):
        """ Test the sequence of pause, move, pause, resume, pause, resume, succeed.

            Test sequence:.
                1. Trigger pause.
                2. Start robot motion.
                3. Trigger pause.
                4. Trigger resume wait until robot moves.
                5. Trigger pause.
                6. Trigger resume wait until robot moves..

            Test Results:
                1. -
                2. Robot does not move.
                3. -
                4. Robot starts to move.
                5. Robot movement paused.
                6. Robot continues to move.
        """
        # 1. trigger pause
        self.robot.pause()

        # 2. start the robot motion
        move_thread = MoveThread(self.robot, self.ptp, RobotMoveFailed)
        move_thread.start()
        # no movement
        self.assertFalse(self.robot_motion_observer.is_robot_moving(self._SLEEP_TIME_S,
                                                                    self._TOLERANCE_FOR_MOTION_DETECTION_RAD))

        # 3. trigger pause
        self.robot.pause()

        # 4. trigger resume
        self.robot.resume()
        self.assertTrue(self.robot_motion_observer.wait_motion_start(
            move_tolerance=self._TOLERANCE_FOR_MOTION_DETECTION_RAD, sleep_interval=self._SLEEP_TIME_S))

        # 5. trigger pause
        self.robot.pause()

        # 6. trigger resume
        self.robot.resume()
        self.assertTrue(self.robot_motion_observer.wait_motion_start(
            move_tolerance=self._TOLERANCE_FOR_MOTION_DETECTION_RAD, sleep_interval=self._SLEEP_TIME_S))

        move_thread.join()
        self.assertFalse(move_thread.exception_thrown)

    def test_pause_stop_move_succeed(self):
        """ Test the sequence of pause, stop, move, succeed.
        Test stop will erase pause request

            Test sequence:.
                1. Trigger pause.
                2. Trigger stop.
                3. Start robot motion.

            Test Results:
                1. -
                2. -
                3. Robot moves successfully.
        """
        # 1. trigger pause
        self.robot.pause()

        # 2. trigger stop
        self.robot.stop()

        # 3. start the robot motion
        move_thread = MoveThread(self.robot, self.ptp, RobotMoveFailed)
        move_thread.start()
        self.assertTrue(self.robot_motion_observer.wait_motion_start(
            move_tolerance=self._TOLERANCE_FOR_MOTION_DETECTION_RAD, sleep_interval=self._SLEEP_TIME_S))
        move_thread.join()
        self.assertFalse(move_thread.exception_thrown)

    def test_stop_move_succeed(self):
        """ Test the sequence of stop, move, succeed.
        Test stop before move has no effect

            Test sequence:.
                1. Trigger stop.
                2. Start robot motion.

            Test Results:
                1. -
                2. Robot moves successfully.
        """
        # 1. trigger stop
        self.robot.stop()

        # 2. start the robot motion
        move_thread = MoveThread(self.robot, self.ptp, RobotMoveFailed)
        move_thread.start()
        self.assertTrue(self.robot_motion_observer.wait_motion_start(
            move_tolerance=self._TOLERANCE_FOR_MOTION_DETECTION_RAD, sleep_interval=self._SLEEP_TIME_S))
        move_thread.join()
        self.assertFalse(move_thread.exception_thrown)

    def test_resume_move_succeed(self):
        """ Test the sequence of resume, move, succeed.
        Test resume before move has no effect

            Test sequence:.
                1. Trigger resume.
                2. Start robot motion.

            Test Results:
                1. -
                2. Robot moves successfully.
        """
        # 1. trigger resume
        self.robot.resume()

        # 2. start the robot motion
        move_thread = MoveThread(self.robot, self.ptp, RobotMoveFailed)
        move_thread.start()
        self.assertTrue(self.robot_motion_observer.wait_motion_start(
            move_tolerance=self._TOLERANCE_FOR_MOTION_DETECTION_RAD, sleep_interval=self._SLEEP_TIME_S))
        move_thread.join()
        self.assertFalse(move_thread.exception_thrown)

    def test_move_pause_move_exception_resume_succeed(self):
        """ Test the sequence of move, pause, move, exception
        Test multiple call of move will result in exception

            Test sequence:.
                1. Start robot motion in separate thread.
                2. Trigger pause
                3. Start robot motion in current thread.
                4. Trigger Resume

            Test Results:
                1. Robot starts to move.
                2. Robot motion paused.
                3. Exception throw in current thread.
                4. Robot continues to move.
        """
        # 1. start the robot motion in separate thread
        move_thread = MoveThread(self.robot, self.ptp, RobotMoveFailed)
        move_thread.start()
        self.assertTrue(self.robot_motion_observer.wait_motion_start(
            move_tolerance=self._TOLERANCE_FOR_MOTION_DETECTION_RAD, sleep_interval=self._SLEEP_TIME_S))

        # 2. trigger pause
        self.robot.pause()

        # 3. start the robot motion in current thread
        try:
            self.robot.move(self.ptp)
        except RobotMoveAlreadyRunningError:
            pass
        else:
            self.fail('Parallel call to move is not allowed.')

        # 4. trigger resume
        self.robot.resume()
        move_thread.join()
        self.assertFalse(move_thread.exception_thrown)

    def test_move_pause_resume_stop_via_service(self):
        """ Test the sequence of move, pause(service call), resume(service call), stop(service call)

            Test sequence:
                1. Start robot motion and wait for the motion to start.
                2. Trigger pause via service and wait for the motion to stop.
                3. Trigger resume via service and wait for the motion to start.
                4. Trigger stop via service.

            Test Results:
                1. Robot moves.
                2. Robot pauses motion.
                3. Robot starts motion again and moves to goal.
                4. Robot motion stops.
        """

        # 1. start the robot motion
        move_thread = MoveThread(self.robot, self.ptp, RobotMoveFailed)
        move_thread.start()
        self.assertTrue(self.robot_motion_observer.wait_motion_start(
            move_tolerance=self._TOLERANCE_FOR_MOTION_DETECTION_RAD, sleep_interval=self._SLEEP_TIME_S))

        # 2. trigger pause via service
        rospy.wait_for_service(self.robot._PAUSE_TOPIC_NAME)
        pause = rospy.ServiceProxy(self.robot._PAUSE_TOPIC_NAME, Trigger)
        pause()
        self.assertTrue(self.robot_motion_observer.wait_motion_stop(self._WAIT_CMD_STOP_TIME_OUT_S,
                                                                    self._TOLERANCE_FOR_MOTION_DETECTION_RAD))

        # 3. trigger resume via service
        rospy.wait_for_service(self.robot._RESUME_TOPIC_NAME)
        resume = rospy.ServiceProxy(self.robot._RESUME_TOPIC_NAME, Trigger)
        resume()
        self.assertTrue(self.robot_motion_observer.wait_motion_start(
            move_tolerance=self._TOLERANCE_FOR_MOTION_DETECTION_RAD, sleep_interval=self._SLEEP_TIME_S))

        # 4. trigger stop via service
        rospy.wait_for_service(self.robot._STOP_TOPIC_NAME)
        stop = rospy.ServiceProxy(self.robot._STOP_TOPIC_NAME, Trigger)
        stop()
        move_thread.join()
        self.assertTrue(move_thread.exception_thrown)

    def test_pause_and_new_command(self):
        """Tests a time critical edge case of the pause/stop behavior, the edge case is explained in detail below.

            Note:
            To avoid an unstable/time critical test, private functions are called to simulate the time critical
            edge case of the pause/stop behavior.

            Detailed explanation of edge case (activity sequence):
            - The move() function of class Robot is called. The move() function starts working and checks
            the MoveControlState in _move_execution_loop().
            - Before the _execute() method on the concrete command is called, a pause/stop is triggered.
            - The _move_execution_loop() function continues its execution and calls the execute method of the
              concrete command.
            - If everything works correctly, the MoveControlState is atomically checked and the command execution
              is halted/stopped because the pause/stop is detected.
        """
        self.robot.pause()
        cmd_for_testing = Ptp(goal=self.test_data.get_joints("PTPJointValid", PLANNING_GROUP_NAME))
        res = cmd_for_testing._execute(self.robot)
        self.assertEqual(self.robot._STOPPED, res)  # Command is execution is halted/stopped.

    def test_pause_sequence(self):
        """ Test pausing a sequence. This is not implemented yet and should result in a failure of the robot motion.

            Test sequence:
                1. Start robot motion via sequence in separate thread.
                2. Trigger pause

            Test Results:
                1. Robot moves.
                2. Robot motion stops and an exception is throw in the move thread.
        """

        # 1. start the robot motion
        seq = Sequence()
        seq.append(self.ptp, 0.0)
        move_thread = MoveThread(self.robot, seq, RobotMoveFailed)
        move_thread.start()
        self.assertTrue(self.robot_motion_observer.wait_motion_start(
            move_tolerance=self._TOLERANCE_FOR_MOTION_DETECTION_RAD, sleep_interval=self._SLEEP_TIME_S))

        # 2. trigger pause
        self.robot.pause()
        self.assertTrue(self.robot_motion_observer.wait_motion_stop(self._WAIT_CMD_STOP_TIME_OUT_S,
                                                                    self._TOLERANCE_FOR_MOTION_DETECTION_RAD))
        move_thread.join()
        self.assertTrue(move_thread.exception_thrown)


if __name__ == '__main__':
    import rostest
    rospy.init_node('test_api_pause')
    rostest.rosrun('pilz_robot_programming', 'test_api_pause', TestAPIPause)
