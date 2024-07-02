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

from pilz_robot_programming import *
from pilz_industrial_motion_testutils.acceptance_test_utils import askPermission, askSuccess
from pilz_industrial_motion_testutils.integration_test_utils import *
from pilz_industrial_motion_testutils.robot_motion_observer import RobotMotionObserver

_SLEEP_TIME_S = 0.01
_TOLERANCE_FOR_MOTION_DETECTION_RAD = 0.3
_DEFAULT_VEL_SCALE = 0.1
_REQUIRED_API_VERSION = "1"
_PLANNING_GROUP = "manipulator"


def start_program():
    print("Executing " + __file__)

    robot = Robot(_REQUIRED_API_VERSION)
    _test_stop(robot)
    _test_pause_resume(robot)
    _test_pause_stop(robot)
    _test_pause_between_moves(robot)


def _test_stop(robot):
    """Tests stopping a robot motion.

        Test Sequence:
            1. Start robot motion and wait for the motion to start.
            2. Trigger stop.
            3. Start robot motion with different goal.

        Expected Results:
            1. Robot moves.
            2. Robot stops motion before reaching goal.
            3. Robot starts motion and moves to the other goal.
    """

    if askPermission(_test_stop.__name__) == 0:
        return

    _robot_motion_observer = RobotMotionObserver(_PLANNING_GROUP)

    # 1. Create simple ptp command and start thread for movement
    ptp = Ptp(goal=[0, -0.78, 0.78, 0, 1.56, 0], vel_scale=_DEFAULT_VEL_SCALE)
    move_thread = MoveThread(robot, ptp, RobotMoveFailed)
    move_thread.start()
    _robot_motion_observer.wait_motion_start(sleep_interval=_SLEEP_TIME_S,
                                             move_tolerance=_TOLERANCE_FOR_MOTION_DETECTION_RAD)

    # 2. Trigger stop
    robot.stop()

    # Wait for thread to finish
    move_thread.join()

    # 3. Trigger motion with different goal
    robot.move(Ptp(goal=[0.78, 0.0, 0.0, 0, 0, 0], relative=True, vel_scale=_DEFAULT_VEL_SCALE))

    askSuccess(_test_stop.__name__,
               'The robot should have stopped before reaching the upper pick position and continued moving in '
               'another direction.')


def _test_pause_resume(robot):
    """Tests pausing and resuming a robot motion.

        Test Sequence:
            1. Start robot motion and wait for the motion to start.
            2. Trigger pause and wait 5 seconds.
            3. Trigger resume.

        Expected Results:
            1. Robot moves.
            2. Robot stops motion.
            3. Robot starts motion again and moves to goal.
    """
    if askPermission(_test_pause_resume.__name__) == 0:
        return

    _robot_motion_observer = RobotMotionObserver(_PLANNING_GROUP)

    # 1. Create simple ptp command and start thread for movement
    ptp = Ptp(goal=[0, 0.39, -0.39, 0, 0.78, 0], vel_scale=_DEFAULT_VEL_SCALE)
    move_thread = MoveThread(robot, ptp, RobotMoveFailed)
    move_thread.start()
    _robot_motion_observer.wait_motion_start(sleep_interval=_SLEEP_TIME_S,
                                             move_tolerance=_TOLERANCE_FOR_MOTION_DETECTION_RAD)

    # 2. Trigger pause
    robot.pause()
    rospy.sleep(5.0)

    # 3. Trigger resume
    robot.resume()

    # Wait for thread to finish
    move_thread.join()

    askSuccess(_test_pause_resume.__name__,
               'The robot should have paused his movement and continued after approximately 5 seconds.')


def _test_pause_stop(robot):
    """Tests pausing and canceling a robot motion.

        Test Sequence:
            1. Start robot motion and wait for the motion to start.
            2. Trigger pause.
            3. Trigger stop.

        Expected Results:
            1. Robot moves.
            2. Robot stops motion.
            3. Move thread terminates.
    """
    if askPermission(_test_pause_stop.__name__) == 0:
        return

    _robot_motion_observer = RobotMotionObserver(_PLANNING_GROUP)

    # 1. Create simple ptp command and start thread for movement
    ptp = Ptp(goal=[0, -0.78, 0.78, 0, 1.56, 0], vel_scale=_DEFAULT_VEL_SCALE)
    move_thread = MoveThread(robot, ptp, RobotMoveFailed)
    move_thread.start()
    _robot_motion_observer.wait_motion_start(sleep_interval=_SLEEP_TIME_S,
                                             move_tolerance=_TOLERANCE_FOR_MOTION_DETECTION_RAD)

    # 2. Trigger pause
    robot.pause()

    # 3. Trigger stop
    robot.stop()

    # Wait for thread to finish
    move_thread.join()

    askSuccess(_test_pause_stop.__name__,
               'The robot should have paused his movement before reaching the upper pick position '
               'and the test should have terminated immediately.')


def _test_pause_between_moves(robot):
    """Tests pausing the next motion when no motion is active.

        Test Sequence:
            1. Start robot motion and wait for the motion to end.
            2. Trigger pause.
            3. Start robot motion and wait for 5 seconds.
            4. Trigger resume.

        Expected Results:
            1. Robot moves to goal.
            2. Robot does not move.
            3. Robot does not move.
            4. Robot moves to goal.
    """
    if askPermission(_test_pause_between_moves.__name__) == 0:
        return

    # 1. Create simple ptp command and start thread for movement
    ptp = Ptp(goal=[0, -0.78, 0.78, 0, 1.56, 0], vel_scale=_DEFAULT_VEL_SCALE)
    move_thread = MoveThread(robot, ptp, RobotMoveFailed)
    move_thread.start()

    # Wait for thread to finish
    move_thread.join()

    # 2. Trigger pause
    robot.pause()

    # 3. Start another motion
    ptp = Ptp(goal=[0, 0, 0, 0, 0, 0], vel_scale=_DEFAULT_VEL_SCALE)
    move_thread = MoveThread(robot, ptp, RobotMoveFailed)
    move_thread.start()

    rospy.sleep(5.0)

    # 4. Trigger resume
    robot.resume()

    # Wait for thread to finish
    move_thread.join()

    askSuccess(_test_pause_between_moves.__name__,
               'The robot should have moved to the upper pick position and back to [0, 0, 0, 0, 0, 0] after '
               'approximately 5 seconds.')


if __name__ == "__main__":
    # Init a ros node
    rospy.init_node('move_control_acceptance_tests_node', anonymous=True)

    start_program()
