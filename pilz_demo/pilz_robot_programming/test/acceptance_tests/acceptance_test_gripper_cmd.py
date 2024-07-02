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

from pilz_robot_programming.robot import *
from pilz_robot_programming.commands import *
from pilz_industrial_motion_testutils.acceptance_test_utils import askPermission, askSuccess

DEFAULT_PTP_VEL = 0.5

GRIPPER_CLOSED = 0.001
GRIPPER_MIDDLE = 0.01505
GRIPPER_OPEN = 0.0301

_REQUIRED_API_VERSION = "1"

def start_program():
    print("Executing " + __file__)

    robot = Robot(_REQUIRED_API_VERSION)
    _test_neg_gripper_pos(robot)
    _test_too_large_gripper_pos(robot)
    _test_correct_gripper_pos(robot)
    _test_closed_gripper_pos(robot)
    _test_open_gripper_pos(robot)
    _test_gripper_vel_scale(robot)
    _test_gripper_cmd_stress_test(robot)
    _test_rob_gripper_cmd_combi(robot)


def _test_neg_gripper_pos(robot):
    """ Tests a negative gripper position.

        Test Sequence:
            1. Move to start position.
            2. Perform gripper command.

        Expected Results:
            1. Robot moves to start position.
            2. Exception is thrown and error message is displayed.
    """
    if askPermission(_test_neg_gripper_pos.__name__) == 0:
        return
    robot.move(Ptp(goal=[0, 0.007, -1.816, 0, 1.8236, 0], vel_scale=DEFAULT_PTP_VEL))
    try:
        robot.move(Gripper(goal=GRIPPER_CLOSED - 0.01))
    except RobotMoveFailed:
        pass
    else:
        rospy.logerr('Invalid gripper position does not result in exception.')

    askSuccess(_test_neg_gripper_pos.__name__,
               'There should be an error message stating that the gripper position violates the gripper limits.')


def _test_too_large_gripper_pos(robot):
    """ Tests too large gripper position.

        Test Sequence:
            1. Move to start position.
            2. Perform gripper command.

        Expected Results:
            1. Robot moves to start position.
            2. Exception is thrown and error message is displayed.
    """
    if askPermission(_test_too_large_gripper_pos.__name__) == 0:
        return
    robot.move(Ptp(goal=[0, 0.007, -1.816, 0, 1.8236, 0], vel_scale=DEFAULT_PTP_VEL))
    try:
        robot.move(Gripper(goal=GRIPPER_OPEN + 0.01))
    except RobotMoveFailed:
        pass
    else:
        rospy.logerr('Invalid gripper position does not result in exception.')

    askSuccess(_test_too_large_gripper_pos.__name__,
               'There should be an error message stating that the gripper position violates the gripper limits.')


def _test_correct_gripper_pos(robot):
    """ Tests correct gripper position.

        Test Sequence:
            1. Move to start position.
            2. Perform gripper command.

        Expected Results:
            1. Robot moves to start position.
            2. Gripper moved to given position.
    """
    if askPermission(_test_correct_gripper_pos.__name__) == 0:
        return
    robot.move(Ptp(goal=[0, 0.007, -1.816, 0, 1.8236, 0], vel_scale=DEFAULT_PTP_VEL))
    robot.move(Gripper(goal=GRIPPER_MIDDLE))

    askSuccess(_test_correct_gripper_pos.__name__, 'Gripper should have moved.')


def _test_closed_gripper_pos(robot):
    """ Tests smallest gripper position.

        Test Sequence:
            1. Move to start position.
            2. Perform gripper command.

        Expected Results:
            1. Robot moves to start position.
            2. Gripper closed.
    """
    if askPermission(_test_closed_gripper_pos.__name__) == 0:
        return
    robot.move(Ptp(goal=[0, 0.007, -1.816, 0, 1.8236, 0], vel_scale=DEFAULT_PTP_VEL))
    robot.move(Gripper(goal=GRIPPER_CLOSED))

    askSuccess(_test_closed_gripper_pos.__name__, 'Gripper should be closed.')


def _test_open_gripper_pos(robot):
    """ Tests biggest gripper position.

         Test Sequence:
           1. Move to start position.
           2. Perform gripper command.

         Expected Results:
           1. Robot moves to start position.
           2. Gripper completely open.
    """
    if askPermission(_test_open_gripper_pos.__name__) == 0:
        return
    robot.move(Ptp(goal=[0, 0.007, -1.816, 0, 1.8236, 0], vel_scale=DEFAULT_PTP_VEL))
    robot.move(Gripper(goal=GRIPPER_OPEN))

    askSuccess(_test_open_gripper_pos.__name__, 'Gripper should be completely open.')


def _test_gripper_vel_scale(robot):
    """ Tests different velocity scaling of gripper command.

         Test Sequence:
           1. Move to start position.
           2. Perform gripper command with max velocity scale.
           3. Perform gripper command with 0.1 velocity scale.

         Expected Results:
           1. Robot moves to start position.
           2. Gripper completely open with maximal speed.
           3. Gripper completely closed with slower speed
    """
    if askPermission(_test_gripper_vel_scale.__name__) == 0:
        return
    robot.move(Ptp(goal=[0, 0.007, -1.816, 0, 1.8236, 0], vel_scale=DEFAULT_PTP_VEL))
    robot.move(Gripper(goal=GRIPPER_CLOSED, vel_scale=0.1))
    robot.move(Gripper(goal=GRIPPER_OPEN, vel_scale=1))
    robot.move(Gripper(goal=GRIPPER_CLOSED, vel_scale=0.1))

    askSuccess(_test_gripper_vel_scale.__name__, 'Gripper should move with different speed.')


def _test_gripper_cmd_stress_test(robot):
    """ Stress test for gripper cmd.

        Test Sequence:
            1. Move to start position.
            2. Perform a large number of gripper command.

        Expected Results:
            1. Robot moves to start position.
            2. Gripper moves correctly.
    """
    if askPermission(_test_gripper_cmd_stress_test.__name__) == 0:
        return
    robot.move(Ptp(goal=[0, 0.007, -1.816, 0, 1.8236, 0], vel_scale=DEFAULT_PTP_VEL))

    n = 30
    grip_range = GRIPPER_OPEN - GRIPPER_CLOSED
    step_width = grip_range/n
    for i in range(1, n):
        robot.move(Gripper(goal=i * step_width))

    askSuccess(_test_gripper_cmd_stress_test.__name__, 'Gripper should have closed and then opened in %i steps' % n)


def _test_rob_gripper_cmd_combi(robot):
    """ Tests gripper command in combination with move commands.

        Test Sequence:
            For i=1:4
                1. Move to start position.
                2. Perform gripper command.
                3. Move robot to different position.
                4. Perform gripper command.

        Expected Results:
            For i=1:4
                1. Robot moves to start position.
                2. Gripper moves correctly.
                3. Robot move to stated robot position.
                4. Gripper moves correctly.
    """
    if askPermission(_test_rob_gripper_cmd_combi.__name__) == 0:
        return

    n = 5
    grip_range = GRIPPER_MIDDLE - GRIPPER_CLOSED
    step_width = grip_range/n
    for i in range(0, n-1):
        robot.move(Ptp(goal=[0, 0.007, -1.816, 0, 1.8236, 0], vel_scale=DEFAULT_PTP_VEL))
        robot.move(Gripper(goal=GRIPPER_MIDDLE + (i * step_width)))
    
        robot.move(Ptp(goal=[0.5, 0.007, -1.816, 0, 1.8236, 0], vel_scale=DEFAULT_PTP_VEL))
        robot.move(Gripper(goal=GRIPPER_CLOSED + (i * step_width)))

    askSuccess(_test_rob_gripper_cmd_combi.__name__, 'Robot and gripper should have moved alternatingly.')


if __name__ == "__main__":
    # Init a ros node
    rospy.init_node('gripper_acceptance_tests_node', anonymous=True)

    start_program()
