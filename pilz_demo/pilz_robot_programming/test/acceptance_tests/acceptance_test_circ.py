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

from geometry_msgs.msg import Point
from pilz_robot_programming.robot import *
from pilz_robot_programming.commands import *
from pilz_industrial_motion_testutils.acceptance_test_utils import askPermission, askSuccess

VELOCITY_HIGH = 0.3
VELOCITY_LOW = 0.1
DEFAULT_ACC = 0.1

_REQUIRED_API_VERSION = "1"

def start_program():
    print("Executing " + __file__)

    robot = Robot(_REQUIRED_API_VERSION)
    _test_circ_center(robot)
    _test_circ_interim(robot)
    _test_circ_small_vertical(robot)


def _test_circ_center(robot):
    """Tests circ motions given a center point.

        Test Sequence:
          1. Move to start position.
          2. Move a quarter circle two times.
          3. Move a quarter circle in the other direction two times.

        Expected Results:
          1. Robot moves to start position.
          2. Robot moves a quarter circle two times.
          3. Robot moves a quarter circle in the other direction two times.
    """
    if askPermission(_test_circ_center.__name__) == 0:
        return

    START_POSE = Pose(position=Point(0.0, 0.4, 0.7), orientation=from_euler(0.0, 0.0, 0.0))
    CENTER_POINT = Point(0.0, 0.0, 0.7)

    robot.move(Ptp(goal=[1.57, 0.0, -1.57, 0.0, 1.57, 0.0]))
    robot.move(Ptp(goal=START_POSE))

    robot.move(Circ(goal=Pose(position=Point(0.4, 0.0, 0.7), orientation=from_euler(-1.57, 0.0, 0.0)),
                    center=CENTER_POINT, vel_scale=VELOCITY_HIGH, acc_scale=DEFAULT_ACC))
    robot.move(Circ(goal=Pose(position=Point(0.0, -0.4, 0.7), orientation=from_euler(-3.14, 0.0, 0.0)),
                    center=CENTER_POINT, vel_scale=VELOCITY_HIGH, acc_scale=DEFAULT_ACC))
    robot.move(Circ(goal=Pose(position=Point(0.4, 0.0, 0.7), orientation=from_euler(-1.57, 0.0, 0.0)),
                    center=CENTER_POINT, vel_scale=VELOCITY_HIGH, acc_scale=DEFAULT_ACC))
    robot.move(Circ(goal=START_POSE,
                    center=CENTER_POINT, vel_scale=VELOCITY_HIGH, acc_scale=DEFAULT_ACC))

    askSuccess(_test_circ_center.__name__,
               'The robot should have moved to a start position, then a quarter circle two times and then a quarter'
               + ' circle back two times.')


def _test_circ_interim(robot):
    """Tests circ motions given an interim point.

        Test Sequence:
          1. Move to start position.
          2. Move 5/12 of a circle and back to the start position.
          3. Move 5/8 of a circle.

        Expected Results:
          1. Robot moves to start position.
          2. Robot moves 5/12 of a circle and back to the start position.
          3. Robot moves 5/8 of a circle.
    """
    if askPermission(_test_circ_interim.__name__) == 0:
        return

    START_POSE = Pose(position=Point(0.0, 0.4, 0.7), orientation=from_euler(-1.57, 0.0, 0.0))
    INTERIM_POINT = Point(0.4, 0.0, 0.7)

    robot.move(Ptp(goal=[1.57, 0.0, -1.57, 0.0, 1.57, 0.0]))
    robot.move(Ptp(goal=START_POSE))

    robot.move(Circ(goal=Pose(position=Point(0.35355, -0.18709, 0.7), orientation=from_euler(0.0, 0.0, 2.0951)),
                   interim=INTERIM_POINT, vel_scale=VELOCITY_LOW, acc_scale=DEFAULT_ACC))
    robot.move(Circ(goal=START_POSE,
                   interim=INTERIM_POINT, vel_scale=VELOCITY_LOW, acc_scale=DEFAULT_ACC))
    robot.move(Circ(goal=Pose(position=Point(-0.37204, -0.14693, 0.7), orientation=from_euler(0.0, 0.0, 3.9279)),
                    interim=INTERIM_POINT, vel_scale=VELOCITY_LOW, acc_scale=DEFAULT_ACC))

    askSuccess(_test_circ_interim.__name__,
               'The robot should have moved to a start position, then 5/12 of a circle, back to the start position'
               + ' and then 5/8 of a circle.')


def _test_circ_small_vertical(robot):
    """Tests circ motion on a vertical circle.

        Test Sequence:
          1. Move to start position.
          2. Move 1.99 PI along a small vertical circle.

        Expected Results:
          1. Robot moves to start position.
          2. Robot moves 1.99 PI along a small vertical circle.
    """
    if askPermission(_test_circ_small_vertical.__name__) == 0:
        return

    robot.move(Ptp(goal=[0.0, 0.0, 1.57, 0.0, 0.0, 0.0]))
    robot.move(Ptp(goal=Pose(position=Point(-0.5, 0.2, 0.4), orientation=from_euler(0.0, -1.57, -1.57))))
    robot.move(Circ(goal=Pose(position=Point(-0.5, 0.19686, 0.39995), orientation=from_euler(0.0, -1.57, -1.57)),
                    interim=Point(-0.5, 0.3, 0.5), vel_scale=VELOCITY_LOW, acc_scale=DEFAULT_ACC))

    askSuccess(_test_circ_small_vertical.__name__,
               'The robot should have moved to a start position and then 1.99 PI along a small vertical circle.')


if __name__ == "__main__":
    # Init a ros node
    rospy.init_node('acceptance_test_circ_node', anonymous=True)

    start_program()
