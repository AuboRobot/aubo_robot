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

DEFAULT_PTP_VEL = 0.5
PTP_VEL_PICK = 0.05

_REQUIRED_API_VERSION = "1"

def start_program():
    print("Executing " + __file__)

    robot = Robot(_REQUIRED_API_VERSION)
    _test_repeat_ptp_joint(robot)
    _test_repeat_ptp_pose(robot)
    _test_repeat_lin_pose(robot)
    _test_repeat_circ_pose(robot)
    _test_repeat_ptp_in_sequence(robot)


def _test_repeat_ptp_joint(robot):
    """Tests repeating a ptp joint motion.

        Test Sequence:
          1. Move to start position.
          2. Move away from the singularity via ptp joint.
          3. Repeat the previous command

        Expected Results:
          1. Robot moves to start position.
          2. Robot moves away from the singularity.
          3. No INVALID_GOAL is returned from controller
    """
    if askPermission(_test_repeat_ptp_joint.__name__) == 0:
        return

    robot.move(Ptp(goal=[0, 0, 0, 0, 0, 0]))
    robot.move(Ptp(goal=[0, -0.78, 0.78, 0, 1.56, 0]))
    robot.move(Ptp(goal=[0, -0.78, 0.78, 0, 1.56, 0]))

    askSuccess(_test_repeat_ptp_joint.__name__, 'No INVALID_GOAL should appear from controller.')


def _test_repeat_ptp_pose(robot):
    """Tests repeating a ptp pose motion.

        Test Sequence:
          1. Move to start position.
          2. Move away from the singularity.
          3. Move to a pose via ptp
          4. Repeat the previous command

        Expected Results:
          1. Robot moves to start position.
          2. Robot moves away from the singularity.
          3. Robot moves to the given position.
          4. No INVALID_GOAL is returned from controller
    """
    if askPermission(_test_repeat_ptp_pose.__name__) == 0:
        return

    robot.move(Ptp(goal=[0, 0, 0, 0, 0, 0]))
    robot.move(Ptp(goal=[0, -0.78, 0.78, 0, 1.56, 0]))

    robot.move(Ptp(goal=Pose(position=Point(-0.46, -0.21, 0.19), orientation=from_euler(0, -3.14, -0.25))))
    robot.move(Ptp(goal=Pose(position=Point(-0.46, -0.21, 0.19), orientation=from_euler(0, -3.14, -0.25))))

    askSuccess(_test_repeat_ptp_pose.__name__, 'No INVALID_GOAL should appear from controller.')


def _test_repeat_lin_pose(robot):
    """Tests repeating a lin pose motion.

        Test Sequence:
          1. Move to start position via ptp.
          2. Move to a pose via lin
          3. Repeat the previous command

        Expected Results:
          1. Robot moves to start position.
          2. Robot moves to the given position.
          3. No INVALID_GOAL is returned from controller
    """
    if askPermission(_test_repeat_lin_pose.__name__) == 0:
        return

    robot.move(Ptp(goal=[0, 0, 0, 0, 0, 0]))
    robot.move(Ptp(goal=[0, -0.78, 0.78, 0, 1.56, 0]))
    robot.move(Ptp(goal=Pose(position=Point(-0.46, -0.21, 0.19), orientation=from_euler(0, -3.14, -0.25))))
    robot.move(Lin(goal=Pose(position=Point(-0.46, -0.21, 0.19)), vel_scale=PTP_VEL_PICK))

    askSuccess(_test_repeat_lin_pose.__name__, 'No INVALID_GOAL should appear from controller.')


def _test_repeat_circ_pose(robot):
    """Test a circ motion

        Test Sequence:
          1. Move to start position via ptp.
          2. Move half a circle.
          3. Repeat the previous command

        Expected Results:
          1. Robot moves to start position.
          2. Robot moves half a circle.
          3. No INVALID_GOAL is returned from controller
    """
    if askPermission(_test_repeat_circ_pose.__name__) == 0:
        return

    robot.move(Ptp(goal=[0, 0, 0, 0, 0, 0]))
    robot.move(Ptp(goal=[0, -0.78, 0.78, 0, 1.56, 0]))
    robot.move(Ptp(goal=Pose(position=Point(-0.46, -0.21, 0.19), orientation=from_euler(0, -3.14, -0.25))))

    robot.move(Circ(goal=Pose(position=Point(-0.460, 0, 0.19),
                    orientation=from_euler(0, -3.14, -0.25)), vel_scale=PTP_VEL_PICK,
                    interim=Point(-0.355, -0.105, 0.19)))

    try:
      robot.move(Circ(goal=Pose(position=Point(-0.460, 0, 0.19),
                      orientation=from_euler(0, -3.14, -0.25)), vel_scale=PTP_VEL_PICK,
                      interim=Point(-0.355, -0.105, 0.19)))
    except RobotMoveFailed:
      pass

    askSuccess(_test_repeat_circ_pose.__name__, 'No INVALID_GOAL should appear from controller. The following error '
                                                'is o.k.: Failed to create path object for circle. Circle : Plane for '
                                                'motion is not properly defined.')


def _test_repeat_ptp_in_sequence(robot):
    """Tests repeating a ptp joint motion in a sequence.

        Test Sequence:
          1. Perform the following commands in a command sequence:
            - Move to start position.
            - Move away from the singularity via ptp joint.
            - Repeat the previous command.

        Expected Results:
          1. Robot performs the following movements without INVALID_GOAL or other error returned from controller:
            - Moves to start position.
            - Moves away from the singularity.
    """
    if askPermission(_test_repeat_ptp_in_sequence.__name__) == 0:
        return

    seq = Sequence()
    seq.append(Ptp(goal=[0, 0, 0, 0, 0, 0]), blend_radius=0)
    seq.append(Ptp(goal=[0, -0.78, 0.78, 0, 1.56, 0]), blend_radius=0)
    seq.append(Ptp(goal=[0, -0.78, 0.78, 0, 1.56, 0]), blend_radius=0)

    robot.move(seq)

    askSuccess(_test_repeat_ptp_in_sequence.__name__, 'No INVALID_GOAL or other error should appear from controller.')


if __name__ == "__main__":
    # Init a ros node
    rospy.init_node('acceptance_test_node', anonymous=True)

    start_program()
