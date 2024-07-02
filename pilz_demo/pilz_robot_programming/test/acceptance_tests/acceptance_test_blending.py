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

from geometry_msgs.msg import Pose, Point
from pilz_robot_programming.robot import *
from pilz_robot_programming.commands import *
from pilz_industrial_motion_testutils.acceptance_test_utils import askPermission, askSuccess

DEFAULT_PTP_VEL = 0.5
DEFAULT_PTP_ACC = 0.5

_REQUIRED_API_VERSION = "1"

def start_program():
    print("Executing " +__file__)
    print('\n\n___\n')

    robot = Robot(_REQUIRED_API_VERSION)

    _testLinLinBlendWithoutOriChange(robot)
    _testLinLinBlendWithOriChange(robot)
    _testOriChangeInABC(robot)
    _testSeqWithJointPoses(robot)
    _testSeveralLinBlends(robot)
    _testEmptySequence(robot)

    _testTooLargeBlendRadius(robot)
    _testNegBlendRadius(robot)
    _testOverlappingRadi(robot)
    _testLastRadiNonZero(robot)

    _testUnreachableFirstPose(robot)

    _testBlendRadiusOnSecondCommand(robot)

# Tests a sequence which contains Lin motions commanded in joint space
# instead of cartesian space.
#
# Test Sequence:
#   1. Move to start position.
#   2. Execute blended lin motions.
#
#
# Expected Results:
#   1. Robot moves to start position.
#   2. Robot executes a trajectory which consists of two blended lin motions.
#       a. Robot reached goal pose of second lin.
#       b. Robot performed continuous motion (without stop).
#       c. Robot performed continuous orientation change (continuous rotational velocity).
#
def _testSeqWithJointPoses(robot):
    if (askPermission(_testSeqWithJointPoses.__name__) == 0):
        return
    robot.move(Ptp(goal=[0, 0.007, -1.816, 0, 1.8236, 0], vel_scale=DEFAULT_PTP_VEL))

    seq = Sequence()
    seq.append(Lin(goal=[0.52, 0.52, -1.1, 0, 1.56, 0], vel_scale=0.15), blend_radius=0.15)
    seq.append(Lin(goal=[0.52, 0.698, -1.74, 0.349, 1.56, 0.78], vel_scale=0.15), blend_radius=0)
    robot.move(seq)

    askSuccess(_testSeqWithJointPoses.__name__, 'Did the robot perform a lin-lin blending?.')

# Tries to blend lin motions. First lin motion contains unreachable position.
#
# Test Sequence:
#   1. Move to start position.
#   2. Execute blended lin motions.
#
#
# Expected Results:
#   1. Robot moves to start position.
#   2. Robot does NOT execute blended lin motions. An error message is
#       displayed.
#
def _testUnreachableFirstPose(robot):
    if (askPermission(_testUnreachableFirstPose.__name__) == 0):
        return
    # Move to initial position
    robot.move(Ptp(goal=[0, 0.007, -1.816, 0, 1.8236, 0], vel_scale=DEFAULT_PTP_VEL))

    seq1 = Sequence()
    seq1.append(Lin(goal=Pose(position=Point(0.25, 0.6, 0.65)), vel_scale=0.1, acc_scale=0.1), blend_radius=0.3)
    seq1.append(Lin(goal=Pose(position=Point(-0.1, 0.5, 0.65)),vel_scale=0.1, acc_scale=0.1), blend_radius=0)
    try:
        robot.move(seq1)
    except RobotMoveFailed:
        askSuccess(_testUnreachableFirstPose.__name__, 'There should be an error message stating that computation of inverse kinematic failed.')
        return
    print("Test failed!")

# Tries to blend lin motions with non-zero blend radius for the last lin motion.
#
# Test Sequence:
#   1. Move to start position (Please note: Movement consists of two ptps).
#   2. Execute blended lin motions.
#
#
# Expected Results:
#   1. Robot moves to start position.
#   2. Robot does NOT execute blended lin motions. An error message is
#       displayed.
#
def _testLastRadiNonZero(robot):
    if (askPermission(_testLastRadiNonZero.__name__) == 0):
        return

    # Move to start position (Left corner close to robot base)
    robot.move(Ptp(goal=[0, 0.007, -1.816, 0, 1.8236, 0], vel_scale=DEFAULT_PTP_VEL))
    robot.move(Ptp(goal=Pose(position=Point(0.15, 0.25, 0.65)), vel_scale=DEFAULT_PTP_VEL))

    seq = Sequence()
    _addRectangleLin(seq, r1=0.2, r2=0.12, r3=0.2, r4=0.01)

    try:
        robot.move(seq)
    except RobotMoveFailed:
        askSuccess(_testLastRadiNonZero.__name__, 'Robot should have performed no motion blending (only two PTP\'s).\nHowever, there should be an error stating that the last radi is not zero.')
        return
    print("Test failed!")
# Tries to blend lin motions using overlapping blend radii.
#
# Test Sequence:
#   1. Move to start position (Please note: Movement consists of two ptps).
#   2. Execute blended lin motions.
#
#
# Expected Results:
#   1. Robot moves to start position.
#   2. Robot does NOT execute blended lin motions. An error message is
#       displayed.
#
def _testOverlappingRadi(robot):
    if (askPermission(_testOverlappingRadi.__name__) == 0):
        return

    # Move to start position (Left corner close to robot base)
    robot.move(Ptp(goal=[0, 0.007, -1.816, 0, 1.8236, 0], vel_scale=DEFAULT_PTP_VEL))
    robot.move(Ptp(goal=Pose(position=Point(0.15, 0.25, 0.65)), vel_scale=DEFAULT_PTP_VEL))

    seq = Sequence()
    _addRectangleLin(seq, r1=0.2, r2=100, r3=0.2, r4=0)

    try:
        robot.move(seq)
    except RobotMoveFailed:
        askSuccess(_testOverlappingRadi.__name__, 'Robot should have performed no motion blending (only two PTP\'s).\nHowever, there should be an error stating that two blending radi are overlapping.')
        return
    print("Test failed!")

# Tests the blending of a "large" number of lin motions.
#
# Test Sequence:
#   1. Move to start position (Please note: Movement consists of two ptps).
#   2. Execute blended lin motions.
#
#
# Expected Results:
#   1. Robot moves to start position.
#   2. Robot performs a blending motion along a rectangle (without stop).
#       The robot moves along the rectangle for serveral rounds.
#
def _testSeveralLinBlends(robot):
    if (askPermission(_testSeveralLinBlends.__name__) == 0):
        return
    _rectangleLinLoop(robot)
    askSuccess(_testSeveralLinBlends.__name__, 'Robot should have moved in a continues motion along a rectangle (several times).')

# Tries to perform blend motions using an empty sequence.
#
# Test Sequence:
#   1. Execute motion using an empty sequence.
#
#
# Expected Results:
#   1. Robot does NOT execute any motion. A message is displayed indicating
#       the empty sequence.
#
def _testEmptySequence(robot):
    if (askPermission(_testEmptySequence.__name__) == 0):
        return
    _emptySequence(robot)
    askSuccess(_testEmptySequence.__name__, 'Robot should not move at all.')

# Tries to blend two lin motions using a negative blend radius.
#
# Test Sequence:
#   1. Move to start position.
#   2. Execute two blended lin motions.
#
#
# Expected Results:
#   1. Robot moves to start position.
#   2. Robot does NOT execute blended lin motions. An error message is
#       displayed.
#
def _testNegBlendRadius(robot):
    if (askPermission(_testNegBlendRadius.__name__) == 0):
        return
    try:
        _linLinBlend(robot, 0, r=-0.3)
    except RobotMoveFailed:
        askSuccess(_testNegBlendRadius.__name__, 'Robot should only execute ptp but no lin blending.\nThere should be an error stating that a negative blending radius was used.')
        return
    print("Test failed!")

# Tries to blend two lin motions using an out of scale blend radius.
#
# Test Sequence:
#   1. Move to start position.
#   2. Execute two blended lin motions.
#
#
# Expected Results:
#   1. Robot moves to start position.
#   2. Robot does NOT execute blended lin motions. An error message is
#       displayed.
#
def _testTooLargeBlendRadius(robot):
    if (askPermission(_testTooLargeBlendRadius.__name__) == 0):
        return
    try:
        _linLinBlend(robot, 0, r=100)
    except RobotMoveFailed:
        askSuccess(_testTooLargeBlendRadius.__name__, 'Robot should only execute ptp but no lin blending.\nThere should be an error stating that a too large blending radius was used.')
        return
    print("Test failed!")
# Tests the blending of two lin motions. Both lin goals have the same orientation.
#
# Test Sequence:
#   1. Move to start position.
#   2. Execute two blended lin motions.
#
#
# Expected Results:
#   1. Robot moves to start position.
#   2. Robot executes a trajectory which consists of two blended lin motions.
#       a. Robot reached goal pose of second lin.
#       b. Robot performed continuous motion (without stop).
#
def _testLinLinBlendWithoutOriChange(robot):
    if (askPermission(_testLinLinBlendWithoutOriChange.__name__) == 0):
        return
    _linLinBlend(robot, 0)
    askSuccess(_testLinLinBlendWithoutOriChange.__name__)

# Tests the blending of two lin motions.
# Both lin goals have different orientations (difference in euler angle C).
#
# Test Sequence:
#   1. Move to start position.
#   2. Execute two blended lin motions.
#
#
# Expected Results:
#   1. Robot moves to start position.
#   2. Robot executes a trajectory which consists of two blended lin motions.
#       a. Robot reached goal pose of second lin.
#       b. Robot performed continuous motion (without stop).
#       c. Robot performed continuous orientation change (continuous rotational velocity).
#
def _testLinLinBlendWithOriChange(robot):
    if (askPermission(_testLinLinBlendWithOriChange.__name__) == 0):
        return
    _linLinBlend(robot, 1.57)
    askSuccess(_testLinLinBlendWithOriChange.__name__)

# Tests the blending of three motions where only the second has a radius.
# 
# Test Sequence:
#   1. Move to start position.
#   2. Execute two blended motions.
#
#
# Expected Results:
#   1. Robot moves to start position.
#   2. Robot executes a trajectory which consists of two blended motions.
#       a. Robot reached goal pose of third trajectory.
#       b. Robot performed continuous motion (without stop).
#       c. Robot performed continuous orientation change (continuous rotational velocity).
#       d. Robot makes smooth curve between into last trajectory.
#
def _testBlendRadiusOnSecondCommand(robot):
    if (askPermission(_testBlendRadiusOnSecondCommand.__name__) == 0):
        return

    robot.move(Ptp(goal=Pose(position=Point(0.0, 0.0, 0.9), orientation=from_euler(0,0,0)), 
                   vel_scale=DEFAULT_PTP_VEL))
    seq1 = Sequence()
    seq1.append(Ptp(goal=Pose(position=Point(0.2, 0.0, 0.9), orientation=from_euler(0,0,0)), 
                    vel_scale=DEFAULT_PTP_VEL, acc_scale=DEFAULT_PTP_ACC))
    seq1.append(Ptp(goal=Pose(position=Point(0.2, 0.2, 0.9), orientation=from_euler(0,0,0)), 
                    vel_scale=DEFAULT_PTP_VEL, acc_scale=DEFAULT_PTP_ACC), blend_radius=.1)
    seq1.append(Ptp(goal=Pose(position=Point(0.0, 0.2, 0.9), orientation=from_euler(0,0,0)), 
                    vel_scale=DEFAULT_PTP_VEL, acc_scale=DEFAULT_PTP_ACC))
    robot.move(seq1)

    askSuccess(_testBlendRadiusOnSecondCommand.__name__)

# Tests the blending of two lin motions.
# Both lin goals have different orientations (difference in euler angle A, B
# and C).
#
# Test Sequence:
#   1. Move to start position.
#   2. Execute two blended lin motions.
#
#
# Expected Results:
#   1. Robot moves to start position.
#   2. Robot executes a trajectory which consists of two blended lin motions.
#       a. Robot reached goal pose of second lin.
#       b. Robot performed continuous motion (without stop).
#       c. Robot performed continuous orientation change (continuous rotational velocity).
#
def _testOriChangeInABC(robot):
    if (askPermission(_testOriChangeInABC.__name__) == 0):
        return
    robot.move(Ptp(goal=[0, 0.007, -1.816, 0, 1.8236, 0], vel_scale=DEFAULT_PTP_VEL))
    robot.move(Ptp(goal=Pose(position=Point(0.15, 0.25, 0.75), orientation=from_euler(0,0,0)), vel_scale=DEFAULT_PTP_VEL))

    seq = Sequence()
    seq.append(Lin(goal=Pose(position=Point(0.5, 0.25, 0.4), orientation=from_euler(1.2,0.8,-3)), vel_scale=0.25, acc_scale=0.1), blend_radius=0.3)
    seq.append(Lin(goal=Pose(position=Point(0.5, 0, 0.75), orientation=from_euler(0,0,0)), vel_scale=0.25, acc_scale=0.1), blend_radius=0)
    robot.move(seq)

    askSuccess(_testOriChangeInABC.__name__, 'Did the robot perform a lin-lin blending motion with orientation change in A,B,C?')

# Performs a motion using an empty sequence.
def _emptySequence(robot):
    seq1 = Sequence()
    robot.move(seq1)

# Moves to a start position and, consequently, performs a blending motion
# along a rectangle. The robot moves along the rectangle for serveral rounds.
def _rectangleLinLoop(robot):
    # Move to start position (Left corner close to robot base)
    robot.move(Ptp(goal=[0, 0.007, -1.816, 0, 1.8236, 0], vel_scale=DEFAULT_PTP_VEL))
    robot.move(Ptp(goal=Pose(position=Point(0.15, 0.25, 0.68)), vel_scale=DEFAULT_PTP_VEL))

    seq = Sequence()
    for n in range(1, 3):
        _addRectangleLin(seq, r1=0.2, r2=0.12, r3=0.2, r4=0.12)
        _addRectangleLin(seq, r1=0.2, r2=0.12, r3=0.2, r4=0.12)

    _addRectangleLin(seq, r1=0.2, r2=0.12, r3=0.2, r4=0)
    robot.move(seq)

# Adds a rectangle blending motion to the given sequence using the given
# blending radi.
def _addRectangleLin(seq, r1, r2, r3, r4):
    #Left corner away from robot base
    seq.append(Lin(goal=Pose(position=Point(0.5, 0.25, 0.68)), vel_scale=0.1, acc_scale=0.07), blend_radius=r1)
    #Right corner away from robot base
    seq.append(Lin(goal=Pose(position=Point(0.5, -0.25, 0.68)), vel_scale=0.1, acc_scale=0.07), blend_radius=r2)
    #Right corner close to robot base
    seq.append(Lin(goal=Pose(position=Point(0.15, -0.25, 0.68)), vel_scale=0.1, acc_scale=0.07), blend_radius=r3)
    #Left corner close to robot base
    seq.append(Lin(goal=Pose(position=Point(0.15, 0.25, 0.68)), vel_scale=0.1, acc_scale=0.07), blend_radius=r4)

# Blends two Lin's with the given orientation change and blending radius.
def _linLinBlend(robot, z_ori_change, r=0.3):
    # Move to initial position
    robot.move(Ptp(goal=[0, 0.007, -1.816, 0, 1.8236, 0], vel_scale=DEFAULT_PTP_VEL))

    seq1 = Sequence()
    seq1.append(Lin(goal=Pose(position=Point(0.25, 0.5, 0.65), orientation=from_euler(0,0,0)), vel_scale=0.1, acc_scale=0.1), blend_radius=r)
    seq1.append(Lin(goal=Pose(position=Point(-0.1, 0.5, 0.65), orientation=from_euler(0,0,z_ori_change)),vel_scale=0.1, acc_scale=0.1), blend_radius=0)

    robot.move(seq1)

if __name__ == "__main__":
    # Init a rosnode
    rospy.init_node('blend_acceptance_tests_node', anonymous=True)

    start_program()

