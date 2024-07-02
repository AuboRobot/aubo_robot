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
SEQUENCE_TOPIC = "sequence_move_group"


class TestAPIExecutionStop(unittest.TestCase):
    """
    Test the Api Adapter Node. Result is evaluated from the error code in action result


    Strategy for error cases that are caught inside CommandPlanner: We only test for one
    error case, that the returned error is passed on to the user.
    Not every single error case is covered by a test here; referring to the tests on lower
    levels instead.
    """

    _SLEEP_TIME_S = 0.01
    _TOLERANCE_FOR_MOTION_DETECTION_RAD = 0.01

    def setUp(self):
        rospy.loginfo("SetUp called...")
        self.test_data = XmlTestdataLoader(_TEST_DATA_FILE_NAME)
        self.robot = Robot(API_VERSION)
        self.robot_motion_observer = RobotMotionObserver(PLANNING_GROUP_NAME)

    def tearDown(self):
        rospy.loginfo("TearDown called...")
        if hasattr(self, 'robot'):
            self.robot._release()
            self.robot = None

        if hasattr(self, 'test_data'):
            del self.test_data

    def getCmds(self, name, cmd_type='ptp', start_type='joint', goal_type='joint', circ_type='center'):

        if cmd_type == 'ptp' and goal_type == 'joint':
            return Ptp()

        if cmd_type == 'lin':
            lin_cmd = self.test_data.get_lin(name)
            # start cmd
            if start_type == 'joint':
                start_ptp = Ptp(goal=self.test_data.get_joints(lin_cmd[START_POS_STR], lin_cmd[PLANNING_GROUP_STR]))
            else:
                start_ptp = Ptp(goal=self.test_data.get_pose(lin_cmd[START_POS_STR], lin_cmd[PLANNING_GROUP_STR]))

            if goal_type == 'joint':
                lin = Lin(goal=self.test_data.get_joints(lin_cmd[END_POS_STR], lin_cmd[PLANNING_GROUP_STR]),
                          vel_scale=lin_cmd[VEL_STR], acc_scale=lin_cmd[ACC_STR])
            else:
                lin = Lin(goal=self.test_data.get_pose(lin_cmd[END_POS_STR], lin_cmd[PLANNING_GROUP_STR]),
                          vel_scale=lin_cmd[VEL_STR], acc_scale=lin_cmd[ACC_STR])
            return start_ptp, lin

        if cmd_type == "circ":
            # circ cmd
            if circ_type == 'interim':
                circ_cmd = self.test_data.get_circ(name, INTERIM_POS_STR)
                circ = Circ(goal=self.test_data.get_pose(circ_cmd[END_POS_STR], circ_cmd[PLANNING_GROUP_STR]),
                            interim=self.test_data.get_pose(circ_cmd[INTERIM_POS_STR],
                                                            circ_cmd[PLANNING_GROUP_STR]).position,
                            vel_scale=circ_cmd[VEL_STR], acc_scale=circ_cmd[ACC_STR])
            else:
                circ_cmd = self.test_data.get_circ(name, CENTER_POS_STR)
                circ = Circ(goal=self.test_data.get_pose(circ_cmd[END_POS_STR], circ_cmd[PLANNING_GROUP_STR]),
                            center=self.test_data.get_pose(circ_cmd[CENTER_POS_STR],
                                                           circ_cmd[PLANNING_GROUP_STR]).position,
                            vel_scale=circ_cmd[VEL_STR], acc_scale=circ_cmd[ACC_STR])

            # start cmd
            if start_type == 'joint':
                start_ptp = Ptp(goal=self.test_data.get_joints(circ_cmd[START_POS_STR], circ_cmd[PLANNING_GROUP_STR]))
            else:
                start_ptp = Ptp(goal=self.test_data.get_pose(circ_cmd[START_POS_STR], circ_cmd[PLANNING_GROUP_STR]))

            return start_ptp, circ

        if cmd_type == "sequence":
            sequence_cmd = self.test_data.get_sequence(name)
            seq = Sequence()

            # first cmd
            first_cmd = sequence_cmd.pop(0)
            start_ptp, lin = self.getCmds(name=first_cmd[NAME_STR], cmd_type=first_cmd[TYPE_STR], start_type="joint",
                                          goal_type="pose")
            seq.append(lin, float(first_cmd[BLEND_RADIUS_STR]))

            for cmd in sequence_cmd:
                start_, lin_ = self.getCmds(name=cmd[NAME_STR], cmd_type=cmd[TYPE_STR], start_type="joint",
                                            goal_type="pose")
                seq.append(lin_, float(cmd[BLEND_RADIUS_STR]))

            return start_ptp, seq

    def stop_during_execution(self, cmd_for_testing):
        """ Tests what happens if stop is triggered during execution of given command.

            Test sequence:
                1. Start given command in separate thread.
                2. Wait till robot is moving then execute stop function in main test thread.

            Test Results:
                1. Robot starts moving.
                2. Execute methods ends and throws correct exception.
        """
        # +++++++++++++++++++++++
        rospy.loginfo("Step 1")
        # +++++++++++++++++++++++
        move_thread = MoveThread(self.robot, cmd_for_testing, RobotMoveFailed)
        move_thread.start()

        # +++++++++++++++++++++++
        rospy.loginfo("Step 2")
        # +++++++++++++++++++++++

        # Wait till movement started
        self.assertTrue(self.robot_motion_observer.wait_motion_start(
            move_tolerance=self._TOLERANCE_FOR_MOTION_DETECTION_RAD, sleep_interval=self._SLEEP_TIME_S))

        rospy.loginfo("Call stop function.")
        self.robot.stop()

        rospy.loginfo("Wait till thread/command finished to evaluate result...")
        move_thread.join()

        # Check return value of execute method
        self.assertTrue(move_thread.exception_thrown)

    def test_none_robot_cmd(self):
        """ Check what happens if somebody tries to execute a none move command.

            Test sequence:
                1. Create none move command.
                2. Call the execute function directly

            Test Results:
                1. Move function throws exception.
                2. Error is returned.
        """
        self.assertRaises(RobotUnknownCommandType, self.robot.move, None)

        ptp = Ptp()
        self.assertEquals(ptp._execute(self.robot), Robot._FAILURE)

    def test_ptp_execution(self):
        """ Test execution of valid ptp command works successfully.

            Test sequence:
                1. Execute a valid ptp command

            Test results:
                1. Move function returns without throwing an exception.
        """
        self.robot.move(Ptp(goal=self.test_data.get_joints("PTPJointValid", PLANNING_GROUP_NAME)))

    def test_invalid_ptp_execution(self):
        """ Test execution of invalid ptp command works successfully.

            Test sequence:
                1. Execute an invalid ptp command with out of bounds joints

            Test results:
                1. Move function throws exception.
        """
        self.assertRaises(RobotMoveFailed, self.robot.move,
                          Ptp(goal=self.test_data.get_joints("PTPJointInvalid", PLANNING_GROUP_NAME)))

    def test_lin_execution(self):
        """ Test execution of valid lin command works successfully.

            Test sequence:
                1. Move the robot to start pose via ptp
                2. Execute a valid lin command

            Test results:
                1. Move function returns without throwing an exception.
                2. Move function returns without throwing an exception.
        """
        start_ptp, lin = self.getCmds(name="ValidLINCmd", cmd_type="lin", start_type="joint", goal_type="pose")

        self.robot.move(start_ptp)

        self.robot.move(lin)

    def test_invalid_lin_execution(self):
        """ Test execution of invalid lin command will fail.

            Test sequence:
                1. Move the robot to start pose via ptp
                2. Execute an invalid lin command which has an unreachable goal pose

            Test results:
                1. Move function returns without throwing an exception.
                2. Move function throws exception.
        """
        start_ptp, lin = self.getCmds(name="InvalidLINCmd", cmd_type="lin", start_type="joint", goal_type="pose")

        self.robot.move(start_ptp)

        self.assertRaises(RobotMoveFailed, self.robot.move, lin)

    def test_circ_center_execution(self):
        """ Test execution of valid circ center command works successfully.

            Test sequence:
                1. Move the robot out of zero pose via ptp joint
                2. Move the robot to start pose via ptp
                3. Execute a valid circ center command

            Test results:
                1. Move function returns without throwing an exception.
                2. Move function returns without throwing an exception.
                3. Move function returns without throwing an exception.
        """
        self.robot.move(Ptp(goal=self.test_data.get_joints("PTPJointValid", PLANNING_GROUP_NAME)))

        start_ptp, circ = self.getCmds("ValidCircCmd", cmd_type="circ", start_type="pose", circ_type="center")

        self.robot.move(start_ptp)

        self.robot.move(circ)

    def test_invalid_circ_center_execution(self):
        """ Test execution of invalid circ center command will fail.

            Test sequence:
                1. Move the robot out of zero pose via ptp joint
                2. Move the robot to start pose via ptp
                3. Execute an invalid circ center command

            Test results:
                1. Move function returns without throwing an exception.
                2. Move function returns without throwing an exception.
                3. Move function throws exception.
        """
        self.robot.move(Ptp(goal=self.test_data.get_joints("PTPJointValid", PLANNING_GROUP_NAME)))

        start_ptp, circ = self.getCmds("InvalidCircCmd", cmd_type="circ", start_type="pose", circ_type="center")

        self.robot.move(start_ptp)

        self.assertRaises(RobotMoveFailed, self.robot.move, circ)

    def test_circ_interim_execution(self):
        """ Test execution of valid circ interim command works successfully.

            Test sequence:
                1. Move the robot out of zero pose via ptp joint
                2. Move the robot to start pose via ptp
                3. Execute a valid circ interim command

            Test results:
                1. Move function returns without throwing an exception.
                2. Move function returns without throwing an exception.
                3. Move function returns without throwing an exception.
        """
        self.robot.move(Ptp(goal=self.test_data.get_joints("PTPJointValid", PLANNING_GROUP_NAME)))

        start_ptp, circ = self.getCmds("ValidCircCmd", cmd_type="circ", start_type="pose", circ_type="interim")

        self.robot.move(start_ptp)

        self.robot.move(circ)

    def test_lin_lin_blend_execution(self):
        """ Test execution of valid sequence command, consisting of lin-lin, works successfully.

            Test sequence:
                1. Move the robot to start pose via ptp
                2. Execute a valid circ interim command

            Test results:
                1. Move function returns without throwing an exception.
                2. Move function returns without throwing an exception.
        """
        rospy.loginfo("\n\nStart of test...\n")
        start_ptp, sequence = self.getCmds("LINLINBlend", cmd_type="sequence")

        self.robot.move(start_ptp)

        self.robot.move(sequence)

    def test_invalid_lin_lin_blend_execution(self):
        """ Test execution of an invalid sequence command with too large velocity scale fails.

            Test sequence:
                1. Move the robot to start pose via ptp
                2. Execute an invalid sequence command with too large velocity scale

            Test results:
                1. Move function returns without throwing an exception.
                2. Move function throws exception.
        """
        rospy.loginfo("\n\nStart of test...\n")
        start_ptp, sequence = self.getCmds("InvalidLINLINBlend", cmd_type="sequence")

        self.robot.move(start_ptp)

        self.assertRaises(RobotMoveFailed, self.robot.move, sequence)

    def test_execute_abstract_cmd(self):
        """ An AbstractCmd (base class) object should not be executable

            Test sequence:
                1. Execute an invalid AbstractCmd command

            Test results:
                1. Move function throws exception.
        """
        import pilz_robot_programming.commands as cmd
        abstract_cmd = cmd._AbstractCmd()  # private abstract base

        self.assertRaises(RobotMoveFailed, self.robot.move, abstract_cmd)

    def test_ptp_stop_during_command(self):
        """Test stop during ptp command execution.

        For more details see function stop_during_execution.

        """
        # move to start pose
        self.robot.move(Ptp(goal=self.test_data.get_joints("ZeroPose", PLANNING_GROUP_NAME)))

        cmd_for_testing = Ptp(goal=self.test_data.get_joints("PTPJointValid", PLANNING_GROUP_NAME))
        self.stop_during_execution(cmd_for_testing)

    def test_lin_stop_during_command(self):
        """Test stop during lin command execution.

        For more details see function stop_during_execution.
        """

        start_ptp, lin = self.getCmds(name="ValidLINCmd", cmd_type="lin", start_type="joint", goal_type="pose")

        self.robot.move(start_ptp)

        self.stop_during_execution(lin)

    def test_circ_stop_during_command(self):
        """Test stop during circ command execution.

        For more details see function stop_during_execution.
        """
        self.robot.move(Ptp(goal=self.test_data.get_joints("PTPJointValid", PLANNING_GROUP_NAME)))

        start_ptp, circ = self.getCmds("ValidCircCmd", cmd_type="circ", start_type="pose", circ_type="center")

        self.robot.move(start_ptp)

        self.stop_during_execution(circ)

    def test_sequence_stop_during_command(self):
        """Test stop during sequence command execution.

        For more details see function stop_during_execution.
        """
        start_ptp, sequence = self.getCmds("LINLINBlend", cmd_type="sequence")

        self.robot.move(start_ptp)

        self.stop_during_execution(sequence)

    def test_move_external_stop(self):
        """ Tests what happens if stop is triggered during execution of given command.

            Test sequence:
                1. Start given command in separate thread.
                2. Wait till robot is moving then execute client.cancel_all_goals() in main test thread.

            Test Results:
                1. Robot starts moving.
                2. Execute methods ends and throws correct exception.
        """
        # Create move_group action client for external stop
        move_client = SimpleActionClient(SEQUENCE_TOPIC, MoveGroupSequenceAction)
        rospy.loginfo("Waiting for connection to action server " + SEQUENCE_TOPIC + "...")
        move_client.wait_for_server()
        rospy.logdebug("Connection to action server " + SEQUENCE_TOPIC + " established.")

        # Move to zero pose
        self.robot.move(Ptp(goal=self.test_data.get_joints("ZeroPose", PLANNING_GROUP_NAME)))

        # +++++++++++++++++++++++
        rospy.loginfo("Step 1")
        # +++++++++++++++++++++++
        ptp = Ptp(goal=self.test_data.get_joints("PTPJointValid", PLANNING_GROUP_NAME), vel_scale=0.05)
        move_thread = MoveThread(self.robot, ptp, RobotMoveFailed)
        move_thread.start()

        # +++++++++++++++++++++++
        rospy.loginfo("Step 2")
        # +++++++++++++++++++++++

        # Wait till movement started
        self.assertTrue(self.robot_motion_observer.wait_motion_start(
            move_tolerance=self._TOLERANCE_FOR_MOTION_DETECTION_RAD, sleep_interval=self._SLEEP_TIME_S))

        rospy.loginfo("Call stop function from external client.")
        move_client.cancel_all_goals()

        rospy.loginfo("Wait till thread/command finished to evaluate result...")
        move_thread.join()

        # Check return value of execute method
        self.assertTrue(move_thread.exception_thrown)

    def test_move_multi_threading_prevention(self):
        """ Tests what happens if two move() are started in two threads.

            Test sequence:
                1. Start a command in separate thread.
                2. Start another command in main test thread.

            Test Results:
                1. Robot starts moving.
                2. Second move throws RobotMoveAlreadyRunningError. First move finishes successfully.
        """
        # Move to zero pose
        self.robot.move(Ptp(goal=self.test_data.get_joints("ZeroPose", PLANNING_GROUP_NAME)))

        # +++++++++++++++++++++++
        rospy.loginfo("Step 1")
        # +++++++++++++++++++++++
        ptp = Ptp(goal=self.test_data.get_joints("PTPJointValid", PLANNING_GROUP_NAME), vel_scale=0.1)
        move_thread = MoveThread(self.robot, ptp, RobotMoveFailed)
        move_thread.start()

        # +++++++++++++++++++++++
        rospy.loginfo("Step 2")
        # +++++++++++++++++++++++

        # Wait till movement started
        self.assertTrue(self.robot_motion_observer.wait_motion_start(
            move_tolerance=self._TOLERANCE_FOR_MOTION_DETECTION_RAD, sleep_interval=self._SLEEP_TIME_S))

        # start another command
        try:
            self.robot.move(Ptp(goal=self.test_data.get_joints("ZeroPose", PLANNING_GROUP_NAME)))
        except RobotMoveAlreadyRunningError:
            pass
        else:
            self.fail('robot.move() can run in two threads.')

        rospy.loginfo("Wait till thread/command finished to evaluate result...")
        move_thread.join()

        # Check return value of execute method
        self.assertFalse(move_thread.exception_thrown)


if __name__ == '__main__':
    import rostest
    rospy.init_node('test_api_execution_stop')
    rostest.rosrun('pilz_robot_programming', 'test_api_execution_stop', TestAPIExecutionStop)
