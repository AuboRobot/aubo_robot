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
from pathlib import Path
from geometry_msgs.msg import PoseStamped, Pose

_TEST_DATA_FILE_NAME = Path(__file__).parent.parent.absolute() / Path("test_data/test_data.xml")
PLANNING_GROUP_NAME = "manipulator"
API_VERSION = "1"
COMPARE_PRECISION = 6


class TestAPIUtilityFunctions(unittest.TestCase):
    """
    Test utility functions in the python api.
    """

    def setUp(self):
        self.robot = Robot(API_VERSION)
        self.test_data = XmlTestdataLoader(_TEST_DATA_FILE_NAME)

    def tearDown(self):
        if hasattr(self, 'robot'):
            self.robot._release()
            self.robot = None
        if hasattr(self, 'test_data'):
            del self.test_data

    def comparePose(self, pose_1, pose_2):
        self.assertAlmostEqual(pose_1.position.x, pose_2.position.x, COMPARE_PRECISION)
        self.assertAlmostEqual(pose_1.position.y, pose_2.position.y, COMPARE_PRECISION)
        self.assertAlmostEqual(pose_1.position.z, pose_2.position.z, COMPARE_PRECISION)
        self.assertAlmostEqual(pose_1.orientation.x, pose_2.orientation.x, COMPARE_PRECISION)
        self.assertAlmostEqual(pose_1.orientation.y, pose_2.orientation.y, COMPARE_PRECISION)
        self.assertAlmostEqual(pose_1.orientation.z, pose_2.orientation.z, COMPARE_PRECISION)
        self.assertAlmostEqual(pose_1.orientation.w, pose_2.orientation.w, COMPARE_PRECISION)

    def test_get_current_pose(self):
        """ Check if the current pose can be retrieved correctly

            Test sequence:
                1. Move robot to a goal pose via ptp.
                2. Check the current pose by robot.get_current_pose().

            Test Results:
                1. Move function returns without throwing an exception.
                2. The current pose is same as the goal pose.
        """
        # 1
        # move from defined joint pose
        self.robot.move(Ptp(goal=self.test_data.get_joints("PTPJointValid", PLANNING_GROUP_NAME)))
        # move to the goal pose
        goal_pose = self.test_data.get_pose("CartValid", PLANNING_GROUP_NAME)
        self.robot.move(Ptp(goal=goal_pose))

        # 2
        current_pose = self.robot.get_current_pose()
        self.comparePose(goal_pose, current_pose)

    def test_get_current_pose_for_invalid_tf(self):
        """ Check if trying to get the current pose for a nonexistent frame raises an exception
        """
        self.assertRaises(RobotCurrentStateError, self.robot.get_current_pose, target_link="invalid")

    def test_get_current_pose_return_type(self):
        current_pose = self.robot.get_current_pose()
        self.assertTrue(isinstance(current_pose, Pose))

    def test_get_current_pose_stamped_return_type(self):
        current_pose = self.robot.get_current_pose_stamped()
        self.assertTrue(isinstance(current_pose, PoseStamped))

    def test_get_current_joints(self):
        """ Check if the current joints can be retrieved correctly

            Test sequence:
                1. Move robot to a goal position via ptp.
                2. Check the current pose by robot.get_current_joints().

            Test Results:
                1. Move function returns without throwing an exception.
                2. The current joints are same as the goal joints.
        """
        # 1
        # move out of zero pose
        goal_joints = self.test_data.get_joints("RelJoint", PLANNING_GROUP_NAME)
        self.robot.move(Ptp(goal=goal_joints))

        # 2
        current_joints = self.robot.get_current_joint_states()
        self.assertEqual(len(goal_joints), len(current_joints))
        for current_joint, goal_joint in zip(current_joints, goal_joints):
            self.assertAlmostEqual(current_joint, goal_joint, COMPARE_PRECISION)

    def test_get_joints_for_invalid_group(self):
        """ Check that trying to get the current joint states raises an exception.
        """
        self.assertRaises(RobotCurrentStateError, self.robot.get_current_joint_states, planning_group="invalid")


if __name__ == '__main__':
    import rostest
    rospy.init_node('test_api_utility_functions')
    rostest.rosrun('pilz_robot_programming', 'test_api_utility_functions', TestAPIUtilityFunctions)
