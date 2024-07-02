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


class MoveGroupCommanderMock:
    """Mock for testing."""
    def __init__(self, num_of_joints):
        rospy.loginfo("Ctor of MoveGroupCommanderMock called")
        self._num_of_joints = num_of_joints

    def get_end_effector_link(self):
        rospy.loginfo("get_end_effector_link called")
        return ["prbt_tcp"]

    def get_active_joints(self):
        rospy.loginfo("get_active_joints called")
        joint_array = []
        for i in range(self._num_of_joints):
            joint_array.append("joint" + str(i))
        rospy.loginfo(joint_array)
        return joint_array

    def get_current_joint_values(self):
        rospy.loginfo("get_current_joint_values called")
        joint_array = []
        for i in range(self._num_of_joints):
            joint_array.append(0.0)
        rospy.loginfo(joint_array)
        return joint_array


class RobotCommanderMock:
    """Mock for testing."""
    def __init__(self):
        rospy.loginfo("Ctor of RobotCommanderMock called")

    def get_group(self, group_name):
        rospy.loginfo("get_group called")
        if group_name == "manipulator":
            return MoveGroupCommanderMock(6)
        else:
            return MoveGroupCommanderMock(1)

    def get_planning_frame(self):
        rospy.loginfo("get_planning_frame() called")
        return "dummy_str"
