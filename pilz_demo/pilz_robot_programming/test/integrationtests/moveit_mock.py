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

from actionlib import SimpleActionServer

from moveit_msgs.msg import MoveGroupAction, MoveGroupSequenceAction

from std_srvs.srv import Empty


class MoveitMock:
    """Mock for MoveIt which allows us to simulate errors during command execution."""

    def __init__(self):
        rospy.logdebug("Ctor of MoveitMock called.")

        self._as_move_group = SimpleActionServer('move_group', MoveGroupAction,
                                                 execute_cb=self.move_group_execute_callback, auto_start=False)

        self._as_sequence_group = SimpleActionServer('sequence_move_group', MoveGroupSequenceAction,
                                                     execute_cb=self.blend_move_group_execute_callback,
                                                     auto_start=False)

        self._as_move_group.start()
        self._as_sequence_group.start()

        # Service to signal test that node is running.
        self.status_service = rospy.Service('moveit_mock_status', Empty, self.handle_status_service)

        rospy.loginfo('MoveitMock started')

    def handle_status_service(self, req):
        pass

    def move_group_execute_callback(self, goal):
        rospy.loginfo("MoveGroup execute called.")
        rospy.sleep(1)
        self._as_move_group.set_aborted()
        return

    def blend_move_group_execute_callback(self, goal):
        rospy.loginfo("BlendMoveGroup execute called.")
        rospy.sleep(1)
        self._as_sequence_group.set_aborted()
        return


if __name__ == '__main__':
    rospy.init_node('motion_command_action_server')
    rospy.loginfo('Node initialized')

    server = MoveitMock()
    rospy.spin()
