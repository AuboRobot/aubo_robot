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
import rospy

__REQUIRED_API_VERSION__ = "1"
_DEFAULT_VEL_SCALE = 0.1


def start_program():
    print("Executing " + __file__)

    r = Robot(__REQUIRED_API_VERSION__)

    r.move(Ptp(goal=Pose(position=Point(-0.460, -0.21, 0.19),
                         orientation=from_euler(0, -3.14, -0.25)), vel_scale=_DEFAULT_VEL_SCALE))

    for i in range(0, 10):
        r.move(Lin(goal=Pose(position=Point(-0.460, 0, 0.19),
                             orientation=from_euler(0, -3.14, -0.25)), vel_scale=_DEFAULT_VEL_SCALE))

        r.move(Lin(goal=Pose(position=Point(-0.3, 0, 0.19),
                             orientation=from_euler(0, -3.14, -0.25)), vel_scale=_DEFAULT_VEL_SCALE))

        r.move(Lin(goal=Pose(position=Point(-0.3, -0.21, 0.19),
                             orientation=from_euler(0, -3.14, -0.25)), vel_scale=_DEFAULT_VEL_SCALE))

        r.move(Lin(goal=Pose(position=Point(-0.46, -0.21, 0.19),
                             orientation=from_euler(0, -3.14, -0.25)), vel_scale=_DEFAULT_VEL_SCALE))


if __name__ == "__main__":
    # Init a ros node
    rospy.init_node('robot_program_node')

    start_program()

