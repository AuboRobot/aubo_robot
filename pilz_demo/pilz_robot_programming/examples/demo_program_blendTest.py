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
from pilz_robot_programming import *
import math
import rospy

__REQUIRED_API_VERSION__ = "1"


def start_program():
    print("Executing " + __file__)

    r = Robot(__REQUIRED_API_VERSION__)

    # Simple ptp movement
#    r.move(Ptp(goal=[0, 0.5, 0.5, 0, 0, 0], vel_scale=0.4))

    start_joint_values = r.get_current_joint_states()

    # Relative ptp movement
#    r.move(Ptp(goal=[0.1, 0, 0, 0, 0, 0], relative=True, vel_scale=0.2))
#    r.move(Ptp(goal=Pose(position=Point(0, 0, -0.1)), relative=True))
#    r.move(Ptp(goal=[-0.2, 0, 0, 0, 0, 0], relative=True, acc_scale=0.2))

#    pose_after_relative = r.get_current_pose()

    # Simple Lin movement
#    r.move(Lin(goal=Pose(position=Point(0.2, 0, 0.8)), vel_scale=0.1, acc_scale=0.1))

    # Relative Lin movement
#    r.move(Lin(goal=Pose(position=Point(0, -0.2, 0), orientation=from_euler(0, 0, math.radians(15))), relative=True,
#           vel_scale=0.1, acc_scale=0.1))
 #   r.move(Lin(goal=Pose(position=Point(0, 0.2, 0)), reference_frame="base_link",relative=True,
 #          vel_scale=0.1, acc_scale=0.1))
 #   r.move(Lin(goal=Pose(position=Point(0.2, 0, 0)), reference_frame="base_link",relative=True,
 #          vel_scale=0.1, acc_scale=0.1))
 #   r.move(Lin(goal=Pose(position=Point(0, -0.2, 0)), reference_frame="base_link",relative=True,
 #          vel_scale=0.1, acc_scale=0.1))           
 #   r.move(Lin(goal=Pose(position=Point(-0.2, 0, 0)), reference_frame="base_link",relative=True,
 #          vel_scale=0.1, acc_scale=0.1))           
                      

    # Circ movement
#    r.move(Circ(goal=Pose(position=Point(0.2, -0.2, 0.8)), center=Point(0.1, -0.1, 0.8), reference_frame="base_link",acc_scale=0.1))

    # Move robot with stored pose
#   r.move(Ptp(goal=pose_after_relative, vel_scale=0.2))

    # Repeat the previous steps with a sequence command
    sequence = Sequence()
    sequence.append(Ptp(goal=start_joint_values))
    sequence.append(Lin(goal=Pose(position=Point(0.2, 0.1, 0)), reference_frame="base_link",relative=True,
           vel_scale=0.1, acc_scale=0.1), blend_radius=0.05)           
    sequence.append(Lin(goal=Pose(position=Point(0, 0.2, 0)), reference_frame="base_link",relative=True,
           vel_scale=0.1, acc_scale=0.1), blend_radius=0.05)
    sequence.append(Lin(goal=Pose(position=Point(0.2, 0.3, 0)), reference_frame="base_link",relative=True,
           vel_scale=0.1, acc_scale=0.1), blend_radius=0.05)
    sequence.append(Lin(goal=Pose(position=Point(0, 0.4, 0)), reference_frame="base_link",relative=True,
           vel_scale=0.1, acc_scale=0.1), blend_radius=0)
    r.move(sequence)

    # Move to start goal for sequence demonstration
    r.move(Ptp(goal=start_joint_values))

    # Blend sequence
#    blend_sequence = Sequence()
#    blend_sequence.append(Ptp(goal=Pose(position=Point(-0.48, 0.1, 0.95)),vel_scale=0.2))
#    blend_sequence.append(Lin(goal=Pose(position=Point(-0.48, 0.1, 0.95)), acc_scale=0.05), blend_radius=0.2)
#    blend_sequence.append(Lin(goal=Pose(position=Point(-0.48, 0.1, 0.95)), acc_scale=0.05))

#    r.move(blend_sequence)

    # Move with custom reference frame
#    r.move(Ptp(goal=PoseStamped(header=Header(frame_id="wrist3_Link"),
#                                pose=Pose(position=Point(0, 0, 0.1)))))
#    r.move(Ptp(goal=Pose(position=Point(0, -0.1, 0)), reference_frame="base_link", relative=True))

    # Create and execute an invalid ptp command with out of bound joint values
    try:
        r.move(Ptp(goal=start_joint_values))
    except RobotMoveFailed:
        rospy.loginfo("Ptp command did fail as expected.")


if __name__ == "__main__":
    # Init a ros node
    rospy.init_node('robot_program_node')

    start_program()

