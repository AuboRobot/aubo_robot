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

"""=MoveCommandLineTool=
Perform single movement commands via script

Usage:
  movecmd.py [options] ptp joint <j1> <j2> <j3> <j4> <j5> <j6>
  movecmd.py [options] ptp pose <x> <y> <z> <a> <b> <c>
  movecmd.py [options] lin joint <j1> <j2> <j3> <j4> <j5> <j6>
  movecmd.py [options] lin pose <x> <y> <z> <a> <b> <c>
  movecmd.py [options] circ joint <j1> <j2> <j3> <j4> <j5> <j6> (center|interim)  <px> <py> <pz>
  movecmd.py [options] circ pose <x> <y> <z> <a> <b> <c> (center|interim) <px> <py> <pz>
  movecmd.py grip <fw>
  movecmd.py (-h | --help)
  movecmd.py --version

Note:
  Three motion commands are available: ptp, lin and circ. They should be
  followed by a goal subcommand: joint or pose.
  The parameters <j1> to <j6> represent the six joint positions of the robot
  manipulator in rad.

  The parameters <x> <y> <z> represent the tcp position (in m) with respect to the
  base frame, <a> <b> <c> denote the euler angle in rad.

  The circ command requires an additional subcommand: center or interim, to specify the position of the center point
  or an interim point of the circle.

  All options are placed before commands.

Options:
  -s --vel_scale=<vel_scale>             Scale the maximal velocity/speed
  -a --acc_scale=<acc_scale>             Scale the maximal acceleration
  --rel                                  Move relative to current position.
  --ref=<target_frame>                   Move relative to a custom frame
  -h --help                              Show this screen.
  --version                              Show version.

"""

import rospy
import geometry_msgs.msg
from docopt import docopt
from pilz_robot_programming.robot import Robot
from pilz_robot_programming.commands import Ptp, Lin, Circ, Gripper, from_euler

__REQUIRED_API_VERSION__ = "1"


def goal_from_args(goal_args):
    goal = []
    for i in range(6):
        goal.append(float(goal_args["<j{:d}>".format(i + 1)]))
    return goal


def goal_pose_from_args(go_args):
    pose = geometry_msgs.msg.Pose()

    pose.position.x = float(go_args['<x>'])
    pose.position.y = float(go_args['<y>'])
    pose.position.z = float(go_args['<z>'])

    pose.orientation = from_euler(float(go_args['<a>']),
                                  float(go_args['<b>']),
                                  float(go_args['<c>']))

    return pose


# Program entry point
if __name__ == '__main__':
    arguments = docopt(__doc__, version='MoveCommandLine 0.1', options_first=True)
    print(arguments)

    rospy.init_node('moveit_commander', anonymous=True)

    # Robot API access
    r = Robot(__REQUIRED_API_VERSION__)

    # Collect the args
    args = {}

    if arguments['<fw>'] is not None:
        args['goal'] = (float(arguments['<fw>']))

    if arguments['--rel']:
        args['relative'] = True

    if arguments['joint']:
        goal_joint = goal_from_args(arguments)

        # Check the type
        if not all(isinstance(n, float) for n in goal_joint):
            print("Goal joints must be float. Abort!")

        args['goal'] = goal_joint

    elif arguments['pose']:
        args['goal'] = goal_pose_from_args(arguments)

    # Set scaling factors
    if arguments['--vel_scale'] is not None:
        args['vel_scale'] = (float(arguments['--vel_scale']))

    if arguments['--acc_scale'] is not None:
        args['acc_scale'] = (float(arguments['--acc_scale']))

    if arguments['--ref'] is not None:
        args['reference_frame'] = (arguments['--ref'])

    if arguments['center']:
        args['center'] = geometry_msgs.msg.Point(float(arguments['<px>']),
                                                 float(arguments['<py>']),
                                                 float(arguments['<pz>']))

    if arguments['interim']:
        args['interim'] = geometry_msgs.msg.Point(float(arguments['<px>']),
                                                  float(arguments['<py>']),
                                                  float(arguments['<pz>']))
    print(args)

    # Set planner id
    if arguments['ptp']:
        r.move(Ptp(**args))

    elif arguments['lin']:
        r.move(Lin(**args))

    elif arguments['circ']:
        r.move(Circ(**args))

    elif arguments['grip']:
        r.move(Gripper(**args))
