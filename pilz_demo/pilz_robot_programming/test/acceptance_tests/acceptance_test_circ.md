<!--
Copyright (c) 2018 Pilz GmbH & Co. KG

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
-->

# Acceptance Test CIRC Motion using Python-API and the real robot
This test checks that the real robot system is able to perform a CIRC Motion to a goal state given by the user. The test is performed using the moveit motion planning plugin. Note that before you can apply the CIRC command the robot has
to be moved out of singularities.

## Prerequisites
  - Properly connect and startup the robot. Make sure a emergency stop is within reach.

## Starting the CIRC acceptance tests
- Bringup can: `sudo ip link set can0 up type can bitrate 1000000`
- Run `roslaunch prbt_moveit_config moveit_planning_execution.launch sim:=False pipeline:=pilz_industrial_motion_planner gripper:=pg70`<br>
**! This test is tailored towards the prbt with gripper. Without gripper some test cases might fail.**
- Run `rosrun pilz_robot_programming acceptance_test_circ.py`.
  - Please note: Each test must be confirmed before it is executed.
Confirm by entering `y` or `n` to skip test.
  - Please note: At the end of each test the result must be checked.
Confirm success by entering `y`, or `n` in case of failure.


## Test descriptions
- The concrete test descriptions consisting of test-summary, test-sequence and
expected results can be found in `acceptance_test_circ.py`.
