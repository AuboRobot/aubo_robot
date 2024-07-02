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

# Acceptance Test for the gripper command using Python-API and real robot
These acceptance tests check that the real robot system is able to perform
gripper commands.

## Prerequisites
  - Properly connect and startup the robot. Make sure an emergency stop is within reach.

## Starting the tests
- Bringup can: `sudo ip link set can0 up type can bitrate 1000000`
- Run `roslaunch prbt_moveit_config moveit_planning_execution.launch sim:=False gripper:=pg70 pipeline:=pilz_command_planner`
- Run `rosrun pilz_robot_programming acceptance_test_gripper_cmd.py`.
  - Please note: Each test must be confirmed before it is executed.
Confirm by entering `y` or `n` to skip test.
  - Please note: At the end of each test the result must be checked.
Confirm success by entering `y`, or `n` in case of failure.


## Test descriptions
- The concrete test descriptions consisting of test-summary, test-sequence and
expected results can be found in `acceptance_test_gripper_cmd.py`.
