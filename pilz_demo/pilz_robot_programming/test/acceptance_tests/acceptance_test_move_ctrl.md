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

# Acceptance Test for the move-control commands using Python-API and real robot
These acceptance tests check that the real robot system is able to perform
the move-control commands pause, resume and stop.

## Prerequisites
  - Properly connect and startup the robot. Make sure an emergency stop is within reach.

## Starting the tests
- Bringup can: `sudo ip link set can0 up type can bitrate 1000000`
- Run `roslaunch prbt_moveit_config moveit_planning_execution.launch sim:=False gripper:=pg70 pipeline:=pilz_industrial_motion_planner`
- Run `rosrun pilz_robot_programming acceptance_test_move_ctrl.py`.
  - Please note: Each test must be confirmed before it is executed.
Confirm by entering `y` or `n` to skip test.
  - Please note: At the end of each test the result must be checked.
Confirm success by entering `y`, or `n` in case of failure.

## Test descriptions
- The concrete test descriptions consisting of test-summary, test-sequence and
expected results can be found in `acceptance_test_move_ctrl.py`.

## Manual move control tests
- Bringup can: `sudo ip link set can0 up type can bitrate 1000000`
- Run `roslaunch prbt_moveit_config moveit_planning_execution.launch sim:=False gripper:=pg70 pipeline:=pilz_industrial_motion_planner`

#### Test Sequence
  1. Run `rosrun pilz_robot_programming acceptance_test_manual_move_ctrl.py`.
  2. Wait until the robot moves and execute `rosservice call pause_movement`.
  3. Wait until the robot stops and execute `rosservice call resume_movement`.
  4. Wait until the robot moves and execute `rosservice call stop_movement`.
  5. Run `rosrun pilz_robot_programming acceptance_test_manual_move_ctrl.py`.

#### Expected results:
  1. The robot starts to move.
  2. The robot stops moving.
  3. The robot continues moving.
  4. The robot stops moving and the program terminates.
  5. The robot starts to move again.
