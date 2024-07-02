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

# Acceptance Test for the stop1 feature
These acceptance tests check that the real robot system reacts to RUN_PERMITTED messages accordingly.

## Prerequisites for each test
  - Properly connect and startup the robot and power cabinet containing the PSS4000.
    Make sure an emergency stop is within reach.

## 1. Starting while acknowledge button is pressed
### Test Sequence
  1. Press the acknowledge button. Make sure the green light on the power cabinet is on.
     Run `roslaunch prbt_moveit_config moveit_planning_execution.launch sim:=False pipeline:=pilz_industrial_motion_planner`
  2. Release the acknowledge button. Try to move the robot.
  3. Press the acknowledge button. Make sure the green light on the power cabinet is on and try to move the robot.
  4. Using Rviz perform a long motion during which you release the acknowledge button.
### Expected Results
  1. The robot starts properly and is moveable via Rviz.
  2. The robot cannot be moved.
  3. A "Click" indicates the reenabling of the drives. The robot can be moved.
  4. Upon the release of the button the robot stops smoothly without any errors on the console.

## 2. Starting without acknowledge button beeing pressed
### Test Sequence
  1. Do not press the acknowledge button.
     Run `roslaunch prbt_moveit_config moveit_planning_execution.launch sim:=False pipeline:=pilz_industrial_motion_planner`.
     Try to move the robot.
  2. Press the acknowledge button. Again try to move the robot.

### Expected Results
  1. The robot cannot be moved.
  2. A "Click" indicates the enabling of the drives and the robot is moveable via Rviz.

## 3. Loosing ethernet connection during motion
### Test Sequence
  1. Same as Test 1
  2. Move the robot. During the motion pull the ethernet cable from the power cabinet

### Expected Results
  1. Same as Test 1
  2. Robot should stop immediatly and smoothly.

## 4. Fast RUN_PERMITTED switching
### Test Sequence
  1. Same as Test 1
  2. Perform the sequence "Release acknowledge button - press acknowledge button" very quickly. Repeat this test multiple times.
  3. Perform the sequence "Press acknowledge button - release acknowledge button" very quickly. Repeat this test multiple times.

### Expected Results
  1. Same as Test 1
  2. The robot never reaches a state where its drives can not be enabled again through the acknowledge button.
  3. The robot never reaches a state where its drives can not be enabled again through the acknowledge button.
