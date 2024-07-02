<!--
Copyright (c) 2019 Pilz GmbH & Co. KG

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

# Acceptance Test for the braketest feature
These acceptance tests check that the real robot system reacts to a BrakeTest service call properly.

## Prerequisites
  - Properly connect and startup the robot and power cabinet containing the PSS4000.
    Make sure an emergency stop is within reach.

### Test Sequence
  1. Press the acknowledge button. Make sure the green light on the power cabinet is on.
  Run `roslaunch prbt_moveit_config moveit_planning_execution.launch sim:=False pipeline:=pilz_industrial_motion_planner`
  2. Run `rosservice call /prbt/brake_test_required`
  3. Run `rosservice call /prbt/execute_braketest`
  4. Try to perform a robot motion via Rviz during the braketest execution
  5. Perform a long robot motion via Rviz and run `rosservice call /prbt/execute_braketest`
  6. Run `rosservice call /prbt/brake_test_required`
### Expected Results
  1. The robot starts properly and is moveable via Rviz.
  2. Service returns `result: 1`
  3. The brake test is executed for all joints (Multiple *clicks* should be audible).
     The service responds with `success: True`.
  4. No robot motion is executed.
  5. No brake test is executed. The service responds with `success: False` and a descriptive error message.
  6. Service returns `result: -1`
