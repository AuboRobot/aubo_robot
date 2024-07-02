#
# Copyright (c) 2019 Pilz GmbH & Co. KG
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

int8 value

# values of CANopen object brake_test_status, see
# https://www.pilz.com/download/open/PRBT_6_Operat_Manual_1004685-EN-02.pdf (p.100)
int8 STATUS_UNKNOWN = 0
int8 STATUS_PERFORMING = 1
int8 STATUS_SUCCESS = 2
int8 STATUS_NO_SUCCESS = 3
int8 STATUS_NO_CONTROL = 4

# service call failures
int8 TRIGGER_BRAKETEST_SERVICE_FAILURE = 50
int8 GET_NODE_NAMES_FAILURE  = 51
int8 GET_DURATION_FAILURE = 52
int8 GET_STATUS_FAILURE = 53
int8 START_BRAKE_TEST_FAILURE = 54

# other reasons for blocking the execution of brake tests
int8 BRAKETEST_ALREADY_EXECUTING = 70
int8 ROBOT_MOTION_DETECTED = 71

# other
int8 FAILURE = 99
