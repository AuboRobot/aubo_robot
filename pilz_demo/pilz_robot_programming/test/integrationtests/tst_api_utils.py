#!/usr/bin/env python
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

import rospy

from dynamic_reconfigure.srv import Reconfigure, ReconfigureRequest
from dynamic_reconfigure.msg import DoubleParameter


def setOverrideParam(speed_override):
    set_dynamic_parameter = rospy.ServiceProxy('/fake_speed_override_node/set_parameters', Reconfigure)
    rec = ReconfigureRequest()
    double_param = DoubleParameter()
    double_param.name = "speed_override"
    double_param.value = speed_override
    rec.config.doubles.append(double_param)
    set_dynamic_parameter(rec)
