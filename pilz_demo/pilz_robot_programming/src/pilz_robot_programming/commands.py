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

"""API for easy usage of Pilz robot commands."""

from __future__ import absolute_import

from copy import deepcopy
from math import pi
from operator import add

import rospy
import tf2_geometry_msgs  # for buffer.transform() to eat a geometry_msgs.Pose directly
from tf_conversions import transformations
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from moveit_msgs.msg import (Constraints, JointConstraint, MotionPlanRequest,
                             MotionSequenceItem, MoveGroupSequenceGoal,
                             OrientationConstraint, PlanningOptions,
                             PositionConstraint)
import shape_msgs.msg as shape_msgs

from .move_control_request import _MoveControlState

__version__ = '0.0.dev1'

# Default velocities
_DEFAULT_CARTESIAN_VEL_SCALE = 0.1
_DEFAULT_JOINT_VEL_SCALE = 1.0

# Default acceleration
_DEFAULT_ACC_SCALE = 0.1

# Tolerance for cartesian pose
_DEFAULT_POSITION_TOLERANCE = 2e-3
_DEFAULT_ORIENTATION_TOLERANCE = 1e-5

# axis sequence of euler angles
_AXIS_SEQUENCE = "rzyz"

_DEFAULT_PLANNING_GROUP = "manipulator_i5"
_DEFAULT_TARGET_LINK = "wrist3_Link"
_DEFAULT_GRIPPER_PLANNING_GROUP = "gripper"
_DEFAULT_BASE_LINK = "world"


class _AbstractCmd(object):
    """Base class for all commands."""

    def __init__(self, *args, **kwargs):
        super(_AbstractCmd, self).__init__(*args, **kwargs)
        # set robot state as empty diff in planning scene to start with current planning scene
        self._planning_options = PlanningOptions()
        self._planning_options.planning_scene_diff.robot_state.is_diff = True

        return

    def _get_sequence_request(self, robot):
        """Called by robot class to generate a sequence request.
        @note Even single commands are handled as an one-item sequence.
        """
        raise NotImplementedError("Cannot execute abstract command")

    def _execute(self, robot):
        rospy.logdebug("Executing command.")

        try:
            sequence_action_goal = self._get_sequence_request(robot)
        except Exception as e:
            rospy.logerr(str(e))
            return robot._FAILURE

        sequence_action_goal.planning_options = self._planning_options

        rospy.logdebug("Sending goal.")
        if not _AbstractCmd._locked_send_goal(robot, robot._sequence_client, sequence_action_goal):
            rospy.logdebug("Command was paused before goal could be send.")
            return robot._STOPPED
        rospy.logdebug("Wait till motion finished...")
        done = robot._sequence_client.wait_for_result()
        rospy.logdebug("Function wait_for_result() of command finished.")
        assert done is True, "Function wait_for_result() is finished but the goal is not done."

        result_code = robot._sequence_client.get_result()
        if result_code is None:  # pragma: no cover  Paranoia-check should actually never happen.
            rospy.logerr("No result received from action server.")
            return robot._FAILURE

        return robot._map_error_code(result_code.response.error_code)

    @staticmethod
    def _locked_send_goal(robot, action_client, goal):
        """By synchronizing to the robot._move_control_state_machine this function ensures that
        parallel calls to stop/pause and send_goal() cannot cause undefined or erroneous behavior.

        In other words, while stop() is changing the move control order, nobody is allowed to send new goals.
        Also if pause() is changing the move control order, nobody is allowed to send new goals until the move control
        order is changed.
        But it also means that if somebody is currently sending a goal, all move control order changes triggered
        by pause() or stop() have to wait until the goal is send. This ensure clear and predictable behavior.

        :return: True if the function sent a goal to Moveit, otherwise False.
        :rtype: bool
        """
        with robot._move_ctrl_sm:
            # Only if no orders are present, the calling thread is allowed to send a goal to Moveit.
            if robot._move_ctrl_sm.state != _MoveControlState.NO_REQUEST or rospy.core.is_shutdown_requested():
                return False

            action_client.send_goal(goal)
            return True

    def __str__(self):
        out_str = self.__class__.__name__
        return out_str

    __repr__ = __str__


class BaseCmd(_AbstractCmd):
    """Base class for all single commands.

    :param goal: The goal of the motion, which can be given in joint (list or tuple of float, in the order of active
        joints in the planning group) or Cartesian space. For geometry_msgs/Pose you can specify the reference frame
        as extra parameter `reference_frame` (see below). If a PoseStamped is passed as goal, timestamp has to be zero
        and the `frame_id` from the Header is used instead of `reference_frame`

    :note:
        The geometry_msgs/Pose consists of position and orientation (quaternion). When creating an instance of
        geometry_msgs/Pose, the position and orientation can be left as uninitialized. The uninitialized position is
        considered as zero position. The uninitialized orientation is considered as keeping the current orientation
        unchanged. This difference is because uninitialized position can not be recognized when comparing with a zero
        position but uninitialized orientation can.

    :param planning_group: Name of the planning group, default as "manipulator".

    :param target_link: Name of the target link if Cartesian goal is given, default as "prbt_tcp"

    :param relative: Has to be set to:

            * :py:obj:`False` if the goal states the target position as absolute position with regard to base coordinate
                              system.
            * :py:obj:`True` if the goal states the target position as offset relative to the current robot position.

            The orientation is added as offset to the euler-angles.
            The offset has to be stated with regard to the base coordinate system.
            E.g. to move the robot 0.1m up and tilt it 10degrees around the global z-axis use:
            ::

                Ptp(goal=Pose(position=Point(0., 0., 0.1), orientation=from_euler(math.radians(10), 0., 0.)),
                    vel_scale=0.4)


            Note the gimbal lock, if you pass a relative rotation to this function: If b==0, the values for a and c
            depend on each other and may thus give arbitrary euler angles, when converting back from quaternion to
            euler internally.
            The function assumes, you want to take the shorter rotation distance, so you should only pass
            rotations smaller than 180 degrees for relative movements.

    :param reference_frame: The frame of reference parameter allows to change the reference coordinate system for the
        passed goal position and orientation.
        Any published tf can be used as frame by reference. The reference_frame has to be a valid tf id string.
        Setting no reference_frame will use "prbt_base" as default.

    :type relative: bool
    :type reference_frame: string
    """

    def __init__(self, goal=None, planning_group=_DEFAULT_PLANNING_GROUP, target_link=_DEFAULT_TARGET_LINK,
                 vel_scale=_DEFAULT_CARTESIAN_VEL_SCALE, acc_scale=_DEFAULT_ACC_SCALE, relative=False,
                 reference_frame=_DEFAULT_BASE_LINK, *args, **kwargs):
        super(BaseCmd, self).__init__(*args, **kwargs)

        # Needs to be set by derived classes
        self._planner_id = None

        try:
            if isinstance(goal, str):
                raise TypeError()
            self._goal = tuple(goal)
        except TypeError:
            self._goal = goal

        self._planning_group = planning_group
        self._target_link = target_link
        self._vel_scale = vel_scale
        self._acc_scale = acc_scale
        self._relative = relative

        self._reference_frame = reference_frame

    def __str__(self):
        out_str = _AbstractCmd.__str__(self)
        out_str += " vel_scale: " + str(self._vel_scale)
        out_str += " acc_scale: " + str(self._acc_scale)
        out_str += " reference: " + str(self._reference_frame)
        return out_str

    def __eq__(self, other):
        if isinstance(other, BaseCmd):
            return hash(self) == hash(other)
        return NotImplemented

    def __ne__(self, other):
        x = self.__eq__(other)
        if x is not NotImplemented:
            return not x
        return NotImplemented

    def __hash__(self):
        return hash(tuple(sorted([str(t) for t in self.__dict__.items()])))

    __repr__ = __str__

    def _cmd_to_request(self, robot):
        """Transforms the given command to a MotionPlanRequest."""
        req = MotionPlanRequest()

        self._robot_reference_frame = robot.get_planning_frame()
        self._active_joints = robot.get_active_joints(self._planning_group)
        self._start_joint_states = robot.get_current_joint_states(planning_group=self._planning_group)
        self._start_pose = robot.get_current_pose(target_link=self._target_link, base=self._reference_frame)
        self._tf_buffer = robot.tf_buffer_

        # Set general info
        req.planner_id = self._planner_id
        req.group_name = self._planning_group
        req.max_velocity_scaling_factor = self._vel_scale * robot._speed_override
        req.max_acceleration_scaling_factor = self._acc_scale * self._calc_acc_scale(robot._speed_override)
        req.allowed_planning_time = 1.0

        # Set an empty diff as start_state => the current state is used by the planner
        req.start_state.is_diff = True

        # Set goal constraint
        if self._goal is None:
            raise NameError("Goal is not given.")

        convertion_methods = [self._pose_to_constraint,
                              self._pose_stamped_to_constraint,
                              self._joint_values_to_constraint]
        error_list = []

        for method in convertion_methods:
            try:
                req.goal_constraints = method()
                break
            except (TypeError, AttributeError) as e:
                error_list.append(e)
                pass
        else:
            raise NotImplementedError("Unknown type of goal: %s \nerrors: %s" % (str(self._goal), str(error_list)))

        return req

    def _check_header_time(self):
        if self._goal.header.stamp != rospy.Time(0, 0):
            raise ValueError("Given goal has unsupported time for future execution.")

    def _joint_values_to_constraint(self, joint_names=()):
        if isinstance(self._goal, str):
            raise TypeError("String is not convertible into joint values.")
        joint_names = joint_names if len(joint_names) != 0 else self._active_joints
        joint_values = list(self._get_joint_pose())
        if len(joint_names) != len(joint_values):
            raise IndexError("Given joint goal does not match the active joints " + str(joint_names) + ".")

        goal_constraints = Constraints()
        goal_constraints.joint_constraints = [JointConstraint(joint_name=name, position=value, weight=1)
                                              for name, value in zip(joint_names, joint_values)]
        return [goal_constraints]

    def _pose_to_constraint(self):
        goal_pose = self._get_goal_pose()
        goal_constraints = Constraints()
        robot_reference_frame = self._robot_reference_frame
        goal_constraints.orientation_constraints.append(
            _to_ori_constraint(goal_pose, robot_reference_frame, self._target_link))
        goal_constraints.position_constraints.append(
            _to_pose_constraint(goal_pose, robot_reference_frame, self._target_link))
        return [goal_constraints]

    def _pose_stamped_to_constraint(self):
        self._reference_frame = self._goal.header.frame_id if self._goal.header.frame_id != "" else _DEFAULT_BASE_LINK
        self._check_header_time()
        self._goal = self._goal.pose
        return self._pose_to_constraint()

    def _get_sequence_request(self, robot):
        """Constructs a sequence request from the command.
        BaseCmds construct a sequence request with a single item and a blend_radius 0"""
        sequence_action_goal = MoveGroupSequenceGoal()

        # Create and fill request
        sequence_item = MotionSequenceItem()
        sequence_item.blend_radius = 0.0

        # Fill MotionPlanRequest
        sequence_item.req = self._cmd_to_request(robot)

        # Add request to goal
        sequence_action_goal.request.items.append(sequence_item)

        return sequence_action_goal

    def _get_goal_pose(self):
        """Determines the goal pose for the given command."""
        if self._relative:
            self._goal = _pose_relative_to_absolute(self._start_pose, self._goal)

        if not self._reference_frame == _DEFAULT_BASE_LINK:
            return self._to_robot_reference(self._reference_frame, self._goal)

        # in case of uninitialized orientation, set the goal orientation as current
        if _is_quaternion_initialized(self._goal.orientation):
            return self._goal
        else:
            return Pose(position=self._goal.position, orientation=self._start_pose.orientation)

    def _get_joint_pose(self):
        """Determines the joint goal for the given command."""
        goal_joint_state = self._goal if not self._relative else \
            map(add, self._goal, self._start_joint_states)
        return goal_joint_state

    @staticmethod
    def _calc_acc_scale(vel_scale):  # pragma: no cover  Abstract method
        raise NotImplementedError("Needs to be defined by child class")

    def _to_robot_reference(self, pose_frame, goal_pose_custom_ref):
        """ Transforms a pose from a custom reference frame to one in robot reference frame.

        :param pose_frame: is the custom reference frame of the pose.

        :param goal_pose_custom_ref: pose in the custom reference frame.

        :return: A goal pose in robot reference frame.
        """
        if not _is_quaternion_initialized(goal_pose_custom_ref.orientation):
            goal_pose_custom_ref.orientation = Quaternion(w=1)
        if pose_frame == self._robot_reference_frame:
            return goal_pose_custom_ref

        stamped = PoseStamped()
        stamped.header.frame_id = pose_frame
        stamped.pose = goal_pose_custom_ref
        return self._tf_buffer.transform(stamped, self._robot_reference_frame).pose


class Ptp(BaseCmd):
    """Represents a single point-to-point (Ptp) command.
    A :py:class:`Ptp` command allows the user to quickly move the robot from its current position to a specified point
    in space (goal). The trajectory taken to reach the goal is defined by the underlying planning
    algorithms and cannot not be defined by the user.

    :param vel_scale: The velocity scaling factor allows to limit the highest possible axis velocity.
        The velocity scaling factor is a scalar. The value is applied to all axes.
        The value is given in percentage of the maximal velocity of an axis and has to be
        in range: (0,1]. The allowed axis velocity for each axis is calculated as follows:

            allowed axis velocity = vel_scale * maximal axis velocity

    :param acc_scale: The acceleration scaling factor allows to limit the highest possible axis acceleration.
        The acceleration scaling factor is a scalar value. The value is applied to all axes.
        The value is given in percentage of the maximal acceleration of an axis and
        has to be in range: (0,1]. The allowed axis acceleration for each axis is calculated as follows:

            allowed axis acceleration = vel_scale * maximal axis acceleration

        If no acceleration scaling factor is given, the acceleration scaling factor is set as follows:

            acc_scale = vel_scale * vel_scale
    """

    def __init__(self, vel_scale=_DEFAULT_JOINT_VEL_SCALE, acc_scale=None, *args, **kwargs):
        acc_scale_final = acc_scale if acc_scale is not None else Ptp._calc_acc_scale(vel_scale)
        super(Ptp, self).__init__(vel_scale=vel_scale, acc_scale=acc_scale_final, *args, **kwargs)
        self._planner_id = "PTP"

    def __str__(self):
        out_str = BaseCmd.__str__(self)
        if self._relative:
            out_str += " relative: True"
        if isinstance(self._goal, Pose) or isinstance(self._goal, PoseStamped):
            out_str += " Cartesian goal:\n" + str(self._goal)
        elif isinstance(self._goal, tuple):
            out_str += " joint goal: " + str(self._goal)
        return out_str

    __repr__ = __str__

    @staticmethod
    def _calc_acc_scale(vel_scale):
        return vel_scale * vel_scale


class Lin(BaseCmd):
    """Represents a linear command.
    A :py:class:`Lin` command allows the user to move the robot from its current position to a specified point
    in space (goal). The trajectory taken to reach the goal is a straight line (in Cartesian space).

    :param vel_scale: The velocity scaling factor allows to limit the highest possible cartesian velocity
        of the TCP frame. The velocity scaling factor is a scalar value.
        The value is given in percentage of the maximal allowed cartesian velocity and has to be
        in range: (0,1]. The allowed cartesian velocity of the TCP frame is calculated as follows:

            allowed cartesian velocity = vel_scale * maximal cartesian velocity

    :param acc_scale: The acceleration scaling factor allows to limit the highest possible cartesian acceleration
        of the TCP frame. The acceleration scaling factor is a scalar value.
        The value is given in percentage of the maximal allowed cartesian acceleration and has to be
        in range: (0,1]. The allowed cartesian acceleration of the TCP frame is calculated as follows:

            allowed cartesian acceleration = acc_scale * maximal cartesian acceleration

        If no acceleration scaling factor is given, the acceleration scaling factor is set as follows:

            acc_scale = vel_scale
    """

    def __init__(self, vel_scale=_DEFAULT_CARTESIAN_VEL_SCALE, acc_scale=None, *args, **kwargs):

        acc_scale_final = acc_scale if acc_scale is not None else Lin._calc_acc_scale(vel_scale)

        super(Lin, self).__init__(vel_scale=vel_scale, acc_scale=acc_scale_final, *args, **kwargs)

        self._planner_id = "LIN"

    def __str__(self):
        out_str = BaseCmd.__str__(self)
        if self._relative:
            out_str += " relative: True"
        if isinstance(self._goal, Pose) or isinstance(self._goal, PoseStamped):
            out_str += " Cartesian goal:\n" + str(self._goal)
        elif isinstance(self._goal, tuple):
            out_str += " joint goal: " + str(self._goal)
        return out_str

    __repr__ = __str__

    @staticmethod
    def _calc_acc_scale(vel_scale):
        return vel_scale


class Circ(BaseCmd):
    """Represents a circular command. A :py:class:`Circ` command allows the user to move the robot from its
    current position to a specified point in space (goal).
    The trajectory taken to reach the goal represents a circle (in Cartesian space). The circle is defined by the
    current position of the robot, the specified interim/center point and the goal position.

    :note:
        The circle can be completely defined by stating a interim `or` an center position.
        However, only one of both should be stated.

    :param interim: Position in cartesian space (geometry_msgs/Point),
        which lies on the circle on which the robot is supposed to move.
        The position has to lie between the current position of the robot and the goal position.
        The interim position indicates in which direction of the circle the robot is supposed to move.

    :param center: The center point (stated in Cartesian space) of the circle on which the robot is supposed to move.
        If the center point is given, the robot moves in the direction of the smallest angle to the goal.

    :param vel_scale: The velocity scaling factor allows to limit the highest possible cartesian velocity
        of the TCP frame. The velocity scaling factor is a scalar value.
        The value is given in percentage of the maximal allowed cartesian velocity and has to be
        in range: (0,1]. The allowed cartesian velocity of the TCP frame is calculated as follows:

            allowed cartesian velocity = vel_scale * maximal cartesian velocity

    :param acc_scale: The acceleration scaling factor allows to limit the highest possible cartesian acceleration
        of the TCP frame. The acceleration scaling factor is a scalar value.
        The value is given in percentage of the maximal allowed cartesian acceleration and has to be
        in range: (0,1]. The allowed cartesian acceleration of the TCP frame is calculated as follows:

            allowed cartesian acceleration = acc_scale * maximal cartesian acceleration

        If no acceleration scaling factor is given, the acceleration scaling factor is set as follows:

            acc_scale = vel_scale
    """

    def __init__(self, interim=None, center=None, vel_scale=_DEFAULT_CARTESIAN_VEL_SCALE, acc_scale=None,
                 *args, **kwargs):

        acc_scale_final = acc_scale if acc_scale is not None else Circ._calc_acc_scale(vel_scale)

        super(Circ, self).__init__(vel_scale=vel_scale, acc_scale=acc_scale_final, *args, **kwargs)

        self._planner_id = "CIRC"
        self._interim = interim
        self._center = center

    def __str__(self):
        out_str = BaseCmd.__str__(self)
        if isinstance(self._goal, Pose) and self._goal is not None:
            out_str += " goal:\n" + str(self._goal)
        if self._interim is not None:
            out_str += "\ninterim:\n" + str(self._interim)
        if self._center is not None:
            out_str += "\ncenter:\n" + str(self._center)
        return out_str

    __repr__ = __str__

    def _cmd_to_request(self, robot):
        req = BaseCmd._cmd_to_request(self, robot)

        if self._center is not None and self._interim is not None:
            raise NameError("Both center and interim are set for circ command!")

        if self._center is None and self._interim is None:
            raise NameError("Both center and interim are not set for circ command!")

        # Set the position constraint
        path_point = Pose()
        if self._center is not None:
            req.path_constraints.name = 'center'
            path_point.position = self._center
        else:
            req.path_constraints.name = 'interim'
            path_point.position = self._interim

        if self._reference_frame:
            path_point = self._to_robot_reference(self._reference_frame, path_point)

        position_constraint = _to_pose_constraint(path_point, self._robot_reference_frame, self._target_link,
                                                  float('+inf'))

        req.path_constraints.position_constraints = [position_constraint]

        return req

    @staticmethod
    def _calc_acc_scale(vel_scale):
        return vel_scale


class _SequenceSubCmd(object):
    def __init__(self, cmd, blend_radius=0.):
        self.cmd = cmd
        self.blend_radius = blend_radius

    def __str__(self):
        out_str = self.__class__.__name__
        out_str += " - " + str(self.cmd)
        out_str += "\nblend radius: " + str(self.blend_radius)
        return out_str

    __repr__ = __str__


class Sequence(_AbstractCmd):
    """ Represents an overall Sequence command. A :py:class:`Sequence` consists of one or more
     robot motion commands. All commands in a sequence are planned first. After all
     commands in a sequence are planned, they are executed.

     If the blending radius between two or more commands is greater than zero, the commands are blended
     together, in other words, the robot will not stop at the end of each command. To allow a smooth transition from
     one trajectory to the next (in case of blending), the original trajectories are altered slightly
     within the sphere defined by the blending radius.

     :note: In case the blend radius is zero, the robot executes the robot motion commands as if they are sent
            separately.

     :note: The robot always stops between gripper and non-gripper commands.

     :note: Gripper commands cannot be blended together.

     :note: In case the planning of a command in a sequence fails, non of the commands in the sequence are executed.

    """

    def __init__(self, *args, **kwargs):
        super(Sequence, self).__init__(*args, **kwargs)
        # List of tuples containing commands and blend radii
        self.items = []

    def append(self, cmd, blend_radius=0):
        """Adds the given robot motion command to the sequence.

        :param cmd: The robot motion command which has to be added to the sequence.
            The blending happens between the specified command and the command following the specified command
            if a non-zero blend_radius is defined. Otherwise, if the blend radius is zero, the commands will
            execute consecutively.
            The blend radius preceding a gripper command is always ignored. The blend radius stated with a gripper
            command is also ignored.
        :type cmd: :py:class:`pilz_robot_programming.commands.BaseCmd`

        :param blend_radius: The blending radius states how much the robot trajectory can deviate from the
            original trajectory (trajectory without blending) to blend the robot motion from one trajectory to the next.
            The blending happens inside a sphere with a radius specified by the blending radius. When the trajectory
            leaves the sphere the trajectory is back on the original trajectory.
        :type blend_radius: float

        :note:
            The last command of the sequence has to have zero blending radius which can be achieved
            by omitting the last blend radius argument.

        """
        self.items.append(_SequenceSubCmd(cmd, blend_radius))

    def _get_sequence_request(self, robot):
        sequence_action_goal = MoveGroupSequenceGoal()

        for item in self.items:
            # Create and fill request
            curr_sequence_req = MotionSequenceItem()
            curr_sequence_req.blend_radius = item.blend_radius

            # Fill MotionPlanRequest
            curr_sequence_req.req = item.cmd._cmd_to_request(robot)

            # Add request to goal
            sequence_action_goal.request.items.append(curr_sequence_req)

        return sequence_action_goal

    def __str__(self):
        out_str = _AbstractCmd.__str__(self)
        out_str += ":\n"
        for item in self.items:
            out_str += str(item)
            out_str += "\n"
        return out_str

    __repr__ = __str__


class Gripper(BaseCmd):
    """Represents a gripper command to open and close the gripper.
    A :py:class:`gripper` command allows the user to move the gripper finger to desired opening width.

    :param goal: half of the opening width in meter (0 to 0.03m).

    :param vel_scale: The velocity scaling factor allows to limit the highest possible axis velocity.
        The velocity scaling factor is a scalar.
        The value is given in percentage of the maximal velocity of an axis and has to be
        in range: (0,1]. The allowed axis velocity for each axis is calculated as follows:

            allowed axis velocity = vel_scale * maximal axis velocity
    """

    def __init__(self, goal, vel_scale=_DEFAULT_CARTESIAN_VEL_SCALE, *args, **kwargs):
        super(Gripper, self).__init__(goal=goal, planning_group=_DEFAULT_GRIPPER_PLANNING_GROUP,
                                      vel_scale=vel_scale, relative=False, *args, **kwargs)

    def __str__(self):
        out_str = _AbstractCmd.__str__(self)
        if self._goal is not None:
            out_str += " gripper goal: " + str(self._goal)
        out_str += " velocity scaling: " + str(self._vel_scale)
        return out_str

    __repr__ = __str__

    def _cmd_to_request(self, robot):
        """Transforms the gripper command to a MotionPlanRequest."""
        req = MotionPlanRequest()

        # Set general info
        req.planner_id = "PTP"
        req.group_name = self._planning_group
        req.max_velocity_scaling_factor = self._vel_scale
        req.max_acceleration_scaling_factor = self._acc_scale
        req.allowed_planning_time = 1.0
        # Set an empty diff as start_state => the current state is used by the planner
        req.start_state.is_diff = True

        # create goal constraints
        goal_constraints = Constraints()
        if isinstance(self._goal, (float, int)):
            joint_names = robot._robot_commander.get_group(self._planning_group).get_active_joints()

            if len(joint_names) != 1:
                raise IndexError("PG70 should have only one joint. But group " + req.group_name +
                                 " contains " + str(len(joint_names)) + " joints.")

            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_names[0]
            joint_constraint.position = float(self._goal)
            joint_constraint.weight = 1
            goal_constraints.joint_constraints.append(joint_constraint)

        else:
            raise NotImplementedError("Unknown type of goal is given.")

        req.goal_constraints.append(goal_constraints)

        return req


def _to_ori_constraint(pose, reference_frame, link_name, orientation_tolerance=_DEFAULT_ORIENTATION_TOLERANCE):
    """Returns an orientation constraint suitable for ActionGoal's."""
    ori_con = OrientationConstraint()
    ori_con.header.frame_id = reference_frame
    ori_con.link_name = link_name
    ori_con.orientation = pose.orientation
    ori_con.absolute_x_axis_tolerance = orientation_tolerance
    ori_con.absolute_y_axis_tolerance = orientation_tolerance
    ori_con.absolute_z_axis_tolerance = orientation_tolerance
    ori_con.weight = 1
    return ori_con


def _to_pose_constraint(pose, reference_frame, link_name, position_tolerance=_DEFAULT_POSITION_TOLERANCE):
    """Returns an position constraint suitable for ActionGoal's."""
    pos_con = PositionConstraint()
    pos_con.header.frame_id = reference_frame
    pos_con.link_name = link_name
    pos_con.constraint_region.primitive_poses.append(pose)
    pos_con.weight = 1

    region = shape_msgs.SolidPrimitive()
    region.type = shape_msgs.SolidPrimitive.SPHERE
    region.dimensions.append(position_tolerance)

    pos_con.constraint_region.primitives.append(region)

    return pos_con


def _is_quaternion_initialized(quaternion):
    """Check if the quaternion is initialized"""
    return quaternion != Quaternion()  # check, if all fields are zero


def _pose_relative_to_absolute(current_pose, relative_pose):
    """Add the offset relative_pose to current_pose and return an absolute goal pose"""
    goal_pose = deepcopy(current_pose)

    # translation
    goal_pose.position.x += relative_pose.position.x
    goal_pose.position.y += relative_pose.position.y
    goal_pose.position.z += relative_pose.position.z

    # rotation
    a_cur, b_cur, c_cur = transformations.euler_from_quaternion([current_pose.orientation.x,
                                                                 current_pose.orientation.y,
                                                                 current_pose.orientation.z,
                                                                 current_pose.orientation.w],
                                                                axes=_AXIS_SEQUENCE)

    a, b, c = transformations.euler_from_quaternion([relative_pose.orientation.x,
                                                     relative_pose.orientation.y,
                                                     relative_pose.orientation.z,
                                                     relative_pose.orientation.w],
                                                    axes=_AXIS_SEQUENCE)

    # choose shorter distance for relative movement:
    # if we set b -> -b and rotate a and c by 180 degrees,
    # we obtain the same rotation.
    # But we have to make sure, to stay within the -pi,pi range.
    # For b we explicitly allow (-pi,pi) here as well, since the angles
    # are an offset to the current angle, so that b may be negative.
    a2, b2, c2 = min([pi+a, -pi+a], key=abs), -b, min([pi+c, -pi+c], key=abs)
    if abs(a)+abs(b)+abs(c) > abs(a2)+abs(b2)+abs(c2):
        a, b, c = a2, b2, c2

    goal_pose.orientation = from_euler(a + a_cur, b + b_cur, c + c_cur)

    return goal_pose


def from_euler(a, b, c):
    """Convert euler angles into a `geometry.msg.Quaternion`.

    Pass euler angles a, b, c in intrinsic ZYZ convention (in radians).

    Use this function to fill pose values for :py:class:`Ptp` / :py:class:`Lin` commands:
    ::

        r.move(Ptp(goal=Pose(position=Point(0.6, -0.3, 0.2), orientation=from_euler(0, pi, 0))))

    :param a: rotates around the z-axis.
    :param b: rotates around the new y-axis.
    :param c: rotates around the new z-axis.

    :note:
        e.g. (0, pi, 0) orients the tool downwards,
        (pi/2., pi/2., 0) horizontal west

    """
    return Quaternion(*transformations.quaternion_from_euler(a, b, c, axes=_AXIS_SEQUENCE))
