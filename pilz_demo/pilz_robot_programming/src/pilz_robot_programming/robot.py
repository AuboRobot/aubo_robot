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
import psutil
import time
import threading

from actionlib import SimpleActionClient, GoalStatus
from geometry_msgs.msg import Quaternion, PoseStamped, Pose
from moveit_commander import RobotCommander, MoveItCommanderException
from moveit_msgs.msg import MoveItErrorCodes
import rospy
from std_msgs.msg import Header
from std_srvs.srv import Trigger
import tf2_ros
import tf2_geometry_msgs  # for buffer.transform() to eat a geometry_msgs.Pose directly

from moveit_msgs.msg import MoveGroupSequenceAction
from pilz_msgs.msg import IsBrakeTestRequiredResult
from pilz_msgs.srv import GetSpeedOverride, IsBrakeTestRequired, BrakeTest

from .move_control_request import _MoveControlState, MoveControlAction, _MoveControlStateMachine
from .commands import _AbstractCmd, _DEFAULT_PLANNING_GROUP, _DEFAULT_TARGET_LINK, _DEFAULT_BASE_LINK, Sequence
from .exceptions import *

__version__ = '1.1.0'

# Due to bug in actionlib we have to take care about validity of transitions when cancelling
_VALID_GOAL_STATUS_FOR_CANCEL = [GoalStatus.PENDING, GoalStatus.ACTIVE]


class Robot(object):
    """
    Main component of the API which allows the user to execute robot motion commands and pause, resume or stop the
    execution. The following commands are currently supported:

    * :py:class:`.Ptp`
    * :py:class:`.Lin`
    * :py:class:`.Circ`
    * :py:class:`.Sequence`
    * :py:class:`.Gripper`

    For a more detailed description of the individual commands please see the documentation of
    the corresponding command. Especially see the documentation of the `pilz_industrial_motion_planner` package
    to get more information on additional parameters that can be configured in the MoveIt! plugin.

    The commands are executed with the help of Moveit.

    :note:
        To any given time only one instance of the Robot class is allowed to exist.

    :note:
        Before you create an instance of :py:class:`.Robot`, ensure that MoveIt is up and running,
        because the constructor blocks until all necessary connections to Moveit are established.
        Currently connections to the following topics have to be established before the function
        finishes:

        * move_group
        * sequence_move_group

    :note:
        Currently the API does not support creating a new instance of :py:class:`.Robot` after deleting an old one in
        the same program. However this can be realized by calling :py:meth:`_release` before the deletion.

    :param version:
        To ensure that always the correct API version is used, it is necessary to state
        which version of the API is expected. If the given version does not match the
        version of the underlying API then an exception is thrown. Only the major version number
        is considered.

    :raises RobotVersionError: if the given version string does not match the module version.
    :raises RobotMultiInstancesError: if an instance of Robot class already exists.
    """

    # ++++++++++++++++++++++++++
    # + Return value constants +
    # ++++++++++++++++++++++++++

    # Command finished successfully
    _SUCCESS = 1
    # Command was stopped; Value based on Moveit error code for preempted
    _STOPPED = -7
    # Something went wrong while executing the command
    _FAILURE = 99999

    # Topic / Service names
    _PAUSE_TOPIC_NAME = "pause_movement"
    _RESUME_TOPIC_NAME = "resume_movement"
    _STOP_TOPIC_NAME = "stop_movement"
    _SEQUENCE_TOPIC = "sequence_move_group"
    _BRAKE_TEST_EXECUTE_SRV = "/prbt/execute_braketest"
    _GET_SPEED_OVERRIDE_SRV = "/prbt/get_speed_override"
    _BRAKE_TEST_REQUIRED_SRV = "/prbt/brake_test_required"
    _INSTANCE_PARAM = "/robot_api_instance"

    # string constants
    _PID_STRING = "pid"
    _PROCESS_CREATE_TIME_STRING = "create_time"

    # time constant
    _SERVICE_WAIT_TIMEOUT_S = 1

    def __init__(self, version=None, *args, **kwargs):
        super(Robot, self).__init__(*args, **kwargs)
        rospy.logdebug("Initialize Robot Api.")

        # tf listener is necessary for pose transformation
        # when using custom reference frames.
        self.tf_buffer_ = tf2_ros.Buffer()
        self.tf_listener_ = tf2_ros.TransformListener(self.tf_buffer_)

        self._move_lock = threading.Lock()

        # manage the move control request
        self._move_ctrl_sm = _MoveControlStateMachine()

        self._single_instance_flag = False

        self._check_version(version)

        self._claim_single_instance()

        self._establish_connections()

        # We use this auxiliary member to implement a lazy initialization
        # for the '_robot_commander' member. The lazy initialization
        # is necessary to ensure testability.
        self.__robot_commander = None

        # cleanup when ros terminates
        rospy.on_shutdown(self._on_shutdown)

    # The moveit RobotCommander is needed to retrieve robot semantic information.
    # To allow proper testing the RobotCommander is instantiated via lazy initialization.
    @property
    def _robot_commander(self):
        # lazy initialization
        if self.__robot_commander is None:
            self.__robot_commander = RobotCommander()
            rospy.loginfo("RobotCommander created.")
        return self.__robot_commander

    @_robot_commander.setter
    def _robot_commander(self, robot_commander):
        self.__robot_commander = robot_commander

    @property
    def _speed_override(self):
        """ Returns the currently active speed override

        Both velocity and acceleration scaling of a command are factorized during :py:meth:`move`.
        The command itself remains untouched.
        """
        res = self._get_speed_override_srv()

        return res.speed_override

    def get_planning_frame(self):
        """Get the name of the frame in which the robot is planning."""
        return self._robot_commander.get_planning_frame()

    def get_active_joints(self, planning_group):
        "Returns the joints contained in the specified planning group"
        return self._robot_commander.get_group(planning_group).get_active_joints()

    def get_current_joint_states(self, planning_group=_DEFAULT_PLANNING_GROUP):
        """Returns the current joint state values of the robot.
        :param planning_group: Name of the planning group, default value is "manipulator".
        :return: Returns the current joint values as array
        :rtype: array of floats
        :raises RobotCurrentStateError if given planning group does not exist.
        """
        try:
            return self._robot_commander.get_group(planning_group).get_current_joint_values()
        except MoveItCommanderException as e:
            rospy.logerr(e)
            raise RobotCurrentStateError(e)

    def get_current_pose_stamped(self, target_link=_DEFAULT_TARGET_LINK, base=_DEFAULT_BASE_LINK):
        """Returns the current stamped pose of target link in the reference frame.
        :param target_link: Name of the target_link, default value is "prbt_tcp".
        :param base: The target reference system of the pose, default ist "prbt_base".
        :return: Returns the stamped pose of the given frame
        :rtype: geometry_msgs.msg.PoseStamped
        :raises RobotCurrentStateError if the pose of the given frame is not known
        """
        try:
            zero_pose = PoseStamped(header=Header(frame_id=target_link),
                                    pose=Pose(orientation=Quaternion(w=1.0)))
            current_pose = self.tf_buffer_.transform(zero_pose, base, rospy.Duration(5, 0))
            return current_pose
        except tf2_ros.LookupException as e:
            rospy.logerr(e)
            raise RobotCurrentStateError(e)

    def get_current_pose(self, target_link=_DEFAULT_TARGET_LINK, base=_DEFAULT_BASE_LINK):
        """Returns the current pose of target link in the reference frame.
        :param target_link: Name of the target_link, default value is "prbt_tcp".
        :param base: The target reference system of the pose, default ist "prbt_base".
        :return: Returns the pose of the given frame
        :rtype: geometry_msgs.msg.Pose
        :raises RobotCurrentStateError if the pose of the given frame is not known
        """
        return self.get_current_pose_stamped(target_link, base).pose

    def move(self, cmd):
        """ Allows the user to start/execute robot motion commands.

         The function blocks until the specified command is completely executed.

         The commands are executed with the help of Moveit.

        :note:
            While :py:meth:`move` is running no further calls to :py:meth:`move` are allowed.

        :param cmd: The robot motion command which has to be executed. The following commands are currently supported:

            * :py:class:`.Ptp`
            * :py:class:`.Lin`
            * :py:class:`.Circ`
            * :py:class:`.Sequence`
            * :py:class:`.Gripper`

        :raises RobotUnknownCommandType: if an unsupported command is passed to the function.
        :raises RobotMoveAlreadyRunningError: if a move command is already running.
        :raises RobotMoveFailed: if the execution of a move command fails.
            Due to the exception any thread or script calling the :py:meth:`move` function
            ends immediately. An exception is thrown instead of returning an error
            to ensure that no further robot motion commands are executed.
        """
        # Check command type
        if not isinstance(cmd, _AbstractCmd):
            rospy.logerr("Unknown command type.")
            raise RobotUnknownCommandType("Unknown command type.")

        # Check that move is not called by multiple threads in parallel.
        if not self._move_lock.acquire(False):
            raise RobotMoveAlreadyRunningError("Parallel calls to move are note allowed.")

        rospy.loginfo("Move: " + cmd.__class__.__name__)
        rospy.logdebug("Move: " + str(cmd))

        # automatic transition from STOP_REQUESTED to NO_REQUEST when move is called
        if self._move_ctrl_sm.state == _MoveControlState.STOP_REQUESTED:
            self._move_ctrl_sm.switch(MoveControlAction.MOTION_STOPPED)

        # automatic transition from RESUME_REQUESTED to NO_REQUEST when move is called
        if self._move_ctrl_sm.state == _MoveControlState.RESUME_REQUESTED:
            self._move_ctrl_sm.switch(MoveControlAction.MOTION_RESUMED)

        try:
            self._move_execution_loop(cmd)
        finally:
            self._move_lock.release()

    def stop(self):
        """The stop function allows the user to cancel the currently running robot motion command and . This is also
        true for a paused command.

        :note:
            Function calls to :py:meth:`move` and :py:meth:`stop` have to be performed from different threads because
            :py:meth:`move` blocks the calling thread.
            The move-thread is terminated. If no motion command is active, the stop-thread is terminated.
        """
        rospy.loginfo("Stop called.")
        self._move_ctrl_sm.switch(MoveControlAction.STOP)

        with self._move_ctrl_sm:  # wait, if _execute is just starting a send_goal()
            actionclient_state = self._sequence_client.get_state()
            if actionclient_state in _VALID_GOAL_STATUS_FOR_CANCEL:
                self._sequence_client.cancel_goal()

    def pause(self):
        """The pause function allows the user to stop the currently running robot motion command. The :py:meth:`move`
        function then waits for resume. The motion can still be canceled using :py:meth:`stop`.

        :note:
            Function calls to :py:meth:`move` and :py:meth:`pause` have to be performed from different threads because
            :py:meth:`move` blocks the calling thread.
        """
        rospy.loginfo("Pause called.")
        self._move_ctrl_sm.switch(MoveControlAction.PAUSE)

        with self._move_ctrl_sm:  # wait, if _execute is just starting a send_goal()
            actionclient_state = self._sequence_client.get_state()
            if actionclient_state in _VALID_GOAL_STATUS_FOR_CANCEL:
                self._sequence_client.cancel_goal()

    def resume(self):
        """The function resumes a paused robot motion.
        If the motion command is not paused or no motion command is active,
        it has no effects.

        :note:
            Function calls to :py:meth:`move` and :py:meth:`resume` have to be performed from different threads because
            :py:meth:`move` blocks the calling thread.
        """
        rospy.loginfo("Resume called.")
        self._move_ctrl_sm.switch(MoveControlAction.RESUME)

    def is_brake_test_required(self):
        """Checks whether a brake test is currently required.

        :raises ServiceException: if it cannot determined if a braketest is needed.
        :returns: `True`: if brake test is required,
            `False`: otherwise


        :note:
            Function blocks until an answer is available.
        """
        rospy.loginfo("Checking whether brake test is required ...")
        try:
            rospy.wait_for_service(self._BRAKE_TEST_REQUIRED_SRV, self._SERVICE_WAIT_TIMEOUT_S)
            is_brake_test_required_client = rospy.ServiceProxy(
                self._BRAKE_TEST_REQUIRED_SRV,
                IsBrakeTestRequired)
            resp = is_brake_test_required_client()
            if resp.result.value == IsBrakeTestRequiredResult.REQUIRED:
                rospy.loginfo("Brake Test REQUIRED")
                return True
            elif resp.result.value == IsBrakeTestRequiredResult.NOT_REQUIRED:
                rospy.loginfo("Brake Test NOT REQUIRED")
                return False
            elif resp.result.value == IsBrakeTestRequiredResult.UNKNOWN:
                rospy.logerr("Failure during call of braketest required service: BrakeTestRequirementStatus UNKNOWN")
                raise rospy.ROSException("Could not determine if braketest is required.")
        except rospy.ROSException as e:
            rospy.logerr("Failure during call of braketest required service: {0}".format(e))
            raise e

    def execute_brake_test(self):
        """Execute a brake test. If successful, function exits without exception.

        :raises RobotBrakeTestException: when brake test was not successful.
            Will contain information about reason for failing of the brake test.
        :raises ServiceException: when the required ROS service is not available.

        :note:
            Function blocks until brake test is finished.
        """
        rospy.loginfo("Executing brake test")

        execute_brake_test_client = self._get_execute_brake_test_service()

        try:
            resp = execute_brake_test_client()
        except rospy.ROSException as e:
            rospy.logerr("Failure during call of braketest execute service: {0}".format(e))
            raise e

        rospy.loginfo("Brake Test Success: {0:b}, msg: {1}".format(
            resp.success,
            resp.error_msg
        ))
        if not resp.success:
            e = RobotBrakeTestException(resp.error_msg)
            rospy.logerr("Brake Test returned: " + str(e))
            raise e

    def _get_execute_brake_test_service(self):
        try:
            rospy.wait_for_service(self._BRAKE_TEST_EXECUTE_SRV, self._SERVICE_WAIT_TIMEOUT_S)
        except rospy.ROSException as e:
            rospy.logerr(
                "Unsuccessful waited for braketest execute service to come up: {0}".format(e))
            raise e

        execute_brake_test_client = rospy.ServiceProxy(
            self._BRAKE_TEST_EXECUTE_SRV,
            BrakeTest
        )

        return execute_brake_test_client

    def _move_execution_loop(self, cmd):
        continue_execution_of_cmd = True
        first_iteration_flag = True

        while continue_execution_of_cmd:
            rospy.logdebug("Move execution loop.")

            # execute
            if ((self._move_ctrl_sm.state == _MoveControlState.NO_REQUEST and first_iteration_flag) or
                self._move_ctrl_sm.state == _MoveControlState.RESUME_REQUESTED) and \
                    continue_execution_of_cmd:
                rospy.logdebug("start execute")

                # automatic switch to no request
                if self._move_ctrl_sm.state == _MoveControlState.RESUME_REQUESTED:
                    self._move_ctrl_sm.switch(MoveControlAction.MOTION_RESUMED)

                execution_result = cmd._execute(self)

                # evaluate the result of execute
                # motion preempt
                if execution_result == Robot._STOPPED:
                    # need to wait for resume, or execute the motion again
                    if self._move_ctrl_sm.state == _MoveControlState.PAUSE_REQUESTED \
                            or self._move_ctrl_sm.state == _MoveControlState.RESUME_REQUESTED:
                        # For now we make an exception for Sequence commands because
                        # right now we are not able to properly handle pause requests during
                        # blending.
                        if isinstance(cmd, Sequence):
                            rospy.logerr("Pause not implemented for sequence yet")
                            raise RobotMoveFailed("Pause not implemented for sequence yet")
                    # external stop
                    elif self._move_ctrl_sm.state == _MoveControlState.NO_REQUEST:
                        rospy.logerr("External stop of move command")
                        raise RobotMoveFailed("External stop of move command")
                    # normal stop
                    else:
                        rospy.logerr("Execution of move command is stopped")
                        raise RobotMoveFailed("Execution of move command is stopped")
                # motion succeeded
                elif execution_result == Robot._SUCCESS:
                    continue_execution_of_cmd = False
                # motion failed
                else:
                    rospy.logerr("Failure during execution of: " + str(cmd))
                    raise RobotMoveFailed("Failure during execution of: " + str(cmd))

            # pause
            if self._move_ctrl_sm.state == _MoveControlState.PAUSE_REQUESTED:
                rospy.loginfo("start wait for resume")
                self._move_ctrl_sm.wait_for_resume()

            # stop
            if self._move_ctrl_sm.state == _MoveControlState.STOP_REQUESTED:
                rospy.logerr("Execution of move command is stopped")
                raise RobotMoveFailed("Execution of move command is stopped")

            first_iteration_flag = False

    def _on_shutdown(self):
        if self.__robot_commander is not None:
            del self.__robot_commander
        with self._move_ctrl_sm:  # wait, if _execute is just starting a send_goal()
            actionclient_state = self._sequence_client.get_state()
        # stop movement
        if actionclient_state != GoalStatus.LOST:  # is the client currently tracking a goal?
            self._sequence_client.cancel_goal()
            self._sequence_client.wait_for_result(timeout=rospy.Duration(2.))

    def _pause_service_callback(self, request):
        self.pause()
        return [True, "success"]

    def _resume_service_callback(self, request):
        self.resume()
        return [True, "success"]

    def _stop_service_callback(self, request):
        self.stop()
        return [True, "success"]

    def _map_error_code(self, moveit_error_code):
        """Maps the given Moveit error code to API specific return values."""
        if moveit_error_code.val == MoveItErrorCodes.SUCCESS:
            return self._SUCCESS
        elif moveit_error_code.val == MoveItErrorCodes.PREEMPTED:
            return self._STOPPED
        else:
            return self._FAILURE

    @staticmethod
    def _check_version(version):
        # check if version is set by user
        if version is None:
            rospy.logerr("Version of Robot API is not set!")
            raise RobotVersionError("Version of Robot API is not set!"
                                    "Current installed version is " + __version__ + "!")

        # check given version is correct
        if version != __version__.split(".")[0]:
            rospy.logerr("Version of Robot API does not match!")
            raise RobotVersionError("Version of Robot API does not match! "
                                    "Current installed version is " + __version__ + "!")

    def _claim_single_instance(self):
        # check if we are the single instance
        if self._check_single_instance():
            # If no other instance exists, the pid and create_time is stored (overwrites old one)
            self._single_instance_flag = True
            process = psutil.Process()
            rospy.set_param(self._INSTANCE_PARAM, {self._PID_STRING: process.pid,
                                                   self._PROCESS_CREATE_TIME_STRING: process.create_time()})
        else:
            raise RobotMultiInstancesError("Only one instance of Robot class can be created!")

    def _check_single_instance(self):
        # return True if no other instance exists
        # If running the same program twice the second should kill the first, however the parameter server
        # has a small delay so we check twice for the single instance flag.
        if rospy.has_param(self._INSTANCE_PARAM):
            time.sleep(1)

        if rospy.has_param(self._INSTANCE_PARAM):
            instance = rospy.get_param(self._INSTANCE_PARAM)
            pid = instance[self._PID_STRING]
            create_time = instance[self._PROCESS_CREATE_TIME_STRING]

            if psutil.pid_exists(pid):
                process = psutil.Process(pid)

                if process.create_time() == create_time:
                    rospy.logerr("An instance of Robot class already exists (pid=" + str(pid) + ").")
                    return False

        return True

    def _establish_connections(self):
        # Create sequence_move_group client, only for manipulator
        self._sequence_client = SimpleActionClient(self._SEQUENCE_TOPIC, MoveGroupSequenceAction)
        rospy.loginfo("Waiting for connection to action server " + self._SEQUENCE_TOPIC + "...")
        self._sequence_client.wait_for_server()
        rospy.logdebug("Connection to action server " + self._SEQUENCE_TOPIC + " established.")

        # Start ROS Services which allow to pause, resume and stop movements
        self._pause_service = rospy.Service(
            Robot._PAUSE_TOPIC_NAME, Trigger, self._pause_service_callback)
        self._resume_service = rospy.Service(
            Robot._RESUME_TOPIC_NAME, Trigger, self._resume_service_callback)
        self._stop_service = rospy.Service(
            Robot._STOP_TOPIC_NAME, Trigger, self._stop_service_callback)

        # Connect to speed override service
        rospy.loginfo("Waiting for connection to service " + self._GET_SPEED_OVERRIDE_SRV + "...")
        rospy.wait_for_service(self._GET_SPEED_OVERRIDE_SRV, self._SERVICE_WAIT_TIMEOUT_S)
        rospy.loginfo("Connection to service " + self._GET_SPEED_OVERRIDE_SRV + " established")
        self._get_speed_override_srv = rospy.ServiceProxy(self._GET_SPEED_OVERRIDE_SRV, GetSpeedOverride)

    def _release(self):
        rospy.logdebug("Release called")
        try:
            self._pause_service.shutdown(reason="Robot instance released.")
            self._resume_service.shutdown(reason="Robot instance released.")
            self._stop_service.shutdown(reason="Robot instance released.")
        except AttributeError:
            rospy.logdebug("Services do not exists yet or have already been shutdown.")
        # do not delete pid parameter if it has not been set or overwritten
        if self._single_instance_flag:
            rospy.logdebug("Delete single instance parameter from parameter server.")
            try:
                rospy.delete_param(self._INSTANCE_PARAM)
            except KeyError:
                rospy.logdebug("Instance parameter was missing.")
            self._single_instace_flag = False

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.__del__()

    def __del__(self):
        rospy.logdebug("Dtor called")
        self._release()
