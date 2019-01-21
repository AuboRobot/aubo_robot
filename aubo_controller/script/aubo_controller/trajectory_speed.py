#!/usr/bin/env python

# Software License Agreement (BSD License)
# Copyright (c) 2017-2018, Aubo Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#  * Neither the name of the Southwest Research Institute, nor the names
#    of its contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from copy import deepcopy

def scale_trajectory_speed(traj, scale):
    # Create a new trajectory object
    new_traj = JointTrajectory()

    # Get the number of joints involved
    n_joints = len(traj.joint_names)

    # Get the number of points on the trajectory
    n_points = len(traj.points)

    # Store the trajectory points
    points = list(traj.points)

    # Cycle through all points and joints and scale the time from start, as well as joint speed and acceleration
    for i in range(n_points):
        point = JointTrajectoryPoint()

        # The joint positions are not scaled so pull them out first
        point.positions = traj.points[i].positions

        # Next, scale the time_from_start for this point
        point.time_from_start = traj.points[i].time_from_start / scale

        # Get the joint velocities for this point
        point.velocities = list(traj.points[i].velocities)

        # Get the joint accelerations for this point
        point.accelerations = list(traj.points[i].accelerations)

        # Scale the velocity and acceleration for each joint at this point
        for j in range(n_joints):
            point.velocities[j] = point.velocities[j] * scale
            point.accelerations[j] = point.accelerations[j] * scale * scale

        # Store the scaled trajectory point
        points[i] = point

    # Assign the modified points to the new trajectory
    new_traj.points = points

    # Return the new trajecotry
    return new_traj

def set_trajectory_speed(traj, speed):
    # Create a new trajectory object
    new_traj = RobotTrajectory()

    # Initialize the new trajectory to be the same as the input trajectory
    new_traj.joint_trajectory = traj.joint_trajectory

    # Get the number of joints involved
    n_joints = len(traj.joint_trajectory.joint_names)

    # Get the number of points on the trajectory
    n_points = len(traj.joint_trajectory.points)

    # Store the trajectory points
    points = list(traj.joint_trajectory.points)

    # Cycle through all points and joints and scale the time from start, as well as joint speed and acceleration
    for i in range(n_points):
        point = JointTrajectoryPoint()

        # The joint positions are not scaled so pull them out first
        point.positions = traj.joint_trajectory.points[i].positions

        # Next, scale the time_from_start for this point
        point.time_from_start = traj.joint_trajectory.points[i].time_from_start / speed

        # Initialize the joint velocities for this point
        point.velocities = [speed] * n_joints

        # Get the joint accelerations for this point
        point.accelerations = [speed / 4.0] * n_joints

        # Store the scaled trajectory point
        points[i] = point

    # Assign the modified points to the new trajectory
    new_traj.joint_trajectory.points = points

    # Return the new trajecotry
    return new_traj

def create_tracking_trajectory(traj, speed, min_speed):
    # Create a new trajectory object
    new_traj = RobotTrajectory()

    # Initialize the new trajectory to be the same as the input trajectory
    new_traj.joint_trajectory = deepcopy(traj.joint_trajectory)

    # Get the number of joints involved
    n_joints = len(traj.joint_trajectory.joint_names)

    # Get the number of points on the trajectory
    n_points = len(traj.joint_trajectory.points)

    # Cycle through all points and joints and scale the time from start,
    # as well as joint speed and acceleration
    for i in range(n_points):
        # The joint positions are not scaled so pull them out first
        new_traj.joint_trajectory.points[i].positions = traj.joint_trajectory.points[i].positions

        # Next, scale the time_from_start for this point
        new_traj.joint_trajectory.points[i].time_from_start = traj.joint_trajectory.points[i].time_from_start / speed

        # Initialize the joint velocities for this point
        new_traj.joint_trajectory.points[i].velocities = [speed] * n_joints + [min_speed] * n_joints

        # Get the joint accelerations for this point
        new_traj.joint_trajectory.points[i].accelerations = [speed / 4.0] * n_joints

    # Return the new trajecotry
    return new_traj