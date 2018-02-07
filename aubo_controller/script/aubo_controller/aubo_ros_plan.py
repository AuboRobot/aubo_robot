#!/usr/bin/env python

# Software License Agreement (BSD License)
# Copyright (c) 2017-2018, Aubo Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.



import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import Float32MultiArray

from std_msgs.msg import String

global planType
global planning

global pose_target
global group_variable_values

def plancallback(msg_in):
   planning = True
   planType = msg_in.data[0]
   if planType == 1.0:
     pose_target
     group.set_pose_target(pose_target)
     plan = group.plan()
     display_trajectory = moveit_msgs.msg.DisplayTrajectory()
     display_trajectory.trajectory_start = robot.get_current_state()
     display_trajectory.trajectory.append(plan)
     display_trajectory_publisher.publish(display_trajectory)
     success = group.go(wait=True)
   elif (0.0 == planType): ## Planning to a joint-space goal
      rospy.loginfo('Planning to a joint-space goal')
      group.clear_pose_targets()
      group_variable_values = group.get_current_joint_values()
      group_variable_values[0] = msg_in.data[1]
      group_variable_values[1] = msg_in.data[2]
      group_variable_values[2] = msg_in.data[3]
      group_variable_values[3] = msg_in.data[4]
      group_variable_values[4] = msg_in.data[5]
      group_variable_values[5] = msg_in.data[6]
      group.set_joint_value_target(group_variable_values)
      plan = group.plan()
      rospy.loginfo('move to a joint-space goal')
      success = group.go(wait=True)
      rospy.loginfo('success!!!')
   planning = False


def main():

  planType = -1
  planning = False

  pose_target = geometry_msgs.msg.Pose()
  rate = rospy.Rate(100) #10hz
  group_variable_values = group.get_current_joint_values()
  while not rospy.is_shutdown():
    # if(0 == planType):     # Planning to a Pose goal
    #   planning = True
    #   planType = -1
    #   group.set_pose_target(pose_target)
    #   plan = group.plan()
    #   display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    #   display_trajectory.trajectory_start = robot.get_current_state()
    #   display_trajectory.trajectory.append(plan)
    #   display_trajectory_publisher.publish(display_trajectory)
    #
    #   success = group.go(wait=True)
    #   planning = False
    # elif (1 == planType): ## Planning to a joint-space goal
    #   group.clear_pose_targets()
    #   # group_variable_values = group.get_current_joint_values()
    #   group.set_joint_value_target(group_variable_values)
    #   plan = group.plan()
    #   success = group.go(wait=True)
    #   planning = False

    rate.sleep()


  ## Cartesian Paths
  ## You can plan a cartesian path directly by specifying a list of waypoints
  ## for the end-effector to go through.
  waypoints = []

  # start with the current pose
  waypoints.append(group.get_current_pose().pose)

  # first orient gripper and move forward (+x)
  wpose = geometry_msgs.msg.Pose()
  wpose.orientation.w = 1.0
  wpose.position.x = waypoints[0].position.x + 0.1
  wpose.position.y = waypoints[0].position.y
  wpose.position.z = waypoints[0].position.z
  waypoints.append(copy.deepcopy(wpose))

  # second move down
  wpose.position.z -= 0.10
  waypoints.append(copy.deepcopy(wpose))

  # third move to the side
  wpose.position.y += 0.05
  waypoints.append(copy.deepcopy(wpose))

  ## We want the cartesian path to be interpolated at a resolution of 1 cm
  ## which is why we will specify 0.01 as the eef_step in cartesian
  ## translation.  We will specify the jump threshold as 0.0, effectively
  ## disabling it.
  (plan3, fraction) = group.compute_cartesian_path(
                               waypoints,   # waypoints to follow
                               0.01,        # eef_step
                               0.0)         # jump_threshold

  print "============ Waiting while RVIZ displays plan3..."
  rospy.sleep(5)


  ## Adding/Removing Objects and Attaching/Detaching Objects
  ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  ## First, we will define the collision object message
  collision_object = moveit_msgs.msg.CollisionObject()



  ## When finished shut down moveit_commander.
  moveit_commander.roscpp_shutdown()

  ## END_TUTORIAL

  print "============ STOPPING"


# print "============ Waiting for RVIZ..."
rospy.sleep(2)
print "============ Starting tutorial "
## First initialize moveit_commander and rospy.
print "============ Starting tutorial setup"
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('aubo_ros_plan', anonymous=True)
rospy.Subscriber("moveit_target", Float32MultiArray, plancallback)

## Instantiate a RobotCommander object.  This object is an interface to the robot as a whole.
robot = moveit_commander.RobotCommander()

## Instantiate a PlanningSceneInterface object.  This object is an interface
## to the world surrounding the robot.
scene = moveit_commander.PlanningSceneInterface()

## Instantiate a MoveGroupCommander object.  This object is an interface  to one group of joints.  In this case the group is the joints in the left
## arm.  This interface can be used to plan and execute motions on the left arm.
group = moveit_commander.MoveGroupCommander("manipulator")


## We create this DisplayTrajectory publisher which is used below to publish trajectories for RVIZ to visualize.
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)

## We can get the name of the reference frame for this robot
# print "============ Reference frame: %s" % group.get_planning_frame()

## We can also print the name of the end-effector link for this group
# print "============ Reference frame: %s" % group.get_end_effector_link()

## We can get a list of all the groups in the robot
print "============ Robot Groups:"
print robot.get_group_names()

## Sometimes for debugging it is useful to print the entire state of the
## robot.
print "============ Printing robot state"
print robot.get_current_state()


if __name__ == '__main__': main()

