/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017-2018, AUBO Robotics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "joint_trajectory_action.h"
#include "utils.h"
#include "utils_.h"
#include "param_utils.h"
#include "utils.h"
#include <map>

namespace industrial_robot_client
{
namespace joint_trajectory_action
{
const double JointTrajectoryAction::WATCHDOG_PERIOD_ = 1.0;
const double JointTrajectoryAction::DEFAULT_GOAL_THRESHOLD_ = 0.002;

JointTrajectoryAction::JointTrajectoryAction(std::string controller_name) :
    action_server_(node_, controller_name, boost::bind(&JointTrajectoryAction::goalCB, this, _1),
                   boost::bind(&JointTrajectoryAction::cancelCB, this, _1), false), has_active_goal_(false),
                       controller_alive_(false), has_moved_once_(false)
{
  ros::NodeHandle pn("~");

  pn.param("constraints/goal_threshold", goal_threshold_, DEFAULT_GOAL_THRESHOLD_);

  if (!industrial_utils::param::getJointNames("controller_joint_names", "robot_description", joint_names_))
    ROS_ERROR("Failed to initialize joint_names.");

  // The controller joint names parameter includes empty joint names for those joints not supported
  // by the controller.  These are removed since the trajectory action should ignore these.
  std::remove(joint_names_.begin(), joint_names_.end(), std::string());
  ROS_INFO_STREAM("Filtered joint names to " << joint_names_.size() << " joints");

  pub_trajectory_command_ = node_.advertise<trajectory_msgs::JointTrajectory>("joint_path_command", 100);
  sub_trajectory_state_ = node_.subscribe("feedback_states", 1, &JointTrajectoryAction::controllerStateCB, this);
  sub_robot_status_ = node_.subscribe("robot_status", 1, &JointTrajectoryAction::robotStatusCB, this);
  trajectory_execution_subs_ = node_.subscribe("trajectory_execution_event", 1, &JointTrajectoryAction::trajectoryExecutionCallback,this);


  watchdog_timer_ = node_.createTimer(ros::Duration(WATCHDOG_PERIOD_), &JointTrajectoryAction::watchdog, this, true);
  action_server_.start();
}

JointTrajectoryAction::~JointTrajectoryAction()
{
}

void JointTrajectoryAction::trajectoryExecutionCallback(const std_msgs::String::ConstPtr &msg)
{
    if(msg->data == "stop")
    {
        ROS_INFO("trajectory execution status: stop1");
        // Marks the current goal as canceled.

        active_goal_.setAborted();
        has_active_goal_ = false;
    }
}

void JointTrajectoryAction::robotStatusCB(const industrial_msgs::RobotStatusConstPtr &msg)
{
  last_robot_status_ = msg; //caching robot status for later use.
  has_moved_once_ = has_moved_once_ ? true : (last_robot_status_->in_motion.val == industrial_msgs::TriState::TRUE);

  if (msg->e_stopped.val == 1 /*isEmergencyStopped()*/|| msg->in_error.val == 1 /*isProtectiveStopped()*/)
  {
      if (msg->e_stopped.val == 1)
      {
          ROS_INFO("Emergency stop pressed!");
      }
      else
      {
           ROS_INFO("Robot is protective stopped!");
      }
      if(has_active_goal_)
      {
          ROS_INFO("Aborting trajectory");
          active_goal_.setAborted();
          has_active_goal_ = false;
      }
  }
}

void JointTrajectoryAction::watchdog(const ros::TimerEvent &e)
{
  // Some debug logging
  if (!last_trajectory_state_)
  {
    ROS_DEBUG("Waiting for subscription to joint trajectory state");
  }

  ROS_WARN("Trajectory state not received for %f seconds", WATCHDOG_PERIOD_);
  controller_alive_ = false;

  // Aborts the active goal if the controller does not appear to be active.
  if (has_active_goal_)
  {
    // last_trajectory_state_ is null if the subscriber never makes a connection
    if (!last_trajectory_state_)
    {
      ROS_WARN("Aborting goal because we have never heard a controller state message.");
    }
    else
    {
      ROS_WARN_STREAM("Aborting goal because we haven't heard from the controller in " << WATCHDOG_PERIOD_ << " seconds");
    }
    abortGoal();
  }
}

void JointTrajectoryAction::goalCB(const JointTractoryActionServer::GoalHandle &gh)
{
  ROS_INFO("Received new goal");
  JointTractoryActionServer::GoalHandle tmp_gh = const_cast<JointTractoryActionServer::GoalHandle&>(gh);

  // reject all goals as long as we haven't heard from the remote controller
  if (!controller_alive_)
  {
    ROS_ERROR("Joint trajectory action rejected: waiting for (initial) feedback from controller");
    control_msgs::FollowJointTrajectoryResult rslt;
    rslt.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
    tmp_gh.setRejected(rslt, "Waiting for (initial) feedback from controller");

    // no point in continuing: already rejected
    return;
  }

  if (!tmp_gh.getGoal()->trajectory.points.empty())
  {
    if (industrial_utils::isSimilar(joint_names_, tmp_gh.getGoal()->trajectory.joint_names))
    {

      // Cancels the currently active goal.
      if (has_active_goal_)
      {
        ROS_WARN("Received new goal, canceling current goal");
        abortGoal();
      }

      tmp_gh.setAccepted();
      active_goal_ = tmp_gh;
      has_active_goal_ = true;
      time_to_check_ = ros::Time::now() +
          ros::Duration(active_goal_.getGoal()->trajectory.points.back().time_from_start.toSec() / 2.0);
      has_moved_once_ = false;

      ROS_INFO("Publishing trajectory");

      current_traj_ = active_goal_.getGoal()->trajectory;
      pub_trajectory_command_.publish(current_traj_);

    }
    else
    {
      ROS_ERROR("Joint trajectory action failing on invalid joints");
      control_msgs::FollowJointTrajectoryResult rslt;
      rslt.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
      tmp_gh.setRejected(rslt, "Joint names do not match");
    }
  }
  else
  {
    ROS_ERROR("Joint trajectory action failed on empty trajectory");
    control_msgs::FollowJointTrajectoryResult rslt;
    rslt.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
    tmp_gh.setRejected(rslt, "Empty trajectory");
  }

  // Adding some informational log messages to indicate unsupported goal constraints
  if (tmp_gh.getGoal()->goal_time_tolerance.toSec() > 0.0)
  {
    ROS_WARN_STREAM("Ignoring goal time tolerance in action goal, may be supported in the future");
  }
  if (!tmp_gh.getGoal()->goal_tolerance.empty())
  {
    ROS_WARN_STREAM(
        "Ignoring goal tolerance in action, using paramater tolerance of " << goal_threshold_ << " instead");
  }
  if (!tmp_gh.getGoal()->path_tolerance.empty())
  {
    ROS_WARN_STREAM("Ignoring goal path tolerance, option not supported by ROS-Industrial drivers");
  }
}

void JointTrajectoryAction::cancelCB(const JointTractoryActionServer::GoalHandle &gh)
{
  JointTractoryActionServer::GoalHandle tmp_gh = const_cast<JointTractoryActionServer::GoalHandle&>(gh);
  ROS_DEBUG("Received action cancel request");
  if (active_goal_ == gh)
  {
    // Stops the controller.
    trajectory_msgs::JointTrajectory empty;
    empty.joint_names = joint_names_;
    pub_trajectory_command_.publish(empty);

    // Marks the current goal as canceled.
    tmp_gh.setCanceled();
    has_active_goal_ = false;
  }
  else
  {
    ROS_WARN("Active goal and goal cancel do not match, ignoring cancel request");
  }
}

void JointTrajectoryAction::controllerStateCB(const control_msgs::FollowJointTrajectoryFeedbackConstPtr &msg)
{
  ROS_DEBUG("Checking controller state feedback");
  last_trajectory_state_ = msg;
  controller_alive_ = true;

  watchdog_timer_.stop();
  watchdog_timer_.start();

  if (!has_active_goal_)
  {
    //ROS_DEBUG("No active goal, ignoring feedback");
    return;
  }
  if (current_traj_.points.empty())
  {
    ROS_DEBUG("Current trajectory is empty, ignoring feedback");
    return;
  }

  if (!industrial_utils::isSimilar(joint_names_, msg->joint_names))
  {
    ROS_ERROR("Joint names from the controller don't match our joint names.");
    return;
  }

  if (!has_moved_once_ && (ros::Time::now() < time_to_check_))
  {
    ROS_DEBUG("Waiting to check for goal completion until halfway through trajectory");
    return;
  }

  // Checking for goal constraints
  // Checks that we have ended inside the goal constraints and has motion stopped

  ROS_DEBUG("Checking goal constraints");
  if (withinGoalConstraints(last_trajectory_state_, current_traj_))
  {
    if (last_robot_status_)
    {
      // Additional check for motion stoppage since the controller goal may still
      // be moving.  The current robot driver calls a motion stop if it receives
      // a new trajectory while it is still moving.  If the driver is not publishing
      // the motion state (i.e. old driver), this will still work, but it warns you.
      if (last_robot_status_->in_motion.val == industrial_msgs::TriState::FALSE)
      {
        ROS_INFO("Inside goal constraints, stopped moving, return success for action");
        active_goal_.setSucceeded();
        has_active_goal_ = false;
      }
      else if (last_robot_status_->in_motion.val == industrial_msgs::TriState::UNKNOWN)
      {
        ROS_INFO("Inside goal constraints, return success for action");
        ROS_WARN("Robot status in motion unknown, the robot driver node and controller code should be updated");
        active_goal_.setSucceeded();
        has_active_goal_ = false;
      }
      else
      {
        ROS_INFO("Within goal constraints but robot is still moving");
      }
    }
    else
    {
      ROS_INFO("Inside goal constraints, return success for action");
      ROS_WARN("Robot status is not being published the robot driver node and controller code should be updated");
      active_goal_.setSucceeded();
      has_active_goal_ = false;
    }
  }
}

void JointTrajectoryAction::abortGoal()
{
  // Stops the controller.
  trajectory_msgs::JointTrajectory empty;
//  pub_trajectory_command_.publish(empty);

  // Marks the current goal as aborted.
  active_goal_.setAborted();
  has_active_goal_ = false;
}

bool mapInsert(const std::string & key, double value, std::map<std::string, double> & mappings)
{
  bool rtn = false;

  std::pair<std::map<std::string, double>::iterator, bool> insert_rtn;

  insert_rtn = mappings.insert(std::make_pair(key, value));

  // The second value returned form insert is a boolean (true for success)
  if (!insert_rtn.second)
  {
    ROS_ERROR_STREAM(__FUNCTION__ << "::Failed to insert item into map with key: " << key);
    rtn = false;
  }
  else
  {
    rtn = true;
  }
  return rtn;

}


bool toMap(const std::vector<std::string> & keys, const std::vector<double> & values,
           std::map<std::string, double> & mappings)
{
  bool rtn;

  mappings.clear();

  if (keys.size() == values.size())
  {
    rtn = true;

    for (size_t i = 0; i < keys.size(); ++i)
    {
      rtn = mapInsert(keys[i], values[i], mappings);
      if (!rtn)
      {
        break;
      }
    }

  }
  else
  {
    ROS_ERROR_STREAM(__FUNCTION__ << "::keys size: " << keys.size()
                     << " does not match values size: " << values.size());

    rtn = false;
  }

  return rtn;
}

bool isWithinRange(const std::vector<std::string> & keys, const std::map<std::string, double> & lhs,
                   const std::map<std::string, double> & rhs, double full_range)
{
  bool rtn = false;

  if ((keys.size() != rhs.size()) || (keys.size() != lhs.size()))
  {
    ROS_ERROR_STREAM(__FUNCTION__ << "::Size mistmatch ::lhs size: " << lhs.size() <<
                     " rhs size: " << rhs.size() << " key size: " << keys.size());

    rtn = false;
  }
  else
  {
    // Calculating the half range causes some precision loss, but it's good enough
    double half_range = full_range / 2.0;
    rtn = true; // Assume within range, catch exception in loop below

    // This loop will not run for empty vectors, results in return of true
    for (size_t i = 0; i < keys.size(); ++i)
    {
      if (fabs(lhs.at(keys[i]) - rhs.at(keys[i])) > fabs(half_range))
      {
        rtn = false;
        break;
      }
    }

  }

  return rtn;
}


bool isWithinRange(const std::vector<std::string> & lhs_keys, const std::vector<double> & lhs_values,
                   const std::vector<std::string> & rhs_keys, const std::vector<double> & rhs_values, double full_range)
{
  bool rtn = false;
  std::map<std::string, double> lhs_map;
  std::map<std::string, double> rhs_map;
  if (industrial_utils::isSimilar(lhs_keys, rhs_keys))
  {
    if (toMap(lhs_keys, lhs_values, lhs_map) && toMap(rhs_keys, rhs_values, rhs_map))
    {
      rtn = isWithinRange(lhs_keys, lhs_map, rhs_map, full_range);
    }
  }
  else
  {
    ROS_ERROR_STREAM(__FUNCTION__ << "::Key vectors are not similar");
    rtn = false;
  }
  return rtn;
}

bool JointTrajectoryAction::withinGoalConstraints(const control_msgs::FollowJointTrajectoryFeedbackConstPtr &msg,
                                                  const trajectory_msgs::JointTrajectory & traj)
{
  bool rtn = false;
  if (traj.points.empty())
  {
    ROS_WARN("Empty joint trajectory passed to check goal constraints, return false");
    rtn = false;
  }
  else
  {
    int last_point = traj.points.size() - 1;

//    std::cout<<"last_trajectory_state"<<last_trajectory_state_->joint_names[0]<<","<<last_trajectory_state_->joint_names[1]<<","<<last_trajectory_state_->joint_names[2]
//                                                    <<","<<last_trajectory_state_->joint_names[3]<<","<<last_trajectory_state_->joint_names[4]
//                                                    <<","<<last_trajectory_state_->joint_names[5]<<","<<last_trajectory_state_->joint_names[6]<<std::endl;
//    std::cout<<"traj.joint_names"<<traj.joint_names[0]<<","<<traj.joint_names[1]<<","<<traj.joint_names[2]
//                                                    <<","<<traj.joint_names[3]<<","<<traj.joint_names[4]
//                                                    <<","<<traj.joint_names[5]<<","<<traj.joint_names[6]<<std::endl;


//    ROS_INFO("laset position,%f,%f,%f,%f,%f,%f,%f",last_trajectory_state_->actual.positions[0],last_trajectory_state_->actual.positions[1],last_trajectory_state_->actual.positions[2],last_trajectory_state_->actual.positions[3],
//            last_trajectory_state_->actual.positions[4],last_trajectory_state_->actual.positions[5],last_trajectory_state_->actual.positions[6]);

//    ROS_INFO("current position,%f,%f,%f,%f,%f,%f,%f",traj.points[last_point].positions[0],traj.points[last_point].positions[1],traj.points[last_point].positions[2],traj.points[last_point].positions[3],
//            traj.points[last_point].positions[4],traj.points[last_point].positions[5],traj.points[last_point].positions[6]);

    if (industrial_robot_client::utils::isWithinRange(last_trajectory_state_->joint_names,
                                                      last_trajectory_state_->actual.positions, traj.joint_names,
                                                      traj.points[last_point].positions, goal_threshold_))
    {
      rtn = true;
    }
    else
    {
      rtn = false;
    }
  }
  return rtn;
}

} //joint_trajectory_action
} //industrial_robot_client


/** This node should be loaded after the robot description**/
using industrial_robot_client::joint_trajectory_action::JointTrajectoryAction;
int main(int argc, char** argv)
{
  // initialize node
  ros::init(argc, argv, "aubo_joint_follow_action");

  std::string robot_name, controller_name;
  ros::param::get("/robot_name", robot_name);
  while(robot_name == "")
  {
    sleep(1);
    ROS_INFO("Waiting for the robot description to start up!");
  }
  if(robot_name == "aubo_i5")
      controller_name = "aubo_i5_controller/follow_joint_trajectory";
  else if(robot_name == "aubo_i3")
          controller_name = "aubo_i3_controller/follow_joint_trajectory";
  else if(robot_name == "aubo_i7")
          controller_name = "aubo_i7_controller/follow_joint_trajectory";
  else if(robot_name == "aubo_i10")
          controller_name = "aubo_i10_controller/follow_joint_trajectory";
  else if(robot_name == "aubo_i5l")
          controller_name = "aubo_i5l_controller/follow_joint_trajectory";
  else if(robot_name == "aubo_i16")
          controller_name = "aubo_i16_controller/follow_joint_trajectory";
  else if(robot_name == "aubo_e5")
          controller_name = "aubo_e5_controller/follow_joint_trajectory";
  else if(robot_name == "aubo_e3")
          controller_name = "aubo_e3_controller/follow_joint_trajectory";

  JointTrajectoryAction action(controller_name);
  action.run();

  return 0;
}
