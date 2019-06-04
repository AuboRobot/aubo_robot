/* Author: Anirudh, Akash
Priting current end effector pose w.r.t world frame */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/collision_detection/collision_matrix.h>
#include <geometric_shapes/shape_operations.h>

#include <ros/console.h>
#include <ros/console.h>

#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include "tf/LinearMath/Transform.h"

#include "octomap_msgs/conversions.h"
#include "geometry_msgs/Quaternion.h"
#include <tf/transform_datatypes.h>
#include <moveit/robot_state/conversions.h>
#include <unistd.h>

#define EXECUTION_ITERATIONS                  (1)

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_movement_with_constraint");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    ros::console::notifyLoggerLevelsChanged();

    static const std::string PLANNING_GROUP = "move_arm";     //might vary for different arm models.manipulator
    ROS_INFO_NAMED("arm_movement_with_constraint","STARTING MOVEMENT SCRIPT");

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    ros::Publisher traj_publisher = node_handle.advertise<trajectory_msgs::JointTrajectory>("/trajectory_visualizer", 1000);


    ROS_INFO("Reference frame: %s", move_group.getPlanningFrame().c_str());

    ROS_INFO("End effector link: %s", move_group.getEndEffectorLink().c_str());

    move_group.setPlannerId("PRMstarkConfigDefault");  // DO not change this.

    move_group.setPlanningTime(3);

    move_group.setMaxVelocityScalingFactor(0.3);
    move_group.setMaxAccelerationScalingFactor(1);

    move_group.setPoseReferenceFrame( move_group.getPlanningFrame());

    bool success ;
    moveit_msgs::MoveItErrorCodes pogp_success;

    std::vector< double > test_point_1 ;
    test_point_1.push_back(3.045322);
    test_point_1.push_back(0.1);
    test_point_1.push_back(0.544908);
    test_point_1.push_back(-1.884432);
    test_point_1.push_back(0.749672);
    test_point_1.push_back(1.410019);
    test_point_1.push_back(0.000000);

    //Change joint angles over here as per joint angle you want to test. Angles order is starting with rail's dof and then robot's axis starting from base joint.
    std::vector< double > test_point_2 ;
    test_point_2.push_back(1.57);
    test_point_2.push_back(0.1);
    test_point_2.push_back(0.544908);
    test_point_2.push_back(-1.884432);
    test_point_2.push_back(0.749672);
    test_point_2.push_back(1.410019);
    test_point_2.push_back(0.0);

    int plan_number = 0 ;

    geometry_msgs::PoseStamped current_pose_message;
    std::vector< double > joint_vals;


    current_pose_message=move_group.getCurrentPose(move_group.getEndEffectorLink() );
    joint_vals=move_group.getCurrentJointValues();

    move_group.setGoalTolerance(0.001);
    move_group.setJointValueTarget(test_point_2);
    pogp_success = move_group.move();

    if(pogp_success.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
    {

      moveit::planning_interface::MoveGroupInterface::Plan test_plan_1 ;
      moveit::planning_interface::MoveGroupInterface::Plan test_plan_2 ;

      while(ros::ok() || (plan_number < EXECUTION_ITERATIONS))
      {
        traj_publisher.publish(test_plan_1.trajectory_.joint_trajectory);

        usleep(1000*300);

        robot_state::RobotState ds(*move_group.getCurrentState());

        moveit_msgs::RobotState dummy_state;
        robotStateToRobotStateMsg(ds,dummy_state,true);
        dummy_state.is_diff=true;
        move_group.setStartState(dummy_state);

        move_group.setGoalTolerance(0.001);
        move_group.setJointValueTarget(test_point_1);
        pogp_success = move_group.plan(test_plan_1);
        pogp_success = move_group.execute(test_plan_1);

        if(pogp_success.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
        {
          ROS_INFO("Execution success for plan 1!!!!");

          usleep(1000*300);

          robot_state::RobotState ds(*move_group.getCurrentState());

          moveit_msgs::RobotState dummy_state;
          robotStateToRobotStateMsg(ds,dummy_state,true);
          dummy_state.is_diff=true;
          move_group.setStartState(dummy_state);

          move_group.setGoalTolerance(0.001);
          move_group.setJointValueTarget(test_point_2);
          pogp_success = move_group.plan(test_plan_2);
          pogp_success = move_group.execute(test_plan_2);
          if(pogp_success.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
          {
            ROS_INFO("Execution success for plan 2!!!!");
            plan_number += 1 ;
          }
          else
          {
            ROS_ERROR("Execution failed for plan 2!!!");
            break ;
          }
        }
        else
        {
          ROS_ERROR("Execution failed for plan 1!!!");
          break ;
        }
      }
    }
    else
    {
      ROS_ERROR("Plan failed for plan 2!!!");
    }

  ros::shutdown();
  return 0;

}
