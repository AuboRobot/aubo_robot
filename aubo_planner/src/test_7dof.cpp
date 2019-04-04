/* Author: Anirudh, Akash
Priting current end effector pose w.r.t world frame */

#include <moveit/move_group_interface/move_group.h>
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

#define EXECUTION_ITERATIONS                  (1)

int main(int argc, char **argv)
{
    ros::init(argc, argv, "7dof_testing");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();


    static const std::string PLANNING_GROUP = "manipulator";     //might vary for different arm models.
    ROS_INFO_NAMED("arm_movement_with_constraint","STARTING MOVEMENT SCRIPT");

    moveit::planning_interface::MoveGroup move_group(PLANNING_GROUP);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


    ros::Publisher traj_publisher = node_handle.advertise<trajectory_msgs::JointTrajectory>("/trajectory_visualizer", 1000);


    move_group.setPlannerId("PRMstarkConfigDefault");  // DO not change this.

    move_group.setPlanningTime(3);

    move_group.setMaxVelocityScalingFactor(0.3);
    move_group.setMaxAccelerationScalingFactor(1);

    move_group.setPoseReferenceFrame( move_group.getPlanningFrame());

    bool success ;
    moveit_msgs::MoveItErrorCodes pogp_success;

    std::vector< double > test_point_1 ;
    test_point_1.push_back(0.1);
    test_point_1.push_back(3.045322);
    test_point_1.push_back(0.544908);
    test_point_1.push_back(-1.884432);
    test_point_1.push_back(0.749672);
    test_point_1.push_back(1.410019);
    test_point_1.push_back(0.000000);

    std::vector< double > test_point_2 ;
    test_point_2.push_back(0.7);
    test_point_2.push_back(3.045322);
    test_point_2.push_back(0.544908);
    test_point_2.push_back(-1.884432);
    test_point_2.push_back(0.749672);
    test_point_2.push_back(1.410019);
    test_point_2.push_back(0.0);

    int plan_number = 0 ;

    move_group.setGoalTolerance(0.001);
    move_group.setJointValueTarget(test_point_2);
    pogp_success = move_group.move();
    if(pogp_success.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
      robot_state::RobotState ds(*move_group.getCurrentState());
      const double j1=test_point_2[0];
      const double j2=test_point_2[1];
      const double j3=test_point_2[2];
      const double j4=test_point_2[3];
      const double j5=test_point_2[4];
      const double j6=test_point_2[5];
      const double j7=test_point_2[6];

      ds.setJointPositions("horizontal_rail_joint",&j1);
      ds.setJointPositions("shoulder_joint",&j2);
      ds.setJointPositions("upperArm_joint",&j3);
      ds.setJointPositions("foreArm_joint",&j4);
      ds.setJointPositions("wrist1_joint",&j5);
      ds.setJointPositions("wrist2_joint",&j6);
      ds.setJointPositions("wrist3_joint",&j7);

      moveit_msgs::RobotState dummy_state;
      robotStateToRobotStateMsg(ds,dummy_state,true);
      dummy_state.is_diff=true;
      move_group.setStartState(dummy_state);

      moveit::planning_interface::MoveGroup::Plan test_plan_1 ;
      moveit::planning_interface::MoveGroup::Plan test_plan_2 ;



      move_group.setGoalTolerance(0.001);
      move_group.setJointValueTarget(test_point_1);
      pogp_success = move_group.plan(test_plan_1);
      if(pogp_success.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
      {
        ROS_INFO("Plan successfull for plan 1!!!");

        robot_state::RobotState ds(*move_group.getCurrentState());
        const double j1=test_point_1[0];
        const double j2=test_point_1[1];
        const double j3=test_point_1[2];
        const double j4=test_point_1[3];
        const double j5=test_point_1[4];
        const double j6=test_point_1[5];
        const double j7=test_point_1[6];

        ds.setJointPositions("horizontal_rail_joint",&j1);
        ds.setJointPositions("shoulder_joint",&j2);
        ds.setJointPositions("upperArm_joint",&j3);
        ds.setJointPositions("foreArm_joint",&j4);
        ds.setJointPositions("wrist1_joint",&j5);
        ds.setJointPositions("wrist2_joint",&j6);
        ds.setJointPositions("wrist3_joint",&j7);

        moveit_msgs::RobotState dummy_state;
        robotStateToRobotStateMsg(ds,dummy_state,true);
        dummy_state.is_diff=true;
        move_group.setStartState(dummy_state);

        move_group.setGoalTolerance(0.001);
        move_group.setJointValueTarget(test_point_2);
        pogp_success = move_group.plan(test_plan_2);

        ROS_INFO("First Plan");
        std::cout << test_plan_1.trajectory_ << std::endl ;

        ROS_INFO("Second Plan");
        std::cout << test_plan_2.trajectory_ << std::endl ;

        if(pogp_success.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
        {
          ROS_INFO("Plan successfull for plan 2!!!");

          while(ros::ok() || (plan_number < EXECUTION_ITERATIONS))
          {
            traj_publisher.publish(test_plan_1.trajectory_.joint_trajectory);
            pogp_success = move_group.execute(test_plan_1);
            if(pogp_success.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
            {
              ROS_INFO("Execution success for plan 1!!!!");

              ros::Duration(2.0).sleep();

              traj_publisher.publish(test_plan_2.trajectory_.joint_trajectory);

              pogp_success = move_group.execute(test_plan_2);
              if(pogp_success.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
              {
                ROS_INFO("Execution success for plan 2!!!!");
                plan_number += 1 ;
                ros::Duration(2.0).sleep();
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
      }
      else
      {
        ROS_ERROR("Plan failed for plan 1!!!");
      }
    }
    else
    {
      ROS_ERROR("Failure in planning and execution of home plan");
    }

    ros::shutdown();
  return 0;

}
