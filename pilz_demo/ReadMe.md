#1. Launch the robot launch files
#source devel/setup.bash
#roslaunch aubo_i5_pilz_moveit_config moveit_planning_execution.launch robot_ip:=127.0.0.1

#2. Launch fake_speed_override_node
#source devel/setup.bash
#rosrun prbt_hardware_support fake_speed_override_node
#or
#To create the service /get_speed_override you can add this line in your launch file:
#"<node name="fake_speed_override_node" pkg="prbt_hardware_support" type="fake_speed_override_node"/>"

#3. rosrun demo
#source devel/setup.bash
#rosrun pilz_robot_programming demo_program.py 

#or to move the robot with python:
#from pilz_robot_programming import *
#__REQUIRED_API_VERSION__ = "1" # That is required
#robot  = Robot(version= __REQUIRED_API_VERSION__)
#cmdJoint = Ptp(goal=angles, vel_scale=1.0,target_link='you_tool_link',planning_group = "your_move_group_name", #reference_frame="base_link")
#robot.move(cmdJoint)
#rosservice call pause_movement
#rosservice call resume_movement
#rosservice call stop_movement
#Multithreading
#When move() is running in a separate thread, the move control orders can be issued directly via the following methods of the robot object:

#r.pause()
#r.resume()
#r.stop()
#In this case stop() only ends the move-thread.
#if need use blending sequence,must disable Addtime adapter
