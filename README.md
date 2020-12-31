# AUBO Robot
**Maintainer: zhaoyu@aubo-robotics.cn**




* Aubo_robot [ROS](http://www.ros.org/) meta-package for [ROS-Industrial](http://wiki.ros.org/Industrial). See the [ROS wiki](http://wiki.ros.org/)  page for more information.

* This repository provides ROS support for [AUBO robots](https://aubo-robotics.com/en/). This repository holds source code for `kinetic`. The corresponding robot controller software version is `V4`. For those software version < `V4`, can refer to [here]( http://wiki.ros.org/aubo_robot).

### Installation from Source
---

For the latest features and developments you might want to install from source.<br>

1. First set up a catkin workspace (see this [tutorials](http://wiki.ros.org/catkin/Tutorials)).<br>
2. Then clone the repository into the `src/` folder. It should look like `/user_name/catkin_workspace/src/aubo_robot`.<br>
3. In order to compile the package successfully, you need to install some packages related to `MoveIt` and `industrial_core`package.<br>
4. Make sure to source the correct setup file according to your workspace hierarchy, then use **catkin_make** to compile.<br>

5. update Moveit_core_lib : enter the direction `/user_name/catkin_workspace/src/aubo_robot/UpdateMoveitLib/Kinetic/`,Add permissions to the Update.sh file`chmod +x Update.sh` , Execution script file `sudo ./Update.sh`

	The `recommended` gcc version is 4.8 for Ubuntu 14.04; And for Ubuntu 16.04, the gcc version is 5.4.

### Contents
----
* The repository contains packages may be unstable, and they will be migrated to the [ROS-I](https://github.com/ros-industrial) repository after sufficient testing.<br>
* The contents of these packages are subject to change, without prior notice.<br>
* Any available APIs are to be considered unstable and are not guaranteed to be complete and / or functional.


### MoveIt! with a simulated robot
---

You can use MoveIt! to control the simulated robot like ***RVIZ*** ,***Gazebo*** or ***VREP*** environment. If you want to simulate the robot in RVIZ:

* First set up the MoveIt nodes to allow motion planning and run:

**rviz**
```  
        1.roslaunch <robot_name>_moveit_config moveit_planning_execution.launch robot_ip:=127.0.0.1  
```

**gazebo**  
```
        2.roslaunch aubo_gazebo aubo_<robot_name>_gazebo_control.launch
```
**you should install some package when you use aubo model in gazebo** [here](https://github.com/lg609/aubo_robot/blob/master/aubo_gazebo/README.md)

* Then select `"Interact"` and move the end-effector to a new goal.

* In  `"Motion Planning"` -> `"Plan and Execute"` to send trajectory to the sim robot

* Exit RViz and Ctrl-C the demo.launch window

### Usage with real robot
---
```diff
+  When operating an AUBO robot under ROS-Industrial control, make certain that no one is within the robot workspace and the e-stop is under operator control.
```
* Source the correct setup shell files and use a new terminal for each command.

* To bring up the real robot, run:

```
roslaunch <robot_name>_moveit_config moveit_planning_execution.launch sim:=false robot_ip:=<192.168.***.***>
```


* Move the robot around manually using a teach-pendant. Make sure the robot is controlled by robot-controller, see [here](https://github.com/lg609/aubo_robot/tree/master/aubo_driver).

* Verify that the joint positions in RVIZ match the physical robot configuration.

* Then switch to the ros-controller and control the robot through MoveIt.
* Repeat for every joint and pose.

**CAUTION:<br>
	Mistakes made during this verification step can result in dangerous collisions when experiment with using the MoveIt planning environment to 	command trajectories with the real robot. Be certain that an E-stop is close by whenever commanding robot motion.**
