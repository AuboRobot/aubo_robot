# AUBO Robot Driver

* A new driver for the `aubo_i3/aubo_i5/aubo_i7/aubo_i10` robot arms from AUBO robots. <br>
* It is designed to replace the [old driver](https://github.com/auboliuxin/aubo_robot/tree/master/aubo_driver) (For robot controller software version < `V4`), while solving some issues, improving usability as well as enabling compatibility of [**ros_control**](http://wiki.ros.org/ros_control).
  
### Features
___

* You can control the robot by using both teach pendant(through *`AUBO API`*) and `MoveIt`.<br>
  When lauching the driver node, the default controller is `ros-controller`, when you want to switch to the `robot-controller`, you should publish a message, like:<br> 
  ```
  rostopic pub -1 aubo_driver/controller_switch std_msgs/Int32 -- 0
  ```
  The last argument mean the robot is controlled by robot-controller, when you want to go back to ros-controller, just change the value **"0"** to **"1"**.

* The driver maintain some of the same functionality as the previous [aubo_driver](https://github.com/auboliuxin/aubo_robot):

	* Action interface on `/follow_joint_trajectory` for seamless integration with MoveIt.

	* Publishes robot joint state on `/joint_states`

	* Publishes IO state on `/aubo_driver/io_states`

	* Service call to set outputs.

* Added support for `ros_control`.

	* As ros_control wants to have control over the robot at all times, ros_control compatibility is set via a parameter at launch-time.
	* With ros_control active, the driver doesn't open the action_lib interface. This is handled by ros_control instead.
	* Currently one controller is available, controlling the joint position of the robot, useable for trajectroy execution.

### Installation
---
* As the driver communicates with the robot via ethernet and depends on reliable continous communication, it is not possible to reliably control a AUBO Robot from a virtual machine.

* Just clone the repository into your catkin working directory and make it with catkin_make.

* Note that this package depends on aubo_msgs, hardware_interface, and controller_manager so it cannot directly be used with ROS versions for indigo and kinetic.

### Usage

```
roslaunch robot_name_moveit_config robot_name_moveit_planning_executing.launch
roslaunch robot_name_moveit_config moveit_rviz.launch config:=true
```
There is a parameter named `"/aubo_controller/velocity_scale_factor"`, you can use it to sacle the velocity of the trajectory. The bigger the parameter is, faster the robot will move.
### Test
---
* `tsetIO`: The message of topic `/aubo_driver/io_states` is published at a rate of 100Hz. To set IO states, you need to establish a ServiceClient to call servive on `/aubo_driver/set_io`
* `testAuboAPI`: There are plenty of functionality you can use to control the robot, see this [declaration](https://github.com/lg609/aubo_robot/blob/master/aubo_driver/include/aubo_driver/servicehenterface.h). If you want to control the robot more efficiency, it is recommanded to use the Aubo API to realize the functions.

