AUBO Robot Driver

A new driver for the aubo_i3/aubo_i5/aubo_i7/aubo_i10 robot arms from AUBO Robots. It is designed to replace the old driver(For robot controller software version < V4), while solving some issues, improving usability as well as enabling compatibility of ros_control.
  
Features

You can control the robot by using both teach pendant and MoveIt.

The driver exposes the same functionality as the previous aubo_driver:

Action interface on /follow_joint_trajectory for seamless integration with MoveIt

Publishes robot joint state on /joint_states

Publishes IO state on /aubo_driver/io_state

Service call to set outputs.

Added support for ros_control.

As ros_control wants to have control over the robot at all times, ros_control compatibility is set via a parameter at launch-time.
With ros_control active, the driver doesn't open the action_lib interface. This is handled by ros_control instead.
Currently one controller is available, controlling the joint position of the robot, useable for trajectroy execution.

As the driver communicates with the robot via ethernet and depends on reliable continous communication, it is not possible to reliably control a AUBO Robot from a virtual machine.

Just clone the repository into your catkin working directory and make it with catkin_make.

Note that this package depends on aubo_msgs, hardware_interface, and controller_manager so it cannot directly be used with ROS versions for indigo and kinetic.

Usage

roslaunch robot_name_moveit_config robot_name_moveit_planning_executing.launch
roslaunch robot_name_moveit_config moveit_rviz.launch config:=true
