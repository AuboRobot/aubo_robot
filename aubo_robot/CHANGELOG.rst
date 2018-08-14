^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package aubo_robot
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.3 (2018-08-01)
------------------
1. To unify the way to start the driver node, you have to specify the IP address in the command line, like : roslaunch moveit_planning_execution.launch robot_ip:=127.0.0.1;
2. You can publish a message on "robot_control" topic to control the robot. For example, if you wanna power on the robot from the driver,  you can publish a message named "powerOn";
3. You can subscribe to the "robot_status" ,  if the robot runs into singularity Overspeed, you can get that status form the param "msg->error_code".

1.3.2 (2018-07-31)
------------------
