#### Table of Contents
- [Introduction](#introduction)
- [Supported DIN EN ISO 10218-1 safety features](#supported-din-en-iso-10218-1-safety-features)
  - [Needed/supported hardware](#neededsupported-hardware) 
  - [How-to activate and deactivate hardware features](#how-to-activate-and-deactivate-hardware-features)
  - [Definitions](#definitions)
  - [Safe stop 1](#safe-stop-1-ss1)
  - [Brake test](#brake-test)
  - [Operation modes](#operation-modes)
  - [Speed monitoring](#speed-monitoring)
- [Possible error cases and their handling](#possible-error-cases-and-their-handling)
- [Overview system components](#overview-system-components)

# Introduction
The `prbt_hardware_support` package provides support for the Pilz hardware
PNOZmulti and PSS 4000. A number of safety features are provided which are
essential for a DIN EN ISO 10218-1 certifiable robot system (for more
information see section about 
[supported DIN EN ISO 10218-1 safety features](#supported-din-en-iso-10218-1-safety-features)).  
  
Due to the fact, that the communication between the Pilz safety controllers 
and ROS is based on the Modbus communication protocol, the package also 
contains C++ files providing ROS support for the Modbus communication protocol.
  
In the [system overview section](#overview-system-components),
you can find a component diagram showing the overall architecture of our
system. The component diagram shows all our nodes and the connections 
between them.

**Please note:**  
The launch files included in the `prbt_hardware_support` package don't need to
be called directly by the user. They already are included in our top-level
launch file (for more information on how-to run the PRBT robot with ROS see the 
[pilz_robots-README](https://github.com/PilzDE/pilz_robots#running-on-the-real-robot)
and our [application templates](https://github.com/PilzDE/pilz_application_templates).

# Supported DIN EN ISO 10218-1 safety features
The prbt_hardware_support package provides support for a number of safety
features which are essential for a DIN EN ISO 10218-1 certifiable robot system.
  
**Please note:**  
The DIN EN ISO 10218-1 support is currently **WORK IN PROGRESS**.
 
## Needed/supported hardware
In order for the safety features to work, one needs special hardware
components supporting the required safety features.
Currently, we test all our features against the following 
hardware setup:
- Robot: Manipulator module PRBT 6
- Safety controller: PSS 4000 (with a dedicated program)
- Operating mode selector switch: PITmode
- Enabling switch: PITenable
- Pushbutton unit (with emergency stop): PITgatebox

# How-to activate and deactivate hardware features
The following table shows the names and the possible argument values of the features supported by our hardware.
The features can be configured via arguments of `prbt_support/launch/robot.launch`.

| Parameter Description| Argument name in `robot.launch` | Possible values in `robot.launch` |
| - | - | - |
| Gripper Model | `gripper` | *`<arg unset>`*, `pg70` |
| Safety Controller Hardware | `safety_hw` | *`pss4000`*, `pnoz` |
| Brake Test Support | `has_braketest_support` | *`true`*, `false` |
| Operation Mode Support | `has_operation_mode_support` | *`true`*, `false` |
| Visual Status Indicator | `visual_status_indicator` | *`true`*, `false` |

With `pg70` referring to __Schunk PG plus 70__.
For more on gripper models see [prbt_grippers](https://github.com/PilzDE/prbt_grippers).

The optional argument `iso10218_support` can be used to start ROS without connecting to a safety controller. Keep in mind that this disables all safety features, so it should only be used for debugging purposes.

Currently, we only support the following configurations depending on the safety controller. If the safety controller is changed, please ensure that the arguments are set as shown in the table.

| Argument name | Default value with PSS400 | Default value with PNOZmulti |
| - | - | - |
| `safety_hw` | `pss4000` | `pnoz` |
| `has_braketest_support` | `true` | `false` |
| `has_operation_mode_support` | `true` | `true` |
| `visual_status_indicator` | `true` | `true` |

## Definitions
### RUN_PERMITTED signal
The RUN_PERMITTED is a state required in the safety controller for 
the robot to operate. It is sent to the ROS system to inform it in the 
case of an upcoming STO of the robot.

**Please note:**
The ROS nodes only react on changes of the signal, so it may be necessary 
to re-trigger RUN_PERMITTED in order to enable the drives at start.

### STO
The STO function (“Safe torque off”) of the robot arm is a safety function 
to immediately turn off torque of the drives. The behavior triggers the 
RUN_PERMITTED signal.

### SBC
The SBC function ("Safe brake control") of the robot arm is a safety function
which is used in conjunction with the RUN_PERMITTED and prevents a motion when
the torque of the drives is turned off.

## Safe stop 1 (SS1)
To allow a controlled stop, the safety controller delays the STO signal by
several milliseconds. The STO signal, otherwise, would lead to an immediate stop
of the robot via Stop 0. The time delay gives the ros_control time to
stop the drives with a well defined brake ramp.  
The safety controller informs the ros_control about the stop request
via Modbus connection. The Modbus connection is opened by this package.  
After the execution of the brake ramp, the drivers are halted.

**Please note:**  
Should ROS fail (for what ever reason), the safety controller still ensures
that the robot is stopped by executing a Stop 0. The Stop 0 is executed after
the above described time delay.

## Brake test
Brake tests are an integral part of the safe brake control (SBC) functionality,
since they detect malfunctions of the brakes or the brake control in general.
Brake tests for each drive have to be executed at a regular interval. 
When the safety controller requests brake tests, they have to be executed 
within 1 hour, else the robot cannot be moved anymore.

## Operation Modes
The robot system can be controlled in various modes.

These modes are:
  - T1: Speed reduced to 250 mm/s (each robot-frame), enabling switch must be pressed
  - T2<sup>*</sup>: The robot moves at full speed but still the enabling switch must be pressed
  - AUTOMATIC: The robot moves at full speed and follows a predefined program/process. 
    No enabling switch is needed but safety has to be ensured by safety 
    peripherie (fences, light curtains, ...).

See DIN EN ISO 10218-1 for more details or contact us: ros@pilz.de

**Please note:**  
In operation mode T1 the robot can be moved as usual. 
However, if an attempt to exceed the speed limit of 250 mm/s in T1 is detected,
the current motion is aborted and a controlled stop is performed.
For more information see  the [speed monitoring section](#speed-monitoring).

## Speed monitoring
DIN EN ISO 10218-1 requires that in operation mode T1 no part of the robot
moves faster than 250 mm/s. To meet this requirement, 
the `PilzJointTrajectoryController` checks the target velocity for
each robot-frame. If one or more robot-frames exceed the allowed speed limit,
the controller executes a safe stop 1. (For more information about the safe
stop 1, see [safe stop 1 section](#safe-stop-1-ss1)).  
To re-enable the system, the user needs to release and, subsequently, 
press the enabling switch.

**Please note:**  
Currently, only one of the PILZ controllers, namely the
`PilzJointTrajectoryController`, can perform the speed monitoring required by
DIN EN ISO 10218-1.

# Possible error cases and their handling

| Error cases                                             | Handling                                                |
| ------------------------------------------------------- | ------------------------------------------------------- |
| Modbus client crashes                                   | ROS system is shutdown which leads to an abrupt stop of the robot. |
| RUN_PERMITTED Modbus adapter crashes                              | ROS system is shutdown which leads to an abrupt stop of the robot. |
| Connection loss between PNOZmulti/PSS4000 & Modbus client        | Stop 1 is triggered                                     |
| System overload (messages don't arrive in time)         | In case a Stop 1 message does not arrive in time, the safety controller will automatically perform a hard stop. In case a Stop 1-release message does not get through, brakes will remain closed. |
| RUN_PERMITTED Modbus adapter cannot connect to stop services    | ROS system will not start.                              |
| RUN_PERMITTED Modbus adapter cannot connect to recover services | Node does start and robot can be moved until a stop is triggered. Afterwards the brakes will remain closed. |

# Overview system components
The following diagram shows all components of the system and the connections
between them.  

![Component diagram of overall architecture](doc/diag_comp_overall_architecture.png)

## ModbusClient
A Modbus client (for usage with the PNOZmulti or PSS4000) can be started 
with `roslaunch prbt_hardware_support modbus_client.launch`.

### Published Topics
- ~/pilz_modbus_client_node/modbus_read (prbt_hardware_support/ModbusMsgInStamped)
  - Holds information about the modbus holding register. 
    Timestamp is only updated if the register content changed.

### Parameters
- modbus_server_ip
- modbus_server_port
- index_of_first_register_to_read
- num_registers_to_read
- modbus_connection_retries (default: 10)
- modbus_connection_retry_timeout - timeout between retries (default: 1s)
- modbus_response_timeout (default: 20ms)
- modbus_read_topic_name (default: "/pilz_modbus_client_node/modbus_read")
- modbus_write_service_name (default: "/pilz_modbus_client_node/modbus_write")

**Please note:**
- The parameters ``modbus_response_timeout`` and ``modbus_read_topic_name`` are
important for the Safe stop 1 functionality and must NOT be given, if the
``pilz_modbus_client_node`` is used as part of the Safe stop 1 functionality.
If the parameters are not given the default values for these parameters are used.

## ModbusAdapterRunPermittedNode
The ``ModbusAdapterRunPermitted`` is noticed via the topic 
`/pilz_modbus_client_node/modbus_read` if the RUN_PERMITTED is true or false 
and reacts as follows calling the corresponding services of the controllers 
and drivers:
- **RUN_PERMITTED true:**
enable drives, unhold controllers
- **RUN_PERMITTED false:**
hold controllers, disable drives

## ModbusAdapterBrakeTestNode
The ``ModbusAdapterBrakeTestNode`` offers the `/prbt/brake_test_required` 
service which informs if the PSS4000 requests
a brake test or if a brake test request is no longer prevailing.

## BraketestExecutorNode
The ``BraketestExecutorNode`` offers the `/execute_braketest` service which, 
in interaction with the ``CanOpenBraketestAdapter``,
executes a braketest on each drive of the manipulator. This can only be done, 
if the robot is stopped. So, if you want to execute a braketest, 
ensure that the robot stands still.

## ModbusAdapterOperationModeNode
The ``ModbusAdapterOperationModeNode`` publishes the active operation mode 
on the topic `/prbt/operation_mode` everytime it changes and offers 
the `/get_operation_mode` service for accessing the active operation mode.

Use `rosmsg show pilz_msgs/OperationModes` to see the definition
of each value.

## OperationModeSetupExecutorNode
The ``OperationModeSetupExecutorNode`` activates the speed monitoring for 
operation mode T1 and offers a service ``/prbt/get_speed_override``. 
The speed override is chosen such that a speed limit violation is unlikely if 
all robot motions are scaled with it.

<sup>*</sup>Not supported yet
