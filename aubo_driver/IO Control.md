# AUBO IO Control

* Besides controlling the movement of the robot, it is also desirable to be able to read and switch the status
of the robot’s  I/O states. <br>
* It is highly recommended to read the I/O parts of the user manual first to know the definition of the I/O function.
  
### Features
___

* The electrical interface of the control cabinet is divided into: safety I/O and general I/O.<br> 

* The safety I/O designed as dual channel (redundant design) to ensure the safety function shall not lost in any case of single failure. In use, the safety devices and equipment must be implemented in accordance with the safety instructions, and finished the comprehensive risk assessment before use.

* The AUBO interface board general I/O has:

	* 16 general digital input interfaces

	* 16 general digital output interfaces

	* 4 pairs of analog voltage input interfaces

	* 2 pairs of analog voltage output interfaces
	
	* 2 pairs of analog current output interface

* Control cabinet internal IO for the internal function interface, not open to the user. 

* There is a 8-pins mini connector on end-effector, which electrical error is about±10%, to provide power and control signals to specific tools (Holder for example) used in end.


### Usage
---
#### Reading the I/O status
* The status of the I/O can either be read from the "/aubo_driver/io_states" topic or be polled with an AUBO API.

* As the status is continuously broadcasted by the driver(at the rate of `50Hz`), the optimal way of
reading the status is to parse the data stream. <br>
```
stamp: The current IO states: 1528095448.141982s
digital_in_states: 
  - 
    pin: 0
    flag: False
    state: False
 ...
 
 digital_out_states: 
  - 
    pin: 0
    flag: True
    state: False
  ...
  
 analog_in_states: 
  - 
    pin: 0
    state: 0.114774115384
  ...
  
 analog_out_states: 
  - 
    pin: 0
    state: 0.0
  ...
 
 tool_io_states: 
  - 
    pin: 0
    flag: False
    state: False
  ...

tool_ai_states: 
  - 
    pin: 0
    state: 0.0
  ...
 ```

* In digital_in_states definition , pin 0 means pin "U_DI_00", etc.;
* In digital_out_states definition , pin 0 means pin "U_DO_00", etc.;
* In analog_in_states definition , pin 0 means pin "VI0", etc.;
* In analog_out_states definition , pin 0 means pin "VO0", etc.;

#### Seting the I/O state
* There is SetIOService running in the driver, so you can switch the I/O states by a `SetIORequest`.
* There are three keys in a request: `fun` `pin` `state`.
* The definition of the `fun`:
	* 1 : SetRobotBoardUserDO
	* 2 : SetRobotBoardUserAO
	* 3 : if `state == -1`, means setting the ToolDigitalIO to mode `IO_IN`, else the mode is `IO_OUT`
	* 4 : SetRobotToolAO
	* 5 : SetToolPowerType.
		* state==0 toolValtage = 0V
		* state==1 toolValtage = 12V
		* state==2 toolValtage = 24V
* examples(see testIO.cpp):
```
aubo_msgs::SetIO srv;
srv.request.fun = fun;
srv.request.pin = pin;
srv.request.state = state;
client.call(srv)
1)set the twist tool power voltage to 24V
 then: fun = 5, state = 2;
 
2)set the RobotBoardUserDO 'U_DI_00' to valid
then: fun = 1; pin = 0; state = 1.
```

