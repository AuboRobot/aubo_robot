#!/usr/bin/env python

# Copyright (c) 2017-2018 diego. 
# All right reserved.
#

## @file joints.py the jonit clase support functions for joints.

## @brief Joints hold current values.
import roslib
import rospy
from ros_arduino_msgs.srv import *
from math import pi as PI, degrees, radians
class Joint:

    ## @brief Constructs a Joint instance.
    ##
    ## @param servoNum The servo id for this joint.
    ## 
    ## @param name The joint name.
    ## 
    ## @param name The servo control range.
    def __init__(self, name, servoNum, max, min, servo_max, servo_min, inverse):
        self.name = name
        self.servoNum=servoNum
        self.max=max
        self.min=min
        self.servo_max=servo_max
        self.servo_min=servo_min
        self.inverse=inverse

        self.position = 0.0
        self.velocity = 0.0

    ## @brief Set the current position.
    def setCurrentPosition(self):
        rospy.wait_for_service('/arduino/servo_write')
	try:
	        servo_write=rospy.ServiceProxy('/arduino/servo_write',ServoWrite)
	        servo_write(self.servoNum,self.position)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e   

    ## @brief Set the current position.
    ##
    ## @param position The current position. 
    def setCurrentPosition(self, position):
        rospy.wait_for_service('/arduino/servo_write')
	try:
		#if self.servoNum==2:
        	servo_write=rospy.ServiceProxy('/arduino/servo_write',ServoWrite)
        	self.mapToServoPosition(position)	        	
       		servo_write(self.servoNum,radians(self.position))

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    
    ## @brief map to  Servo Position.
    ##
    ## @param position The current position.        
    def mapToServoPosition(self,position):
	    per=(position-self.min)/(self.max-self.min)
	    if not self.inverse:	    	
	    	self.position=self.servo_min+per*(self.servo_max-self.servo_min)
	    else:
	    	self.position=self.servo_max-per*(self.servo_max-self.servo_min)


