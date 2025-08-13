
from __future__ import division  # For correct float division in Python 2
from AriaPy import *
import sys
import numpy as np

"""
An example program demonstrating how to make and use new actions.

This example program combines two actions Go and Turn to enable Line following. 
"""

# Action that drives the robot forward, but stops if obstacles are
# detected by sonar.
class ActionGo ( ArAction ):
    ''' constructor, sets myMaxSpeed and myStopDistance
    '''

    def __init__(self, maxSpeed, stopDistance, strength):
        ArAction.__init__ ( self, "Go" )
        self.myMaxSpeed = maxSpeed
        self.myStopDistance = stopDistance
        self.myDesired = ArActionDesired ( )
        self.strength = strength
        self.mySonar = None

    # Override setRobot() to get a reference to the sonar device
    def setRobot(self, robot):
        ''' Set myRobot object in parent ArAction class (must be done if
        you overload setRobot):
        # self.myRobot = robot
        '''
        print "ActionGo: setting robot on ArAction..."
        self.setActionRobot ( robot )

        # Find sonar device for use in fire():
        self.mySonar = robot.findRangeDevice ( "sonar" )
        if (self.mySonar == None):
            ArLog.log ( ArLog.Terse,
                        "actionExample: ActionGo: Warning: The robot had no sonar range device, deactivating!" )
            self.deactivate ( )

    def fire(self, currentDesired):
        ''' reset the actionDesired (must be done), to clear
        # its previous values.
        '''
        self.myDesired.reset ( )

        # if the sonar is null we can't do anything, so deactivate
        if self.mySonar == None:
            self.deactivate ( )
            return None

        # get the range of the sonar
        range = self.mySonar.currentReadingPolar ( -70, 70 ) - self.getRobot ( ).getRobotRadius ( )

        # if the range is greater than the stop distance, find some speed to go
        if (range > self.myStopDistance):
            # just an arbitrary speed based on the range
            speed = range * 1.8
            # if that speed is greater than our max, cap it
            if (speed > self.myMaxSpeed):
                speed = self.myMaxSpeed
            # now set the velocity
            self.myDesired.setVel ( speed, self.strength )
        else:
            # the range was less than the stop distance, so request stop
            self.myDesired.setVel ( range * 0.99, self.strength )

        # return a reference to our actionDesired to the resolver to make our request
        return self.myDesired