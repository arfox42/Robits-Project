from __future__ import division  # For correct float division in Python 2
from AriaPy import *
import sys
import numpy as np
'''
Created by vldobr on 8/4/2018
using PyCharm
'''



class ActionAvoid ( ArAction ):
    ''' Action that turns the robot away from obstacles detected by the sonar.'''

    def __init__(self, turnThreshold, turnAmount, strength):
        ArAction.__init__ ( self, "Avoid_obstacle" )
        self.myDesired = ArActionDesired ( )
        self.myTurnThreshold = turnThreshold
        self.myTurnAmount = turnAmount
        self.strength = strength

        # remember which turn direction we requested, to help keep turns smooth
        self.myTurning = 0  # -1 == left, 1 == right, 0 == none

    def setRobot(self, robot):
        ''' Sets myRobot in the parent ArAction class (must be done):'''
        print "ActionTurn: calling ArAction.setActionRobot..."
        self.setActionRobot ( robot )
        # self.myRobot = robot

        # Find sonar object for use in fire():
        self.mySonar = robot.findRangeDevice ( "sonar" )
        if (self.mySonar == None):
            ArLog.log ( ArLog.Terse, "actionExample: ActionTurn: Warning: I found no sonar, deactivating." )
            self.deactivate ( )

    def fire(self, currentDesired):
        # reset the actionDesired (must be done)
        self.myDesired.reset ( )

        # if the sonar is null we can't do anything, so deactivate
        if self.mySonar == None:
            self.deactivate ( )
            return None

        # Get the left readings and right readings off of the sonar
        leftRange = (self.mySonar.currentReadingPolar ( 0, 60 ) -
                     self.getRobot ( ).getRobotRadius ( ))
        rightRange = (self.mySonar.currentReadingPolar ( -60, 0 ) -
                      self.getRobot ( ).getRobotRadius ( ))

        # if neither left nor right range is within the turn threshold,
        # reset the turning variable and don't turn
        if (leftRange > self.myTurnThreshold and rightRange > self.myTurnThreshold):
            self.myTurning = 0
            self.myDesired.setDeltaHeading ( 0, self.strength )

        # if we're turning already and left is closer, turn right
        # and set the turning variable so we turn the same direction for as long as
        # we need to
        elif (self.myTurning != 0 and leftRange < rightRange):
            self.myTurning = -1
            self.myDesired.setDeltaHeading ( self.myTurnAmount * self.myTurning, self.strength )

        # if we're turning already and right is closer, turn left
        # and set the turning variable so we turn the same direction for as long as
        # we need to
        else:
            self.myTurning = 1
            self.myDesired.setDeltaHeading ( self.myTurnAmount * self.myTurning, self.strength )

        # return the actionDesired, so resolver knows what to do
        return self.myDesired