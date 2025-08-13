from __future__ import division  # For correct float division in Python 2

'''
Created by vldobr on 8/4/2018
using PyCharm
'''

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


class ActionFollow ( ArAction ):
    ''' Action that guides the robot toward the line ax+by+c=0 defined by the list of [a,b,c]'''

    def __init__(self, P1, P2, strength):
        ArAction.__init__ ( self, "FollowLine" )
        self.myDesired = ArActionDesired ( )
        # self.myTurnThreshold = turnThreshold
        self.line = [0, 0, 0]  # holds line parameters [a, b, c]
        self.defLine ( P1, P2 )
        self.dist_err = 0.
        self.head_err = 0.
        self.strength = strength
        # telemetry log
        self.states_log = []

    def setRobot(self, robot):
        ''' Sets myRobot in the parent ArAction class (must be done):'''
        print "Action Bearing only guidance: calling ArAction.setActionRobot..."
        self.setActionRobot ( robot )
        # Find sonar object for use in fire():
        self.heading = robot.getTh ( )
        if (self.heading == None):
            ArLog.log ( ArLog.Terse, "actionGuidance: Warning: No heading feedback, deactivating." )
            self.deactivate ( )

    def defLine(self, P1, P2):
        self.line = np.array ( [-(P2[0] - P1[0]), P2[1] - P1[1], P2[0] * P1[1] - P2[1] * P1[0]] )

    def distance2line(self):
        ''' calculate distance between the line given by 2 points and the point(x0,y0)'''
        num = np.inner ( self.line, np.array ( [self.pose.y, self.pose.x, 1.] ) )
        den = np.sqrt ( self.line[0] ** 2 + self.line[1] ** 2 )
        # self.dist_err = np.abs(num)/den
        self.dist_err = num / den
        if np.abs ( self.dist_err ) > 3000:
            self.dist_err = 3000 * np.sign ( num )

    def bearing2line(self):
        # self.head_err = 180./np.pi * ( self.wrap( np.arctan2(-self.line[1],self.line[0]) ) \
        #                 - self.wrap( np.pi*self.pose.th/180) )
        angles = np.unwrap ( np.array ( [np.arctan2 ( -self.line[1], self.line[0] ), np.pi * self.pose.th / 180] ),
                             axis=-1 )
        self.head_err = 180. / np.pi * (angles[0] - angles[1])

    def logRobotState(self):
        """ log and report the telemetry"""
        state_instance = [self.getRobot ( ).getOdometerTime ( ),
                          self.pose.x, self.pose.y, self.pose.th,
                          self.dist_err, self.head_err]
        self.states_log.append ( state_instance )
        print (state_instance, self.getRobot().getVel())

    def fire(self, currentDesired):
        # reset the actionDesired (must be done)
        self.myDesired.reset ( )
        self.pose = self.getRobot ( ).getPose ( )
        # log state of the robot
        self.logRobotState ( )
        # print "Robot's position %s"%self.pose
        print "Robot's time= %s " % self.getRobot ( ).getOdometerTime ( )
        self.distance2line ( )
        self.bearing2line ( )
        # control law
        rot_cmd = -0.1 * self.dist_err + 5.5 * self.head_err
        # print "commanded rotation %s, dist_error %s, head_err %s"% (rot_cmd, self.dist_err, self.head_err)
        self.myDesired.setRotVel ( rot_cmd, self.strength )  # setRotVel (double rotVel, double strength=MAX_STRENGTH)
        # self.myDesired.setDeltaHeading(rot_cmd, self.strength)
        # verify the duration of the experiment
        if robot.getOdometerTime ( ) >= 25.:
            print "Time expired"
            # save telemetry in a file
            logRobot = np.array ( self.states_log )  # convert list of records into one matrix of size [N x 8]
            filename = 'linefollow.log'
            with open ( filename, 'w' ) as out:
                np.savetxt ( filename, logRobot, delimiter=',' )
            Aria_exit ( 1 )

        return self.myDesired

if __name__ == '__main__':
    Aria_init ( )
    parser = ArArgumentParser ( sys.argv )
    parser.loadDefaultArguments ( )
    robot = ArRobot ( )
    conn = ArRobotConnector ( parser, robot )
    sonar = ArSonarDevice ( )
    robot.getPacketSender().com(ArCommands.SIM_RESET)

    scale = 1
    # WP1 = [2500 * scale, 5000 * scale] # end
    # WP2 = [-5000 * scale, 2500 * scale] # start
    # WP1 = [2500 * scale, 5000 * scale]
    # WP2 = [-5000 * scale, 2500 * scale]

    WP2 = [5000 * scale, -5000 * scale]
    WP1 = [-2000 * scale, -3000 * scale]

    # Kinematic constraints
    robot.setTransAccel ( 900 )
    robot.setTransDecel ( 900 )
    robot.setTransVelMax ( 1300 )
    robot.setAbsoluteMaxRotVel ( 100 )
    # initial speed
    robot.setVel ( 0 )

    # Create instances of the actions defined above, plus ArActionStallRecover,
    # a predefined action from Aria_
    go = ActionGo ( 500, 650, 1.0 )  # 0.1
    avoid = ActionAvoid ( 2500, 60, 1.0 )
    follow = ActionFollow ( WP1, WP2, 1.0 )  # 0.4

    # reuse a set of library actions from Aria
    avoidfront = ArActionAvoidFront ( "avoid front obstacles", 1400, 200, 10, True )
    avoidside = ArActionAvoidSide ( "Avoid side", 500, 15 )
    recover = ArActionStallRecover ( )  # standard recover
    limiter = ArActionLimiterForwards ( "speed limiter near", 25, 500, 250, 1 )  # limiter for close obstacles
    limiterFar = ArActionLimiterForwards ( "speed limiter far", 500, 1500, 500, 2 )  # limiter for far away obstacles

    # Connect to the robot
    if not conn.connectRobot ( ):
        ArLog.log ( ArLog.Terse, "actionExample: Could not connect to robot, thus Exiting..." )
        Aria_exit ( 1 )

    # Parse all command-line arguments
    if not Aria_parseArgs ( ):
        Aria_logOptions ( )
        Aria_exit ( 1 )

    # Add the range device to the robot. You should add all the range
    # devices and such before you add actions
    robot.addRangeDevice ( sonar )

    # Add our actions in order. The second argument is the priority,
    # with higher priority actions going first, and possibly pre-empting lower
    # priority actions.
    # robot.addAction(go, 5)      #2
    robot.addAction ( follow, 9 )  # 4
    # robot.addAction(avoid, 8)    #10

    # reuse a set of library actions from Aria
    # robot.addAction(avoidfront,8)
    # robot.addAction(avoidside,8)
    # robot.addAction(recover, 80)
    # robot.addAction(limiter,82)
    # robot.addAction(limiterFar,85)
    robot.setVel ( 3300 )

    # Enable the motors
    robot.enableMotors ( )

    # Run the robot processing cycle.
    # 'true' means to return if it loses connection,
    # after which we exit the program.
    robot.run ( True )

    Aria_exit ( 0 )

    pass
