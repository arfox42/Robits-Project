from __future__ import division
from AriaPy import *
import sys
from Actions.avoid import ActionAvoid
from Actions.draw import ActionDrawWaypoints
from Actions.go import ActionGo
from Actions.follow import ActionFollow
from Functions.read_waypoints import read_waypoints





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

    #WP1 = [5000 * scale, -5000 * scale]
    #WP2 = [-2000 * scale, -3000 * scale]

    #Create Waypoint List
    waypoint_list = read_waypoints.read_waypoints_from_file("waypoint_list.txt")

    WP1 = waypoint_list[0]
    WP2 = waypoint_list[1]
    WP3 = waypoint_list[2]
    WP4 = waypoint_list[3]
    WP5 = waypoint_list[4]
    WP6 = waypoint_list[5]



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
    follow = ActionFollow ( waypoint_list, 1.0 )  # 0.4
    draw = ActionDrawWaypoints ( waypoint_list )

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
    #robot.addAction (draw, 1)

    # reuse a set of library actions from Aria
    # robot.addAction(avoidfront,8)
    # robot.addAction(avoidside,8)
    # robot.addAction(recover, 80)
    # robot.addAction(limiter,82)
    # robot.addAction(limiterFar,85)
    #robot.setVel ( 3300 )

    # Enable the motors
    robot.enableMotors ( )

    # Run the robot processing cycle.
    # 'true' means to return if it loses connection,
    # after which we exit the program.
    robot.run ( True )

    Aria_exit ( 0 )

    pass
