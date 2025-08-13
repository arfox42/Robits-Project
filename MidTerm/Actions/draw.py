from AriaPy import *

class ActionDrawWaypoints(ArAction):
    def __init__(self, waypoints):
        ArAction.__init__(self, "DrawWaypoints")
        self.waypoints = waypoints
        self.myDesired = ArActionDesired()

    def setRobot(self, robot):
        # Must call the parent class's setRobot
        print "ActionDrawWaypoints: calling ArAction.setActionRobot..."
        self.setActionRobot(robot)

        # Draw the waypoints here as a one-time action
        self.draw_waypoints()

    def draw_waypoints(self):
        # Create a single drawing object for the path
        draw_path = ArDrawingData(ArDrawingData.LINE_STRIP, ArColor(255, 0, 0), 1)

        # Loop through waypoints and add them to the path drawing data
        for wp in self.waypoints:
            draw_path.addPoint(wp.getX(), wp.getY())

        # Send the path drawing command to the simulator
        self.getRobot().getPacketSender().com(ArCommands.DRAW, draw_path)

        # Create separate drawing objects for each point
        for wp in self.waypoints:
            draw_point = ArDrawingData(ArDrawingData.CIRCLE_FILLED, ArColor(0, 255, 0), 1)
            draw_point.addPoint(wp.getX(), wp.getY())
            draw_point.setSize(50)  # Set circle radius in mm
            self.getRobot().getPacketSender().com(ArCommands.DRAW, draw_point)

    def fire(self, currentDesired):
        # This action's job is done, so it requests no movement
        self.myDesired.reset()
        return self.myDesired