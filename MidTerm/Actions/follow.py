from __future__ import division  # For correct float division in Python 2
from AriaPy import *
import sys
import numpy as np
from pidControl import PID



class ActionFollow ( ArAction ):
    ''' Action that guides the robot toward the line ax+by+c=0 defined by the list of [a,b,c]'''

    def __init__(self, waypoints,  strength):
        ArAction.__init__ ( self, "FollowLine" )
        self.myDesired = ArActionDesired ( )
        # self.myTurnThreshold = turnThreshold
        self.waypoints = waypoints
        self.num_waypoints = len(waypoints)
        self.current_index = 1
        self.is_looping = False
        WP1 = self.waypoints[0,:]
        WP2 = self.waypoints[1,:]
        #self.line = [0, 0, 0]  # holds line parameters [a, b, c]
        self.defLine ( WP1, WP2 )
        #print(self.line)
        self.dist_err = 0.0
        self.head_err = 0.0
        self.strength = strength
        self.target_wp = waypoints[1,:]

        self.heading_PID = self.heading_PID = PID(P=0.5, I=0.05, D=0.25, windupGuard=10)
        self.heading_PID.setSampleTime(0.01)

        self.velocity_PID = PID(P=2, I=0.005, D=0.25, windupGuard=10)
        self.velocity_PID.setSampleTime(0.01)


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
        # build ax + by + c = 0 for line through P1 -> P2
        x1, y1 = float(P1[0]), float(P1[1])
        x2, y2 = float(P2[0]), float(P2[1])
        # Standard implicit line: a x + b y + c = 0
        # a = y1 - y2,  b = x2 - x1,  c = x1*y2 - x2*y1
        self.line = np.array([y1-y2, x2-x1, x1*y2 - x2*y1])

        self.dx, self.dy = x2-x1, y2-y1
        self._norm = max(1e-9, np.hypot(self.line[0], self.line[1]))  # |(a,b)|

    def distance2line(self):
        x0, y0 = self.pose.x, self.pose.y
        num = self.line[0]*x0 + self.line[1]*y0 + self.line[2]
        #print(self.num)

        self.dist_err = float(np.clip(num / self._norm, -3000, 3000))
        print("Distance Error {:.2f} mm".format(self.dist_err))

    def bearing2line(self):
        # heading error (deg) between robot and line direction
        seg_brg_deg = np.degrees(np.arctan2(self.dy, self.dx))
        # shortest-turn wrap into [-180,180]
        self.head_err = ((seg_brg_deg - self.pose.th + 180.0) % 360.0) - 180.0
        #print("Heading Error {:.2f} deg".format(self.head_err*-0.1))

    def distance2waypoint(self, current_pose, target_wp):
        return float(np.linalg.norm(target_wp - current_pose))

    def advance_waypoint(self):
        import numpy as np
        # Called only AFTER confirming proximity to target

        if self.current_index == 0 and self.is_looping:
            print "Path Complete!"
            self.myDesired.setVel(0, self.strength)
            self.myDesired.setRotVel(0, self.strength)

            # save telemetry as CSV
            try:
                import numpy as np, time
                logRobot = np.array(self.states_log, dtype=float)

                # Handle empty or single-row logs robustly
                if logRobot.ndim == 1:
                    logRobot = logRobot.reshape(1, -1)

                fname = "linefollow.csv"
                np.savetxt(fname, logRobot, delimiter=",",
                           header="t_sec,x_mm,y_mm,theta_deg,cte_mm,head_err_deg",
                           comments="", fmt="%.3f")
                nrows = 0 if logRobot.size == 0 else logRobot.shape[0]
                print "Saved log to %s (%d rows)" % (fname, nrows)
            except Exception as e:
                print "Failed to save log:", e

            Aria_exit(0)
            return

        # remember previous target index (this becomes start P of the new segment)
        prev_idx = self.current_index

        # advance target by one (wrap at end)
        if self.current_index == (self.num_waypoints - 1):
            print "Reached end of the path. Looping back to the start."
            self.current_index = 0
            self.is_looping = True
        else:
            self.current_index += 1
            print "Moving to Next Waypoint"

        # update target and define NEW segment as P=prev_idx -> Q=current_index
        self.target_wp = self.waypoints[self.current_index, :]
        P = self.waypoints[prev_idx, :]
        Q = self.waypoints[self.current_index, :]
        self.defLine(P, Q)

        # --- debug prints (Py2-compatible) ---

        print "New segment: %d -> %d, P=%s, Q=%s" % (prev_idx, self.current_index, P, Q)
        print "Line a,b,c = %s" % (np.array2string(self.line, precision=2),)
        seg_brg = np.degrees(np.arctan2(self.dy, self.dx))
        seg_brg = float(seg_brg)
        print "Seg bearing: %.1f deg" % (seg_brg,)

        # sanity-check the initial distance error on the new segment
        self.distance2line()
        print "Initial dist_err on new segment: %.1f mm" % (self.dist_err,)

    def logRobotState(self):
        t = self.getRobot().getOdometerTime()
        x = self.pose.getX()
        y = self.pose.getY()
        th = self.pose.getTh()
        state_instance = [t, x, y, th, self.dist_err, self.head_err]
        self.states_log.append(state_instance)
        #print(state_instance, self.getRobot().getVel())

    def fire(self, currentDesired):
        self.myDesired.reset()
        self.pose = self.getRobot().getPose()
        robot_pose_np = np.array([self.pose.getX(), self.pose.getY()])

        self.logRobotState()

        self.distance2line()

        self.bearing2line()

        self.heading_PID.SetPoint= 0
        self.heading_PID.update(self.head_err)
        rot_cmd = self.heading_PID.output



        self.velocity_PID.SetPoint = 0
        self.velocity_PID.update(self.dist_err)
        #pid_cmd = self.velocity_PID.output
        #print("Forward Velocity Command: {:.2f} mm".format(pid_cmd))

        base_speed = 400.0

        #fwd_cmd = base_speed + pid_cmd
        fwd_cmd = base_speed



        # control law (keep your gains)
        #rot_cmd = -0.1 * self.dist_err + 5.5 * self.head_err
        #print(rot_cmd)
        # optional clamp
        rot_cmd = max(min(rot_cmd, 500), -500)

        # add forward velocity so it actually tracks the line
        #fwd_cmd = 400  # mm/s; tune as needed


        self.myDesired.setVel(fwd_cmd, self.strength)
        self.myDesired.setRotVel(rot_cmd, self.strength)

        dist_to_wp = self.distance2waypoint(robot_pose_np, self.target_wp)
        #print("Distance to waypoint {}: {:.2f} mm".format(self.current_index + 1, dist_to_wp))

        # ONLY advance when close to target waypoint
        if self.distance2waypoint(robot_pose_np, self.target_wp) < 400:
            self.advance_waypoint()

        return self.myDesired



            #ELSE print we're done! and log the data.




        #
        #return self.myDesired