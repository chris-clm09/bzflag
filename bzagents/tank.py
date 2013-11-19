__author__ = 'scott'

from common import *


WAYPOINT_MET_THRESHOLD = 100                                     # radius around waypoint


class Tank(object):
    def __init__(self, _waypoints):
        self.waypoints = _waypoints                             # list of Point
        self.waypoint_index = 0
        self.index = None
        self.callsign = None                                    # string
        self.status = None                                      # string
        self.shots_avail = None                                 # int
        self.time_to_reload = None                              # float
        self.flag = None                                        # string
        self.x = None                                           # float
        self.y = None                                           # float
        self.angle = None                                       # float
        self.vx = None                                          # float
        self.vy = None                                          # float
        self.angvel = None                                      # float
        print "Initialized tank with waypoints %s" % str(self.waypoints)
        
    def update_state(self, bzrc_answer, time=None):
        self.index = bzrc_answer.index
        self.callsign = bzrc_answer.callsign
        self.status = bzrc_answer.status
        self.shots_avail = bzrc_answer.shots_avail
        self.time_to_reload = bzrc_answer.time_to_reload
        self.flag = bzrc_answer.flag
        self.x = bzrc_answer.x
        self.y = bzrc_answer.y
        self.angle = bzrc_answer.angle
        self.vx = bzrc_answer.vx
        self.vy = bzrc_answer.vy
        self.angvel = bzrc_answer.angvel
        self.update_waypoints(time)

    def update_waypoints(self, time):
        if distance(self.x, self.y, self.waypoints[self.waypoint_index]) < WAYPOINT_MET_THRESHOLD:  # hit waypoint
            self.waypoint_index = (self.waypoint_index + 1) % len(self.waypoints)
            return
        # TODO: use time for waypoint updating (timeouts)

    def get_waypoint(self):
        return self.waypoints[self.waypoint_index]