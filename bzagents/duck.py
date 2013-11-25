#!/usr/bin/python -tt

# An incredibly simple agent.  All we do is find the closest enemy tank, drive
# towards it, and shoot.  Note that if friendly fire is allowed, you will very
# often kill your own tanks with this code.

#################################################################
# NOTE TO STUDENTS
# This is a starting point for you.  You will need to greatly
# modify this code if you want to do anything useful.  But this
# should help you to know how to interact with BZRC in order to
# get the information you need.
#
# After starting the bzrflag server, this is one way to start
# this code:
# python agent0.py [hostname] [port]
#
# Often this translates to something like the following (with the
# port name being printed out by the bzrflag server):
# python agent0.py localhost 49857
#################################################################

import sys
import math
import time
import random

from bzrc import BZRC, Command


class AGoal:
    def __init__(self, x, y):
        self.x = x
        self.y = y


####################################################################
# Distance between two points.
####################################################################
def distance(x, y, goal):
    return math.sqrt(((goal.y - y)*(goal.y - y)) + ((goal.x - x)*(goal.x - x)))


def distance_points(x, y, xg, yg):
    return math.sqrt(((yg - y)*(yg - y)) + ((xg - x)*(xg - x)))


def sign(a):
    if a == 0 or a == -0:
        return 0
    return a / -a


####################################################################
# Generate a single attractive vector.
####################################################################
def gen_an_attractive_field(x, y, goal):
    r = 1.5
    s = 30.0
    al = 1.0/s
    
    d = distance(x, y, goal)
    
    theta = math.atan2(goal.y - y, goal.x - x)
    
    temp = None
    if d < r:
        temp = (0.0, 0.0)
    elif r <= d <= s+r:
        temp = (al*(d-r)*math.cos(theta), al*(d-r)*math.sin(theta))
    elif d > s+r:
        temp = (al*s*math.cos(theta), al*s*math.sin(theta))

    return temp


####################################################################
####################################################################
####################################################################
####################################################################
class Agent(object):
    """Class handles all command and control logic for a teams tanks."""

    def __init__(self, bzrc):
        self.bzrc = bzrc
        self.constants = self.bzrc.get_constants()
        self.commands = []

        self.kp = 0.60
        self.kd = 0.50

        self.current_x_goal = 0
        self.current_y_goal = 400

        self.tank_error = 0        
        self.tank_time = 0
        self.my_tanks = []
        self.other_tanks = []
        self.flags = []
        self.shots = []

    def tick(self, time_diff):
        """Some time has passed; decide what to do next."""
        my_tanks, other_tanks, flags, shots = self.bzrc.get_lots_o_stuff()
        self.my_tanks = my_tanks
        self.other_tanks = other_tanks
        self.flags = flags
        self.shots = shots
        self.commands = []

        if len(self.my_tanks) > 0:
            self.send_to_fly(self.my_tanks[0], time_diff)

        results = self.bzrc.do_commands(self.commands)

    ####################################################################
    # Determines the current goal for the dunk: fly dunk back and forth
    # between (0, 350) and (0,-350) -> up and down the middle of the
    # map.
    ####################################################################
    def get_fly_goal(self, tank):
        close_enough_offset = 70

        if (tank.y + close_enough_offset) >= 400:
            self.current_y_goal = -400
        elif (tank.y - close_enough_offset) <= -400:
            self.current_y_goal = 400

        return AGoal(self.current_x_goal, self.current_y_goal)

    ####################################################################
    # Send this dunk agent back and forth across the map.
    ####################################################################
    def send_to_fly(self, tank, time_diff):
        
        goal = self.get_fly_goal(tank)

        delta_position = gen_an_attractive_field(tank.x, tank.y, goal)
        
        new_theta = math.atan2(delta_position[1], delta_position[0])
        
        new_theta = new_theta + 2 * math.pi if new_theta < 0 else new_theta
        pos_tank_angle = tank.angle + 2 * math.pi if tank.angle < 0 else tank.angle
        
        error = new_theta - pos_tank_angle
        
        error = error - 2 * math.pi if error > math.pi else error
        
        derivative = (error - self.tank_error) / (time_diff - self.tank_time)
      
        new_angle_velocity = (self.kp * error) + (self.kd * derivative)
        
        temp_angle = math.fabs(new_angle_velocity)
        if temp_angle >= 1:
            speed = 0.0
        else:
            speed = 1.0 - temp_angle
        
        fly_command = Command(tank.index, speed, new_angle_velocity, True)
        self.commands.append(fly_command)
        
        self.tank_error = error
        self.tank_time = time_diff
        
        return 


def main():
    # Process CLI arguments.
    try:
        execname, host, port = sys.argv
    except ValueError:
        execname = sys.argv[0]
        print >>sys.stderr, '%s: incorrect number of arguments' % execname
        print >>sys.stderr, 'usage: %s hostname port' % sys.argv[0]
        sys.exit(-1)

    # Connect.
    #bzrc = BZRC(host, int(port), debug=True)
    bzrc = BZRC(host, int(port))

    agent = Agent(bzrc)

    prev_time = time.time()

    # Run the agent
    try:
        while True:
            time_diff = time.time() - prev_time
            agent.tick(time_diff)
    except KeyboardInterrupt:
        print "Exiting due to keyboard interrupt."
        bzrc.close()


if __name__ == '__main__':
    main()

# vim: et sw=4 sts=4
