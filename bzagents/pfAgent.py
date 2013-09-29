#!/usr/bin/python -tt

import sys
import math
import time

from myPrint import *
from bzrc import BZRC, Command

###########################Potential Field Fun############################################

####################################################################
# Distance between two points.
####################################################################
def distance(x,y,goal):
    return math.sqrt(((goal.y - y)*(goal.y - y)) + ((goal.x - x)*(goal.x - x)))
def distancePoints(x,y,xg,yg):
    return math.sqrt(((yg - y)*(yg - y)) + ((xg - x)*(xg - x)))
   
def sign(a):
    if a == 0 or a == -0:
        return 0
    return a / -a

####################################################################
# Generate a Single Repulsive feild.
####################################################################
def generateAnRepulsiveField(x,y, obsticle, makeItTangent=False):
    r  = distancePoints(obsticle[0][0],
                        obsticle[0][1],
                        obsticle[2][0],
                        obsticle[2][1]) / 2.0
    center = (obsticle[0][0] + ((obsticle[2][0] - obsticle[0][0]) / 2.0),
              obsticle[0][1] + ((obsticle[2][1] - obsticle[0][1]) / 2.0))
    s  = 40.0
    b = 1.0/s
    
    d     = distancePoints(x,y,center[0], center[1])
    theta = math.atan2(center[1] - y, center[0] - x)
    
    if makeItTangent:
        theta -= (math.pi / 2.0)
    
    temp = None
    if d < r:
        temp = (-math.cos(theta) * s, -math.sin(theta) * s)
    elif r <= d and d <= s+r:
        temp = (-b * (s + r -d)*math.cos(theta), -b * (s+r-d) * math.sin(theta))
    elif d > s+r:
        temp = (0,0)
    
    return temp
    
####################################################################
# Calculate repulsive fields on a given location.
####################################################################
def generateRepulsiveField(x, y, obsticles):
    total = [0,0]
    
    for o in obsticles:
        temp = generateAnRepulsiveField(x,y,o)
        total[0] += temp[0]
        total[1] += temp[1]
        
    return total

####################################################################
# Generate a single atractive vector.
####################################################################
def genAnAttractiveField(x, y, goal):
    r  = 10.0
    s  = 100.0
    al = 1.0/s
    
    d = distance(x,y,goal)
    
    theta = math.atan2(goal.y - y, goal.x - x)
    
    temp = None
    if d < r:
        temp = (0.0,0.0)
    elif r <= d and d <= s+r:
        temp = (al*(d-r)*math.cos(theta), al*(d-r)*math.sin(theta))
    elif d > s+r:
        temp = (al*s*math.cos(theta), al*s*math.sin(theta))

    return temp

####################################################################
# Genertes the attractive vector given every possible goal.
####################################################################
def generateAttractiveField(x, y, goals):
    total = [0,0]
    
    amin = distance(x,y,goals[0])
    minGoal = goals[0]
    
    for g in goals:
        temp = distance(x,y,g)
        if temp < amin:
            minGoal = g
            amin = temp
    
    return genAnAttractiveField(x,y,minGoal)

####################################################################
# Calculate a Tangential field
####################################################################
def generateTangentialFields(x, y, obsticles):
    total = [0,0]
    
    for o in obsticles:
        temp = generateAnRepulsiveField(x, y, o, True)
        total[0] += temp[0]
        total[1] += temp[1]
        
    return total

####################################################################
# Generate the potential field for a given point.
####################################################################
def generatePotentialField(x,y,flags,obsticles):
    
    tan = generateTangentialFields(x,y,obsticles)
    att = generateAttractiveField(x,y,flags)
    rep = generateRepulsiveField(x,y,obsticles)
    
    return (tan[0] + att[0] + rep[0],
            tan[1] + att[1] + rep[1])




####################################################################
####################################################################
## Calculate a Tangential field
##
####################################################################
####################################################################
class Agent(object):
    """Class handles all command and control logic for a teams tanks."""

    ####################################################################
    # Constructor
    ####################################################################
    def __init__(self, bzrc):
        self.bzrc = bzrc
        self.constants = self.bzrc.get_constants()
        self.commands = []

    ####################################################################
    # Set command to move to given coordinates.
    ####################################################################
    def move_to_position(self, tank, target_x, target_y):
        target_angle = math.atan2(target_y - tank.y,
                                  target_x - tank.x)
        relative_angle = self.normalize_angle(target_angle - tank.angle)
        command = Command(tank.index, 1, 2 * relative_angle, True)
        self.commands.append(command)

    ####################################################################
    # Make any angle be between +/- pi.
    ####################################################################
    def normalize_angle(self, angle):
        angle -= 2 * math.pi * int (angle / (2 * math.pi))
        if angle <= -math.pi:
            angle += 2 * math.pi
        elif angle > math.pi:
            angle -= 2 * math.pi
        return angle

    ####################################################################
    ####################################################################
    def removeMyFlag(self, flags):
        temp = None
        for f in flags:
            if f.color == self.constants['team']:
                temp = f
            else:
                print f.color, " ", f.x, ":", f.y
        flags.remove(f)
        return flags

    ####################################################################
    # Make any angle be between +/- pi.
    ####################################################################
    def printPFields(self):
        obsticles = self.bzrc.get_obstacles()
        flags     = self.removeMyFlag(self.bzrc.get_flags())
        
        #printer = PFPrinter('aFields.gpi')
        #printer.printObsticles(obsticles)        
        #printer.printPotentialFields(lambda x,y: generateAttractiveField(x, y,flags))
        
        #printer = PFPrinter('rFields.gpi')
        #printer.printObsticles(obsticles)        
        #printer.printPotentialFields(lambda x,y: generateRepulsiveField(x, y, obsticles))
        
        #printer = PFPrinter('tFields.gpi')
        #printer.printObsticles(obsticles)        
        #printer.printPotentialFields(lambda x,y: generateTangentialFields(x, y, obsticles))

        printer = PFPrinter('pFields.gpi')
        printer.printObsticles(obsticles)        
        printer.printPotentialFields(lambda x,y: generatePotentialField(x, y, flags, obsticles))
        

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

    

    #prev_time = time.time()
    #
    ## Run the agent
    #try:
    #    while True:
    #        time_diff = time.time() - prev_time
    #        agent.tick(time_diff)
    #except KeyboardInterrupt:
    #    print "Exiting due to keyboard interrupt."
    bzrc.close()


if __name__ == '__main__':
    if len(sys.argv) == 4:
        execname, host, port, printMe = sys.argv

        if printMe == "-p":
            bzrc = BZRC(host, int(port))
            agent = Agent(bzrc)
            agent.printPFields()
    
    #main()

# vim: et sw=4 sts=4
