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
def generateAnRepulsiveField(x,y, obsticle, makeItTangent=False, goal=None):
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
    
    dx = -math.cos(theta)
    dy = -math.sin(theta)
    
    if makeItTangent:
        thetaL = theta - (math.pi / 2.0)
        thetaR = theta + (math.pi / 2.0)
        
        dxL = -math.cos(thetaL)
        dyL = -math.sin(thetaL)
        
        dxR = -math.cos(thetaR)
        dyR = -math.sin(thetaR)
        
        if distancePoints(x + dxL, y + dyL, goal.x, goal.y) < distancePoints(x+dxR,y+dyR,goal.x,goal.y):
            dx = dxL
            dy = dyL
        else:
            dx = dxR
            dy = dyR
        
    
    temp = None
    if d < r:
        temp = (dx * s, dy * s)
    elif r <= d and d <= s+r:
        temp = (b * (s + r -d) * dx, b * (s + r - d) * dy)
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
    r  = 1.5
    s  = 30.0
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
# Return the closest goal.
####################################################################
def getMinGoal(x,y,goals):
    amin = distance(x,y,goals[0])
    minGoal = goals[0]
    
    for g in goals:
        temp = distance(x,y,g)
        if temp < amin:
            minGoal = g
            amin = temp
    
    return minGoal

####################################################################
# Genertes the attractive vector given every possible goal.
####################################################################
def generateAttractiveField(x, y, goals):
    total = [0,0]
    
    minGoal = getMinGoal(x,y,goals)
    
    return genAnAttractiveField(x,y,minGoal)

####################################################################
# Calculate a Tangential field
####################################################################
def generateTangentialFields(x, y, obsticles, goal):
    total = [0,0]
    
    for o in obsticles:
        temp = generateAnRepulsiveField(x, y, o, True, goal)
        total[0] += temp[0]
        total[1] += temp[1]
        
    return total

####################################################################
# Generate the potential field for a given point.
####################################################################
def generatePotentialField(x,y,flags,obsticles):
    
    tan = generateTangentialFields(x,y,obsticles, getMinGoal(x,y,flags))
    att = generateAttractiveField(x,y,flags)
    rep = generateRepulsiveField(x,y,obsticles)
    
    return (tan[0] + att[0] + rep[0],
            tan[1] + att[1] + rep[1])


class HomeBaseCenter(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y

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
        self.obsticles = self.bzrc.get_obstacles()
        self.commands = []
        self.error0   = 0
        
        bases = self.bzrc.get_bases()
        for base in bases:
            if base.color == self.constants['team']:
                self.homeBase = base
        
        self.homeBaseCenter = HomeBaseCenter(base.corner1_x + ((base.corner3_x - base.corner1_x) / 2.0),
                                             base.corner1_y + ((base.corner3_y - base.corner1_y) / 2.0))

    ####################################################################
    ####################################################################
    def tick(self, time_diff):
        mytanks, othertanks, flags, shots = self.bzrc.get_lots_o_stuff()
        
        self.mytanks    = mytanks
        self.othertanks = othertanks
        self.flags      = self.removeMyFlag(flags)
        self.shots      = shots
        self.enemies    = [tank for tank in othertanks
                           if tank.color != self.constants['team']]

        #Clear Commands
        self.commands = []

        for tank in mytanks:
            #if tank.index == 0:
            #    self.sendToCaptureFlag(tank, time_diff)
            self.sendToCaptureFlag(tank, time_diff)
            #self.attack_enemies(tank)

        results = self.bzrc.do_commands(self.commands)

    ####################################################################
    ####################################################################
    def determinedGoals(self, tank):
        if tank.flag == '-':
            for f in self.flags:
                print f.color, " "
            print ""
            return self.flags
        else:
            return [self.homeBaseCenter]

    def generateHomePotentialField(self,x,y):
        return generatePotentialField(x,y,[self.homeBaseCenter],self.obsticles)

    ####################################################################
    ####################################################################
    def sendToCaptureFlag(self, tank, time_diff):
        self.Kp = 0.50
        self.Kd = 0.60
        
        deltaPosition = generatePotentialField(tank.x, tank.y,
                                               self.determinedGoals(tank),
                                               self.obsticles)
        newTheta      = math.atan2(deltaPosition[1], deltaPosition[0])
        
        #newTheta = self.normalize_angle(newTheta)
        
        error = newTheta - tank.angle
        
        derivative       = error - self.error0
        newAngleVelocity = (self.Kp * error) + (self.Kd * derivative)
        
        speed = math.sqrt(math.pow(deltaPosition[0], 2) +
                          math.pow(deltaPosition[1], 2))
        
        tempAngle = math.fabs(newAngleVelocity)
        if tempAngle >= 1:
            speed = 0.5
            #print tank.x, ":", tank.y, " dPos: ", deltaPosition[0], ":", deltaPosition[1], " V: ", newAngleVelocity, " Goal: ", self.determinedGoals(tank)[0].x, ":", self.determinedGoals(tank)[0].y
        else:
            speed = 1.0 - tempAngle
        
        captureFlagCommand = Command(tank.index, speed, newAngleVelocity, True)
        self.commands.append(captureFlagCommand)
        
        self.error0 = error
        
        return 

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
    # Remove my flag from the list.
    ####################################################################
    def removeMyFlag(self, flags):
        temp = None
        print "Remove me:"
        for f in flags:
            print f.color, "==", self.constants['team']

            if f.color == self.constants['team']:
                temp = f
....

        flags.remove(f)
        return flags
    
    ####################################################################
    # Return all of the flags in the game save my own.
    ####################################################################
    def getTargetFlags(self):
        return self.removeMyFlag(self.bzrc.get_flags())
    
    ####################################################################
    # Make any angle be between +/- pi.
    ####################################################################
    def printPFields(self):
        obsticles = self.bzrc.get_obstacles()
        flags     = self.getTargetFlags()
        
        #printer = PFPrinter('aFields.gpi')
        #printer.printObsticles(obsticles)        
        #printer.printPotentialFields(lambda x,y: generateAttractiveField(x, y,flags))
        
        #printer = PFPrinter('rFields.gpi')
        #printer.printObsticles(obsticles)        
        #printer.printPotentialFields(lambda x,y: generateRepulsiveField(x, y, obsticles))
        
        #printer = PFPrinter('tFields.gpi')
        #printer.printObsticles(obsticles)        
        #printer.printPotentialFields(lambda x,y: generateTangentialFields(x, y, obsticles))

        printer = PFPrinter('homeFields.gpi')
        printer.printObsticles(obsticles)
        printer.printPotentialFields(lambda x,y: self.generateHomePotentialField(x, y))

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

    prev_time = time.time()
    
    # Run the agent
    try:
        while True:
            time_diff = time.time() - prev_time
            prev_time = time.time()
            agent.tick(time_diff)
    except KeyboardInterrupt:
        print "Exiting due to keyboard interrupt."
    bzrc.close()


if __name__ == '__main__':
    if len(sys.argv) == 4:
        execname, host, port, printMe = sys.argv

        if printMe == "-p":
            bzrc = BZRC(host, int(port))
            agent = Agent(bzrc)
            agent.printPFields()
            bzrc.close()
            
    else:
        main()

# vim: et sw=4 sts=4
