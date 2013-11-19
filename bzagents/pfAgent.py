#!/usr/bin/python -tt

import sys
import math
import time

from myPrint import *
from common import *
from tank import *
from bzrc import BZRC, Command
from grid_example import *

import time
###########################Fun Agent Constants############################################

INITIAL_WORLD_CELL_PROBABILITY = 0.4

########################END Fun Agent Constants END#######################################


###########################Potential Field Fun############################################

####################################################################
# Generate a Single Repulsive field.
####################################################################
def generate_a_repulsive_field(x, y, obstacle, make_it_tangent=False, goal=None):
    r = 20
    center = obstacle
    s = 40.0
    b = 1.0/s
    
    d = distance_coords(x, y, center[0], center[1])
    theta = math.atan2(center[1] - y, center[0] - x)
    
    dx = -math.cos(theta)
    dy = -math.sin(theta)
    
    if make_it_tangent:
        # theta_l = theta - (math.pi / 2.0)
        theta_r = theta + (math.pi / 2.0)
        
        # dx_l = -math.cos(theta_l)
        # dy_l = -math.sin(theta_l)
        
        dx_r = -math.cos(theta_r)
        dy_r = -math.sin(theta_r)
        
        # if distance_coords(x + dx_l, y + dy_l, goal.x, goal.y) < distance_coords(x+dx_r, y+dy_r, goal.x, goal.y):
        #     dx = dx_l
        #     dy = dy_l
        # else:
        dx = dx_r
        dy = dy_r

    temp = None
    if d < r:
        temp = (dx * s, dy * s)
    elif r <= d and d <= s+r:
        temp = (b * (s + r - d) * dx, b * (s + r - d) * dy)
    elif d > s+r:
        temp = (0, 0)
    
    return temp


####################################################################
# Calculate repulsive fields on a given location.
####################################################################
def generate_repulsive_field(x, y, obstacles):
    total = [0, 0]
    
    for o in obstacles:
        temp = generate_a_repulsive_field(x, y, o)
        total[0] += temp[0]
        total[1] += temp[1]
        
    return total


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
    elif r <= d and d <= s+r:
        temp = (al*(d-r)*math.cos(theta), al*(d-r)*math.sin(theta))
    elif d > s+r:
        temp = (al*s*math.cos(theta), al*s*math.sin(theta))

    return temp


####################################################################
# Return the closest goal.
####################################################################
def get_min_goal(x, y, goals):
    a_min = distance(x, y, goals[0])
    min_goal = goals[0]
    
    for g in goals:
        temp = distance(x, y, g)
        if temp < a_min:
            min_goal = g
            a_min = temp
    
    return min_goal


####################################################################
# Generates the attractive vector given every possible goal.
####################################################################
def generate_attractive_field(x, y, goals):
    min_goal = get_min_goal(x, y, goals)
        
    return gen_an_attractive_field(x, y, min_goal)


####################################################################
# Calculate a Tangential field
####################################################################
def generate_tangential_fields(x, y, obstacles, goal):
    total = [0, 0]
    
    for o in obstacles:
        temp = generate_a_repulsive_field(x, y, o, True, goal)
        total[0] += temp[0]
        total[1] += temp[1]
        
    return total


####################################################################
# Generate the potential field for a given point.
####################################################################
def generate_potential_field(x, y, goals, obstacles):
    tan = generate_tangential_fields(x, y, obstacles, get_min_goal(x, y, goals))
    att = generate_attractive_field(x, y, goals)
    rep = generate_repulsive_field(x, y, obstacles)
    
    return (tan[0] + att[0] + rep[0],
            tan[1] + att[1] + rep[1])


def generate_rep_field(x, y, obstacles):
    tan = generate_tangential_fields(x, y, obstacles, Point(0, 0))
    rep = generate_repulsive_field(x, y, obstacles)
    return (tan[0] + rep[0],
            tan[1] + rep[1])


####################################################################
####################################################################
## Calculate a Tangential field
####################################################################
####################################################################
class Agent(object):
    """Class handles all command and control logic for a team's tanks."""

    ####################################################################
    # Constructor
    ####################################################################
    def __init__(self, bzrc):
        my_tanks, other_tanks, flags, shots = bzrc.get_lots_o_stuff()
        self.bzrc = bzrc
        self.constants = self.bzrc.get_constants()
        self.obstacles = []

        self.commands = []
        self.error0 = 0
        #self.my_flag = self.get_my_flag(flags)
        self.my_tanks = None
        self.start = True
        self.update_my_tanks(my_tanks)
        #self.other_flags = None
        #self.shots = shots
        #self.enemies = None

        self.kp = 0.60
        self.kd = 0.50
        self.probability_map = None
        
        bases = self.bzrc.get_bases()
        for base in bases:
            if base.color == self.constants['team']:
                self.home_base = base
        
        self.home_base_center = Point(self.home_base.corner1_x + ((self.home_base.corner3_x - self.home_base.corner1_x) / 2.0),
                                      self.home_base.corner1_y + ((self.home_base.corner3_y - self.home_base.corner1_y) / 2.0))

        self.time_set = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.error0   = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        self.init_world_probability_map(int(self.constants['worldsize']))
        self.init_opengl_window()

        self.init_observer_sensor_probabilities()

    ####################################################################
    # Init observer sensor probabilities.
    ####################################################################
    def init_observer_sensor_probabilities(self):
        self.true_positive  = float(self.constants['truepositive'])
        self.true_negative  = float(self.constants['truenegative'])
        self.false_positive = 1.0 - self.true_positive
        self.false_negative = 1.0 - self.true_negative

    ####################################################################
    # Init Probability Map of the World
    ####################################################################
    def init_world_probability_map(self, square_size):
        #Should ensure square_size is an int.
        initial_probability = INITIAL_WORLD_CELL_PROBABILITY
        self.probability_map = []

        for r in range(0, square_size):
            self.probability_map.append([])
            for c in range(0, square_size):
                self.probability_map[r].append(initial_probability)

        self.half_of_square = square_size / 2

    ####################################################################
    # Init opengl window and draw initial probabilities.
    ####################################################################
    def init_opengl_window(self):
        size = int(self.constants['worldsize'])
        init_window(size, size)
        self.update_opengl_window()

    ####################################################################
    # Update opengl window with current probability view.
    ####################################################################
    def update_opengl_window(self):
        update_grid(self.probability_map)
        draw_grid()

    def update_my_tanks(self, bzrc_answers):
        num_tanks = len(bzrc_answers)
        if self.my_tanks is None:  # initialize Tanks
            self.my_tanks = []
            size = int(self.constants['worldsize'])
            extreme = size / 2
            mow_width = 50
            for i in range(num_tanks):
                j = 0
                waypoints = []
                col = -extreme + i * mow_width
                while col < extreme:
                    # vertical lawn mower style exploration
                    top = Point(col, extreme)
                    bot = Point(col, -extreme)
                    waypoints.append(top)
                    waypoints.append(bot)
                    col += num_tanks*mow_width
                self.my_tanks.append(Tank(waypoints))

        # update tank states
        for i in range(num_tanks):
            self.my_tanks[i].update_state(bzrc_answers[i])

    ####################################################################
    ####################################################################
    def tick(self, time_diff):
        #my_tanks, other_tanks, flags, shots = self.bzrc.get_lots_o_stuff()
        self.update_my_tanks(self.bzrc.get_mytanks())
        #self.my_flag = self.get_my_flag(flags)
        #self.other_tanks = other_tanks
        #self.other_flags = self.remove_my_flag(flags)
        #self.shots = shots
        #self.enemies = [tank for tank in other_tanks
        #                if tank.color != self.constants['team']]

        #Clear Commands
        self.commands = []

        if self.start == True:
            tanks = self.my_tanks
            self.start = False
        else:
            tanks = [self.my_tanks[3], self.my_tanks[4], self.my_tanks[5], self.my_tanks[0], self.my_tanks[8]]

        #MOVE EACH TANK
        for tank in tanks:
            self.generate_commands_from_tank(tank, time_diff)

        results = self.bzrc.do_commands(self.commands)

        #PERFORM OBSERVATION FROM EACH TANK
        self.perform_observation_for_each_tank(tanks)

        self.update_opengl_window()

    ####################################################################
    # This function will query the server for what each tank can see.
    ####################################################################
    def perform_observation_for_each_tank(self, tanks):

        obs = []

        #Perform observation commands
        for tank in tanks:
            pos, grid = self.bzrc.get_occgrid(tank.index)
            obs.append((pos, grid))

        self.update_given_observations(obs)
        return

    ####################################################################
    # This function will interpret and add each observation to the
    # world map.
    ####################################################################
    def update_given_observations(self, obs):

        #Process each Observation (tank)
        for o in obs:
            pos  = o[0]
            grid = o[1]

            #Process each cell in a tanks view range(occgrid)
            for r in range(0, len(grid)):
                for c in range(0, len(grid[0])):
                    
                    temp_pos_wp     = (pos[0] + r, pos[1] + c)
                    temp_pos_raster = self.get_me_raster_x_and_y_from_world_pos(temp_pos_wp)
                    
                    the_observation = int(grid[r][c])

                    self.update_given_obs(
                        temp_pos_wp,
                        temp_pos_raster, 
                        the_observation)

        return

    ####################################################################
    # This function will take an observation and update the raster
    # probability given the observation.
    ####################################################################
    def update_given_obs(self, wp, pos, obs):

        x = pos[0]
        y = pos[1]

        if x >= len(self.probability_map[0]) or y >= len(self.probability_map):
            print "Attempted update out of bounds: ", pos
            return

        p_something_there = None
        p_nothings_there  = None

        current_probability = self.probability_map[y][x]

        if obs == 1:
            p_something_there = self.true_positive  * current_probability
            p_nothings_there  = self.false_positive * current_probability
        elif obs == 0:
            p_something_there = self.false_negative * current_probability
            p_nothings_there  = self.true_negative  * current_probability
        else:
            print "\n\tupdateProbabilityInRasterGivenObj:",\
                  "Wow! What in Sam Hill did you observe!\n",\
                  self, pos, obs
            return

        n = p_something_there + p_nothings_there
        if n > 0:
            p_something_there /= n
            self.probability_map[y][x] = p_something_there
            self.add_obstacle_if_needed(wp, x, y, p_something_there)

        return

    ####################################################################
    # Sets a point as an obstacle given a high enough p_something_there
    # and if 50% of his neighbors are obstacles.
    ####################################################################
    def add_obstacle_if_needed(self, wp, x, y, p_something_there):
        if p_something_there >= .94 and self.might_be_obstacle(x, y):
            
            #if any obsticle is within this new point(wp)'s grid
            #don't add it to the obsticle list.
            for pt in self.obstacles:
                if pt[0] >= wp[0] - 15 and pt[0] <= wp[0] + 15 and \
                   pt[1] >= wp[1] - 15 and pt[1] <= wp[1] + 15:
                   # print "Skipping"
                   return

            # if not wp in self.obstacles:
            self.obstacles.append(wp)
            print "Ob. Add:", wp, (x,y), len(self.obstacles)

    ####################################################################
    # Returns true if 50% of a points neighbors are obstacles.
    ####################################################################
    def might_be_obstacle(self, x, y):
        if x >= len(self.probability_map[0]) or y >= len(self.probability_map):
            return False

        cnt_of_surrounding_obstacles = 0
        for xp in range(x-2, x+3):
            for yp in range(y-2, y+3):
                if xp < len(self.probability_map[0]) and yp < len(self.probability_map):
                    if self.probability_map[yp][xp] > .9:
                        cnt_of_surrounding_obstacles += 1

        return (cnt_of_surrounding_obstacles / 25.0) >= .7
       
    ####################################################################
    # Returns raster coordinates given world coordinates.
    ####################################################################
    def get_me_raster_x_and_y_from_world_pos(self, pos):
        return (self.half_of_square + pos[0],
                self.half_of_square - pos[1])

    ####################################################################
    ####################################################################
    #def determined_goals(self, tank):
    #    result = self.ignore_flags_we_carry(self.other_flags)
    #    if not self.is_our_flag_at_home():
    #        result.append(self.my_flag)  # someone moved our flag, retrieving it is a possible goal
    #
    #    if tank.flag != '-' or len(result) == 0:
    #        return [self.home_base_center]  # go home if we have nothing else to do
    #    else:
    #        return result

    def generate_home_potential_field(self, x, y):
        return generate_potential_field(x, y, [self.home_base_center], self.obstacles)

    ####################################################################
    ####################################################################
    def generate_commands_from_tank(self, tank, time_diff):
        delta_position = generate_potential_field(tank.x, tank.y,
                                                  #self.determined_goals(tank),
                                                  [tank.get_waypoint()],
                                                  self.obstacles)

        new_theta = math.atan2(delta_position[1], delta_position[0])
        
        new_theta = new_theta + 2 * math.pi if new_theta < 0 else new_theta
        pos_tank_angle = tank.angle + 2 * math.pi if tank.angle < 0 else tank.angle
        
        error = new_theta - pos_tank_angle
        
        error = error - 2 * math.pi if error > math.pi else error
        
        derivative = (error - self.error0[tank.index]) / (time_diff - self.time_set[tank.index])
      
        new_angle_velocity = (self.kp * error) + (self.kd * derivative)
        
        temp_angle = math.fabs(new_angle_velocity)
        if temp_angle >= 1:
            speed = 0.0
        else:
            speed = 1.0 - temp_angle
        
        capture_flag_command = Command(tank.index, speed, new_angle_velocity, True)
        self.commands.append(capture_flag_command)
        
        self.error0[tank.index] = error
        self.time_set[tank.index] = time_diff
        
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
        angle -= 2 * math.pi * int(angle / (2 * math.pi))
        if angle <= -math.pi:
            angle += 2 * math.pi
        elif angle > math.pi:
            angle -= 2 * math.pi
        return angle

    ####################################################################
    # Get my flag from the list.
    ####################################################################
    def get_my_flag(self, flags):
        for f in flags:
            if f.color == self.constants['team']:
                return f
        return None

    ####################################################################
    # Remove my flag from the list.
    ####################################################################
    def remove_my_flag(self, flags):
        temp = None
        for f in flags:
            if f.color == self.constants['team']:
                temp = f

        flags.remove(temp)
        return flags

    ####################################################################
    # Do not consider the flags we are currently carrying
    ####################################################################
    def ignore_flags_we_carry(self, flags):
        for f in flags:
            for t in self.my_tanks:
                if t.flag == f.color:
                    flags.remove(f)   # concurrency issue?
        return flags

    ####################################################################
    # Has our flag been moved from home base?
    ####################################################################
    def is_our_flag_at_home(self):
        x2 = (self.home_base_center.x - self.my_flag.x)**2
        y2 = (self.home_base_center.y - self.my_flag.y)**2
        radius = math.fabs(self.home_base.corner3_x - self.home_base.corner1_x) / 2.0
        dist = math.sqrt(x2+y2)
        return dist <= radius

    ####################################################################
    # Return all of the flags in the game save my own.
    ####################################################################
    def get_target_flags(self):
        return self.remove_my_flag(self.bzrc.get_flags())
    
    ####################################################################
    # Make any angle be between +/- pi.
    ####################################################################
    def print_pfields(self):
        # obstacles = self.obstacles #self.bzrc.get_obstacles()
        # flags = self.get_target_flags()

        # print self.constants
        # time.sleep(5)

        # a = self.bzrc.get_occgrid(1)
        # print a

        printer = PFPrinter('repFields.gpi')
        printer.printObstacles(self.bzrc.get_obstacles())
        temp = [(-208, 132), (-208, 148), (-192, 132), (-192, 148), (-149, 200), (-133, 200), (-176, 133), (-175, 149), (-160, 133), (-208, 111), (-192, 111), (-176, 112), (-9, 50), (-8, 27), (7, 43), (8, 25), (-117, 199), (-148, 172), (-132, 172), (7, 59), (-209, -102), (-208, -121), (-208, -148), (-9, 7), (-106, -95), (-98, -138), (-98, -122), (-90, -92), (-192, -148), (-192, -132), (-91, -181), (-91, -165), (-192, -116), (-176, -148), (-176, -132), (-160, -148), (-160, -132), (-148, -204), (-148, -188), (-148, -172), (-144, -156), (-144, -140), (-144, -124), (-132, -204), (-132, -188), (-132, -172), (-128, -156), (-128, -140), (-128, -124), (-176, -116), (-160, -116), (-144, -108), (-128, -108), (-116, -203), (-116, -187), (-112, -171), (-112, -155), (-193, -100), (-183, 91), (-167, 91), (-160, 107), (-151, 91), (-144, 107), (-135, 91), (-128, 107), (-119, 91), (-144, 123), (-128, 123), (-112, 107), (-112, 123), (-103, 91), (-96, 107), (-96, 123), (-93, -207), (-9, -55), (7, -58), (7, 1), (-8, -39), (-8, -23), (8, -42), (8, -26), (91, 186), (91, 202), (107, 184), (107, 200), (123, 183), (123, 199), (139, 183), (139, 199), (91, 166), (107, 165), (123, 165), (139, 165), (-104, 139), (91, -120), (91, -104), (91, 91), (91, 107), (91, 123), (107, 91), (107, 107), (90, 148), (106, 139), (107, 123), (123, 91), (123, 107), (123, 123), (139, 91), (139, 107), (155, 91), (155, 107), (122, 139), (171, 91), (171, 107), (139, 123), (155, 123), (187, 91), (187, 107), (171, 123), (187, 123), (203, 91), (203, 107), (203, 123), (107, -125), (107, -109), (107, -93), (123, -124), (123, -108), (123, -92), (91, -138), (123, -140), (207, 139)]
        printer.printPotentialFields(lambda x, y: generate_rep_field(x, y, temp))

        print "Done Printing"

        # printer = PFPrinter('homeFields.gpi')
        # # printer.printObstacles(obstacles)
        # printer.printPotentialFields(lambda x, y: self.generate_home_potential_field(x, y))

        # printer = PFPrinter('pFields.gpi')
        # printer.printObstacles(obstacles)        
        # printer.printPotentialFields(lambda x, y: generate_potential_field(x, y, flags, obstacles))
        

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
    
    old_diff = 0

    # Run the agent
    try:
        while True:
            time_diff = time.time()
            agent.tick(time_diff)
            
            if old_diff == 0:
                old_diff = time_diff
            elif time_diff - old_diff > 180:
                old_diff = time_diff
                agent.print_pfields()
                print "TICKY TOCKY"
                print agent.obstacles
                bzrc.close()
                return

    except KeyboardInterrupt:
        print "Exiting due to keyboard interrupt."
    bzrc.close()


if __name__ == '__main__':
    if len(sys.argv) == 4:
        execname, host, port, printMe = sys.argv

        if printMe == "-p":
            bzrc = BZRC(host, int(port))
            agent = Agent(bzrc)
            agent.print_pfields()
            bzrc.close()
            
    else:
        main()

# vim: et sw=4 sts=4