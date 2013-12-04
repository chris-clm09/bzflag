__author__ = 'crunk'

import sys
from bzrc import *
import numpy as np
from numpy.matrixlib import matrix
from numpy.linalg import matrix_power
import matplotlib
import matplotlib.mlab as mlab
import matplotlib.pyplot as plt
from math import *

plt.ion()
fig = plt.figure()  # only use one figure
plt.title('Kalman Filter Density')
plt.subplot(111, autoscale_on=False, xlim=(-400, 400), ylim=(-400, 400))

###########################Constants And Stuff#############################
mu_not = matrix('0;\
                 0;\
                 0;\
                 0;\
                 0;\
                 0')

sigma_not = matrix('100  0   0    0   0   0  ;\
                      0  0.1 0    0   0   0  ;\
                      0  0   0.1  0   0   0  ;\
                      0  0   0  100   0   0  ;\
                      0  0   0    0   0.1 0  ;\
                      0  0   0    0   0   0.1')

I = matrix('1 0 0 0 0 0;'
           '0 1 0 0 0 0;'
           '0 0 1 0 0 0;'
           '0 0 0 1 0 0;'
           '0 0 0 0 1 0;'
           '0 0 0 0 0 1')

H = matrix('1 0 0 0 0 0;\
            0 0 0 1 0 0')

sigma_z = matrix('25  0;\
                   0 25')

sigma_x = matrix('0.1 0   0    0   0   0  ;\
                  0   0.1 0    0   0   0  ;\
                  0   0   100  0   0   0  ;\
                  0   0   0    0.1 0   0  ;\
                  0   0   0    0   0.1 0  ;\
                  0   0   0    0   0   100')


def F(delta_t=0.5):
    c = -0.1
    # c = 0.0

    m =     '1 ' + str(delta_t) + ' ' + str(pow(delta_t, 2)/2.0) + ' 0 0 0;'
    m = m + '0 1 ' + str(delta_t) + ' 0 0 0;'
    m = m + '0 ' + str(c) + ' 1 0 0 0;'
    m = m + '0 0 0 1 ' + str(delta_t) + ' ' + str(pow(delta_t, 2)/2.0) + ';'
    m = m + '0 0 0 0 1 ' + str(delta_t) + ';'
    m = m + '0 0 0 0 ' + str(c) + ' 1'
    return matrix(m)

#-----------------------------END--------------------------------
#----------------------------------------------------------------

########################################################################
########################################################################
class Agent(object):
    def __init__(self, bzrc):
        self.bzrc = bzrc
        my_tanks, other_tanks, flags, shots = self.bzrc.get_lots_o_stuff()

        self.constants = self.bzrc.get_constants()
        print self.constants
        self.commands = []
        self.tanks = []
        self.hunter = None
        self.enemies = []
        self.enemy_flag = None
        for flag in flags:
            if flag.color != self.constants['team']:
                self.enemy_flag = flag
        self.target = None
        self.kalman_vars = {}
        self.init_kalman()
        self.ave_time_diff = 0
        self.ave_time_diff_samples = 0

        self.error0 = 0
        self.time_set = 0
        self.kp = 0.60
        self.kd = 0.50

        self.first_hitable_location = 0


    ########################################################################
    ########################################################################
    def tick(self, time_diff):
        # calculate iterative average of time_diff -- used for future prediction
        self.ave_time_diff = self.ave_time_diff_samples * self.ave_time_diff + time_diff
        self.ave_time_diff_samples += 1
        self.ave_time_diff /= self.ave_time_diff_samples

        # Update agent's state
        my_tanks, other_tanks, flags, shots = self.bzrc.get_lots_o_stuff()
        self.tanks = my_tanks
        if self.hunter is None:
            self.hunter = self.tanks[0]
        self.enemies = [tank for tank in other_tanks if tank.color !=
                        self.constants['team']]
        if len(self.enemies) > 0:
            self.target = self.enemies[0]  # this assumes that tank[0] will continue to be tank[0] until it is killed
            self.kalman_update(time_diff)
            self.plot_kalman()
        else:  # we must have killed the target tank
            self.target = None
            self.init_kalman()

    ########################################################################
    ########################################################################
    def init_kalman(self):
        global mu_not, sigma_not
        # initial mean should be around the enemy flag
        initial_mu = mu_not + matrix([[self.enemy_flag.x],
                                      [0],
                                      [0],
                                      [self.enemy_flag.y],
                                      [0],
                                      [0]])
        self.kalman_vars['mu'] = initial_mu
        self.kalman_vars['sigma'] = sigma_not

    ########################################################################
    ########################################################################
    def kalman_update(self, time_diff):
        global H, I, sigma_x, sigma_z
        _F = F(time_diff)
        tmp = _F*self.kalman_vars['sigma']*_F.T + sigma_x
        k = tmp*H.T*(H*tmp*H.T + sigma_z).I
        z = matrix([[self.target.x], [self.target.y]])  # our observation
        new_mu = _F*self.kalman_vars['mu'] + k*(z - H*_F*self.kalman_vars['mu'])
        new_sigma = (I - k*H)*tmp
        self.kalman_vars['mu'] = new_mu
        self.kalman_vars['sigma'] = new_sigma
        # print self.kalman_vars['mu']
        # print self.kalman_vars['sigma']

    ########################################################################
    ########################################################################
    def predict_target_future_mu(self, time_steps):
        _F = F(self.ave_time_diff)
        return matrix_power(_F, time_steps)*self.kalman_vars['mu']

    ########################################################################
    ########################################################################
    def plot_kalman(self):
        #print self.kalman_vars['mu']
        #print self.kalman_vars['sigma']
        s_tx = self.kalman_vars['sigma'][0, 0]
        s_ty = self.kalman_vars['sigma'][3, 3]
        s_txy = self.kalman_vars['sigma'][3, 0]
        mu_tx = self.kalman_vars['mu'][0, 0]
        mu_ty = self.kalman_vars['mu'][3, 0]

        size = int(self.constants['worldsize'])
        delta = 1
        x = np.arange(-size/2, size/2, delta)
        y = np.arange(-size/2, size/2, delta)
        X, Y = np.meshgrid(x, y)
        Z = mlab.bivariate_normal(X, Y, s_tx, s_ty, mu_tx, mu_ty, s_txy)
        plt.clf()
        plt.pcolormesh(X, Y, Z, cmap='hot')
        plt.draw()





    ########################################################################
    ########################################################################
    def get_angular_error_to_location(self,x,y):
        new_theta = math.atan2(y, x)
        
        new_theta      = new_theta  + 2 * math.pi if new_theta  < 0 else new_theta
        pos_tank_angle = tank.angle + 2 * math.pi if tank.angle < 0 else tank.angle
        
        error = new_theta - pos_tank_angle
        
        error = error - 2 * math.pi if error > math.pi else error

        return error

    ########################################################################
    ########################################################################
    def get_percent_of_angular_velocity(self,error):
        derivative = (error - self.error0) / (time_diff - self.time_set)
      
        new_angle_velocity = (self.kp * error) + (self.kd * derivative)

        return new_angle_velocity

    ########################################################################
    ########################################################################
    def get_duck_travel_time(self,travel_to_x,travel_to_y):
        duck_x = self.kalman_vars['mu'][0][0]
        duck_v_x = self.kalman_vars['mu'][1][0]

        return (travel_to_x - duck_x) / float(duck_v_x)

    ########################################################################
    ########################################################################
    def get_tank_rotation_time_to_pos(self, x, y):
        error_thea = get_angular_error_to_location(x,y)
        angular_velocity     = self.constants['tankangvel']
        angular_acceleration = self.constants['angularaccel']

        #Quadratic Equasion to solve
        # 0 = thea_error + ang_v * d_time + 1/2 ang_accel d_time^2
        sol_one = (-angular_velocity + sqrt(pow(angular_velocity,2.0) + 2 * angular_acceleration * error_thea)) / float(angular_acceleration)
        sol_two = (-angular_velocity - sqrt(pow(angular_velocity,2.0) + 2 * angular_acceleration * error_thea)) / float(angular_acceleration)

        time = sol_two if sol_one < 0 else None

        if time:
            if time < 0:
                raise NameError("get_tank_rotation_time_to_pos: Both sol are negative?")
            else:
                return time

        time = sol_one if sol_one > sol_two else sol_two

        if time < 0:
            raise NameError("get_tank_rotation_time_to_pos: Found a negative solution?")

        return time

    ########################################################################
    ########################################################################
    def get_shot_travel_time_to_pos(self,x,y):
        distance = sqrt(pow(x - self.hunter.x,2.0) + pow(y - self.hunter.y, 2.0))
        return distance / float(self.constants['shotspeed'])

    ########################################################################
    ########################################################################
    def can_hit_location(self,x,y):
        dunk_travel_time  = self.get_duck_travel_time(x,y)
        angle_change_time = get_tank_rotation_time_to_pos(x,y)
        shot_travel_time  = get_shot_travel_time_to_pos(x,y)
        reload_time       = 0
        
        error = abs(dunk_travel_time - (angle_change_time+shot_travel_time+reload_time))

        if error < .1:
            print 'Found Hit:', x,y

        return error <= .1

    ########################################################################
    ########################################################################
    def find_first_hitable_location(self):

        return


    ########################################################################
    ########################################################################
    def fire_on_location(self, x,y,time_diff):

        capture_flag_command = Command(tank.index, speed, new_angle_velocity, True)
        self.commands.append(capture_flag_command)
        
        self.error0[tank.index] = error
        self.time_set[tank.index] = time_diff
        
        return





########################################################################
########################################################################
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


########################################################################
########################################################################
if __name__ == '__main__':
    main()
    #plt.ioff()
    #plt.show()