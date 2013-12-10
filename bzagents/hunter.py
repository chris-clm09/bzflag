__author__ = 'crunk'

import sys
from bzrc import *
from common import *
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

def sqr(number):
    return pow(number, 2.0)

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
                  0   0   .25  0   0   0  ;\
                  0   0   0    0.1 0   0  ;\
                  0   0   0    0   0.1 0  ;\
                  0   0   0    0   0   .25')


def F(delta_t=0.5):
    # c = -0.1
    c = 0.0

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
        self.time_set = 1
        self.kp = 1.5
        self.kd = 0.2

        self.last_time = 0

        self.first_hitable_location = None

    ########################################################################
    ########################################################################
    def tick(self, time_diff):
        # calculate iterative average of time_diff -- used for future prediction
        self.ave_time_diff = self.ave_time_diff_samples * self.ave_time_diff + time_diff
        self.ave_time_diff_samples += 1
        self.ave_time_diff /= self.ave_time_diff_samples

        delta_t = time_diff - self.last_time
        self.last_time = time_diff

        # Update agent's state
        my_tanks, other_tanks, flags, shots = self.bzrc.get_lots_o_stuff()
        self.tanks   = my_tanks
        self.hunter  = self.tanks[0]
        self.enemies = [tank for tank in other_tanks if tank.color !=
                        self.constants['team']]

        self.target = self.enemies[0]
        if abs(self.target.x) <= float(self.constants['worldsize']):  # when a tank is killed, it is warped to (-100k, -100k)
            self.kalman_update(delta_t)

            #if self.first_hitable_location is None:
            #    print 'Time:', time_diff
            #    self.first_hitable_location = self.find_first_hitable_location(time_diff)
            self.first_hitable_location = self.find_first_hitable_location_c(time_diff)
            self.fire_on_location(self.first_hitable_location[0],
                                  self.first_hitable_location[1], 
                                  time_diff)
            #else:
            #    self.fire_on_location(self.first_hitable_location[0],
            #                          self.first_hitable_location[1],
            #                          time_diff)

            self.plot_kalman()
        else:  # we must have killed the target tank
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
    def kalman_update(self, delta_t):
        global H, I, sigma_x, sigma_z
        print(self.kalman_vars['mu'])
        print(self.kalman_vars['sigma'])
        _F = F(delta_t)
        sigma_t = self.kalman_vars['sigma']
        mu_t = self.kalman_vars['mu']
        z_t_plus_one = matrix([[self.target.x], [self.target.y]])

        #tmp = _F*self.kalman_vars['sigma']*_F.T + sigma_x
        #k = tmp*H.T*(H*tmp*H.T + sigma_z).I
        #z = matrix([[self.target.x], [self.target.y]])  # our observation
        #print(z)
        #new_mu = _F*self.kalman_vars['mu'] + k*(z - H*_F*self.kalman_vars['mu'])
        #new_sigma = (I - k*H)*tmp
        #self.kalman_vars['mu'] = new_mu
        #self.kalman_vars['sigma'] = new_sigma

        K_t_plus_one = (_F*sigma_t*_F.T + sigma_x)*H.T*(H*(_F*sigma_t*_F.T + sigma_x)*H.T + sigma_z).I
        mu_t_plus_one = _F*mu_t + K_t_plus_one*(z_t_plus_one - H*_F*mu_t)
        sigma_t_plus_one = (I - K_t_plus_one*H)*(_F*sigma_t*_F.T + sigma_x)

        self.kalman_vars['mu'] = mu_t_plus_one
        self.kalman_vars['sigma'] = sigma_t_plus_one
        print(self.kalman_vars['mu'])
        print(self.kalman_vars['sigma'])

    ########################################################################
    ########################################################################
    def predict_target_future_mu(self, delta_t):
        return F(delta_t) * self.kalman_vars['mu']

    ########################################################################
    ########################################################################
    def plot_kalman(self):
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
        plt.pcolormesh(X, Y, Z, cmap='brg')  #'binary')  #'hot')
        plt.draw()

    ########################################################################
    ########################################################################
    def get_angular_error_to_location(self, x, y):
        tank = self.hunter

        new_theta = math.atan2(y - tank.y, x - tank.x)

        new_theta = standardize_angle(new_theta)
        tank_angle = standardize_angle(tank.angle)
        
        error = standardize_angle(new_theta - tank_angle)
        
        error = error - 2 * math.pi if error > math.pi else error  # convert error back to (-pi, pi]

        return error

    ########################################################################
    ########################################################################
    # def find_first_hitable_location(self, time_diff):
    #     shot_speed = float(self.constants['shotspeed'])

    #     duck_state = self.kalman_vars['mu']
    #     duck_v     = sqrt(pow(duck_state[1, 0], 2.0) + pow(duck_state[4, 0], 2.0))

    #     dx = self.hunter.x - duck_state[0, 0]
    #     dy = self.hunter.y - duck_state[3, 0]

    #     #add some reload time
    #     if self.hunter.time_to_reload > 0:
    #         dx = dx - duck_state[1, 0] * self.hunter.time_to_reload
    #         dy = dy - duck_state[4, 0] * self.hunter.time_to_reload

    #     d_to_duck = sqrt(pow(dx, 2.0) + pow(dy, 2.0))

    #     # #add some reload time
    #     # if self.hunter.time_to_reload > 0:
    #     #     d_to_duck = d_to_duck + (duck_v * self.hunter.time_to_reload)

    #     delta_t = (d_to_duck / (shot_speed - duck_v))

    #     future_duck_mu = self.predict_target_future_mu(delta_t)
    #     future_duck_pos = (future_duck_mu[0, 0], future_duck_mu[3, 0])

    #     future_duck_mu_a = self.predict_target_future_mu(delta_t + 3)
    #     future_duck_pos_a = (future_duck_mu_a[0, 0], future_duck_mu_a[3, 0])

    #     print 'Should Fire AT:', delta_t+time_diff 
    #     print "~Enemy is at: \n", self.kalman_vars['mu']
    #     print "Prediction+n: ", future_duck_pos_a 
    #     print "Prediction: ", future_duck_pos

    #     return future_duck_pos

    def find_first_hitable_location_c(self, time_diff):
        duck_state = self.kalman_vars['mu']
        duck_v     = sqrt(pow(duck_state[1, 0], 2.0) + pow(duck_state[4, 0], 2.0))

        target_startX    = duck_state[0, 0]
        target_startY    = duck_state[3, 0]

        target_velocityX = duck_state[2, 0]
        target_velocityY = duck_state[4, 0]

        #target_startX = target_startX + target_velocityX * 1
        #target_startY = target_startY + target_velocityY * 1

        if self.hunter.time_to_reload > 0:
            target_startX = target_startX + target_velocityX * self.hunter.time_to_reload
            target_startY = target_startY + target_velocityY * self.hunter.time_to_reload

        projectile_speed = float(self.constants['shotspeed'])
        tank_X           = self.hunter.x
        tank_Y           = self.hunter.y

        #a * sqr(x) + b * x + c == 0
        a = sqr(target_velocityX) + sqr(target_velocityY) - sqr(projectile_speed)
        b = 2 * (target_velocityX * (target_startX - tank_X)
              + target_velocityY * (target_startY - tank_Y))
        c = sqr(target_startX - tank_X) + sqr(target_startY - tank_Y)

        disc = sqr(b) - (4 * a * c)

        #If the discriminant is less than 0, forget about hitting your target -- 
        #your projectile can never get there in time. Otherwise, look at two 
        #candidate solutions:
        if disc < 0:
            print 'Blow chunks!'
            return None

        t1 = (-b + sqrt(disc)) / (2 * a)
        t2 = (-b - sqrt(disc)) / (2 * a)
        #Note that if disc == 0 then t1 and t2 are equal.

        #If there are no other considerations such as intervening obstacles, 
        #simply choose the smaller positive value.
        t = 0
        if t1 < 0 or t2 < 0:
            t = max(t1, t2)
        elif t1 < 0 and t2 < 0:
            print "Both times are negative!! Crap!"
        else:
            t = min(t1, t2)

        future_duck_mu = self.predict_target_future_mu(t)
        aim_2          = (future_duck_mu[0, 0], future_duck_mu[3, 0])

        aim = (t * target_velocityX + target_startX, 
               t * target_velocityY + target_startY)

        # print 'Should Fire AT:', t+time_diff#, (t,t1,t2)
        # print "~Enemy is at: \n", self.kalman_vars['mu']
        # # print "Prediction: ", aim
        # print "Prediction: ", aim_2

        return aim_2

    ########################################################################
    ########################################################################
    def get_percent_of_angular_velocity(self, error, time_diff):
        derivative = (error - self.error0) / (time_diff - self.time_set)
      
        new_angle_velocity = (self.kp * error) + (self.kd * derivative)

        return new_angle_velocity

    ########################################################################
    ########################################################################
    def fire_on_location(self, x, y, time_diff):
        angle_error = self.get_angular_error_to_location(x, y)

        new_angle_velocity = self.get_percent_of_angular_velocity(angle_error, time_diff)

        commands = []
        capture_flag_command = None

        if abs(angle_error) < .2 and self.first_hitable_location is not None and self.hunter.time_to_reload <= 0:
            # mu = self.kalman_vars['mu']

            # print "Current target (x, y, vx, vy) = (%f, %f, %f, %f)" \
            #       % (mu[0, 0], mu[3, 0], mu[1, 0], mu[4, 0])

            # print "Aimed for (x, y) = (%f, %f)" % (x, y)
            # print "Angle Error: ", angle_error

            # print "Fired At time: ", time_diff
            # print ''

            capture_flag_command = Command(self.hunter.index, 0, new_angle_velocity, True)
            self.first_hitable_location = None
        else:
            capture_flag_command = Command(self.hunter.index, 0, new_angle_velocity, False)

        commands.append(capture_flag_command)
        results = self.bzrc.do_commands(commands)

        self.error0   = angle_error
        self.time_set = time_diff
        
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