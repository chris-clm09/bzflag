__author__ = 'crunk'

import sys
from bzrc import *
from numpy.matrixlib import matrix, matrix_power

###########################Constants And Stuff#############################
mu_not = matrix('000000')

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


class Agent(object):
    def __init__(self, bzrc):
        self.bzrc = bzrc
        self.constants = self.bzrc.get_constants()
        self.commands = []
        self.tanks = []
        self.hunter = None
        self.enemies = []
        self.target = None
        self.kalman_vars = {}
        self.init_kalman()
        self.ave_time_diff = 0
        self.ave_time_diff_samples = 0

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
        else:  # we must have killed the target tank
            self.target = None
            self.init_kalman()

        self.kalman_update(time_diff)
        self.plot_kalman()

    def init_kalman(self):
        global mu_not, sigma_not
        self.kalman_vars['mu'] = mu_not
        self.kalman_vars['sigma'] = sigma_not

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

    def predict_target_future_mu(self, time_steps):
        _F = F(self.ave_time_diff)
        return matrix_power(_F, time_steps)*self.kalman_vars['mu']

    def plot_kalman(self):
        pass


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