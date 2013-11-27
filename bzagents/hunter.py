__author__ = 'crunk'

import sys
from bzrc import *
from numpy import matrix

###########################Kalman Functions#############################


########################################################################
# 
########################################################################
def mu_plus_one():

    return


########################################################################
# 
########################################################################
def sigma_plus_one():

    return


########################################################################
# 
########################################################################
def k_plus_one():

    return

#-----------------------------END-Kalman Functions---------------
#----------------------------------------------------------------

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

sigma_x = matrix('0.1 0   0    0   0   0  ;\
                  0   0.1 0    0   0   0  ;\
                  0   0   100  0   0   0  ;\
                  0   0   0    0.1 0   0  ;\
                  0   0   0    0   0.1 0  ;\
                  0   0   0    0   0   100')

#-----------------------------END--------------------------------
#----------------------------------------------------------------


class Agent(object):
    def __init__(self, bzrc):
        global mu_not, sigma_not
        self.bzrc = bzrc
        self.constants = self.bzrc.get_constants()
        self.commands = []
        self.tanks = []
        self.hunter = None
        self.enemies = []
        self.target = None
        self.kalman_vars = {'mu': mu_not, 'sigma': sigma_not}

    def tick(self, time_diff):
        my_tanks, other_tanks, flags, shots = self.bzrc.get_lots_o_stuff()
        self.tanks = my_tanks
        if self.hunter is None:
            self.hunter = self.tanks[0]
        self.enemies = [tank for tank in other_tanks if tank.color !=
                        self.constants['team']]
        if len(self.enemies) > 0:
            self.target = self.enemies[0]
        else:
            self.target = None

        self.kalman_update()

    def kalman_update(self):
        global H, I, sigma_x, sigma_z
        tmp = F()*self.kalman_vars['sigma']*F().T + sigma_x
        k = tmp*H.T*(H*tmp*H.T + sigma_z).I
        z = matrix([[self.target.x], [self.target.y]])
        new_mu = F()*self.kalman_vars['mu'] + k*(z - H*F()*self.kalman_vars['mu'])
        new_sigma = (I - k*H)*tmp
        self.kalman_vars['mu'] = new_mu
        self.kalman_vars['sigma'] = new_sigma


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