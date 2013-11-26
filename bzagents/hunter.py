__author__ = 'crunk'

import sys
from bzrc import *
from numpy import *

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
mu_not    = matrix('000000')

sigma_not = matrix('100  0   0    0   0   0  ;\
                      0  0.1 0    0   0   0  ;\
                      0  0   0.1  0   0   0  ;\
                      0  0   0  100   0   0  ;\
                      0  0   0    0   0.1 0  ;\
                      0  0   0    0   0   0.1')

H = matrix('1 0 0 0 0 0;\
            0 0 0 1 0 0')

sigma_z = matrix('25  0;\
                   0 25')

def F(delta_t=0.5):
    c = -0.1
    # c = 0.0

    m =     '1 ' + delta_t + ' ' + (pow(delta_t,2)/2.0) + ' 0 0 0;'
    m = m + '0 1 ' + delta_t + ' 0 0 0;'
    m = m + '0 ' + c + ' 1 0 0 0;'
    m = m + '0 0 0 1 ' + delta_t + ' ' + (pow(delta_t,2)/2.0) + ';'
    m = m + '0 0 0 0 1 ' + delta_t + ';'
    m = m + '0 0 0 0 ' + c + ' 1'   
    return matrix(m)

sigma_x =   matrix('  0.1 0   0    0   0   0  ;\
                      0   0.1 0    0   0   0  ;\
                      0   0   100  0   0   0  ;\
                      0   0   0    0.1 0   0  ;\
                      0   0   0    0   0.1 0  ;\
                      0   0   0    0   0   100')



#-----------------------------END--------------------------------
#----------------------------------------------------------------


class Agent(object):
    def __init__(self, bzrc):
        self.bzrc = bzrc
        self.constants = self.bzrc.get_constants()
        self.commands = []
        self.tanks = []
        self.ducks = []
        self.enemies = []
        self.target = None

    def tick(self, time_diff):
        my_tanks, other_tanks, flags, shots = self.bzrc.get_lots_o_stuff()
        self.tanks = my_tanks
        self.ducks = other_tanks
        self.enemies = [tank for tank in other_tanks if tank.color !=
                        self.constants['team']]
        self.target = self.enemies[0]


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