__author__ = 'scott'

import math


def standardize_angle(rad):
    """
    Takes an angle in radians, rad, and returns its equivalent in the range [0, 2*pi)
    """
    result = rad % (2*math.pi)
    if result < 0:  # I think this case gets taken care of in the modulo operator, but just in case...
        return result + 2*math.pi
    return result