__author__ = 'scott'

import math


class Point(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __str__(self):
        return "(" + str(self.x) + ", " + str(self.y) + ")"


class Circle(object):
    def __init__(self, x, y, radius):
        self.center = Point(x, y)
        self.radius = radius

    def __str__(self):
        return "Obstacle at " + str(self.center) + " with radius = " + str(self.radius)


####################################################################
# Distance between two points.
####################################################################
def distance(x, y, goal):
    return distance_coords(x, y, goal.x, goal.y)


def distance_coords(x1, y1, x2, y2):
    return math.sqrt(((y2 - y1)*(y2 - y1)) + ((x2 - x1)*(x2 - x1)))


def distance_points(p1, p2):
    return distance_coords(p1.x, p1.y, p2.x, p2.y)


def sign(a):
    if a == 0 or a == -0:
        return 0
    return a / -a