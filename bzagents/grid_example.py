#!/usr/bin/env python

import OpenGL
import numpy
OpenGL.ERROR_CHECKING = False
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
from numpy import zeros
import time

grid = None

def reverse_grid_values():
    global grid

    for r in range(0, len(grid)):
        for c in range(0, len(grid[r])):
            grid[r][c] = 1.0 - grid[r][c]

def update_grid(new_grid):
    global grid
    grid = numpy.array(new_grid)
    reverse_grid_values()

def draw_grid():
    # This assumes you are using a numpy array for your grid
    width, height = grid.shape
    glRasterPos2f(-1.0, -1.0)
    glDrawPixels(width, height, GL_LUMINANCE, GL_FLOAT, grid)
    glFlush()
    glutSwapBuffers()

def init_window(width, height):
    global window
    global grid
    grid = zeros((width, height))
    glutInit(())
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH)
    glutInitWindowSize(width, height)
    glutInitWindowPosition(0, 0)
    window = glutCreateWindow("Grid filter")
    glutDisplayFunc(draw_grid)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()


#========================================================================
# Comand Line Tests and Run
#========================================================================
def reset_grid_with_value(value, square_size):
    initialProb = value
    probability_map = []

    for r in range(0, square_size):
        probability_map.append([])
        for c in range(0, square_size):
            probability_map[r].append(initialProb)

    update_grid(probability_map)

if __name__ == '__main__':
    init_window(800,800)
    draw_grid()

    for i in range(0,10):
        reset_grid_with_value(i/10.0, 800)
        draw_grid()
        time.sleep(.3)



# vim: et sw=4 sts=4