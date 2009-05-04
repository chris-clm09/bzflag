"""BSFlag Main

The BSFlag Main module contains the program's entry point and event loop.
"""

import asyncore
import os
import socket
import sys

from game import Game
import server

# A higher loop timeout decreases CPU usage but also decreases the frame rate.
LOOP_TIMEOUT = 0.01


def options():
    import optparse
    p = optparse.OptionParser()
    p.add_option('--world', action='store', dest='world')
    p.add_option('--port', action='store', type='int', dest='port', default=0)
    opts, args = p.parse_args()
    if args:
        p.parse_error('No positional arguments are allowed.')
    return opts


def run():
    opts = options()

    from world import World
    if opts.world:
        f = open(opts.world)
        parser = World.parser()
        results = parser.parseString(f.read())
        world = results[0]
    else:
        world = World()

    colors = (1, 2)
    game = Game(colors)

    # Create a server for each team.
    # TODO: allow the port to be specified on the command-line.
    for team in game.teams:
        addr = ('0.0.0.0', opts.port)
        try:
            bzrc = server.Server(addr, team)
        except socket.error, e:
            print >>sys.stderr, 'Socket error:', os.strerror(e.errno)
            sys.exit(1)
        host, port = bzrc.socket.getsockname()
        print 'Listening on port %s for %s team.' % (port, team.color_name())


    import graphics
    display = graphics.Display(world)
    display.setup()

    for team in game.teams:
        for shot in team.shots:
            display.shot_sprite(shot)
        for tank in team.tanks:
            display.tank_sprite(tank)

    while True:
        asyncore.loop(LOOP_TIMEOUT, count=1)

        game.update()
        display.update()


# vim: et sw=4 sts=4
