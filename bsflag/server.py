"""BSFlag BZRC Server"""

import asynchat
import asyncore
import socket
import time

BACKLOG = 5


class Server(asyncore.dispatcher):
    def __init__(self, addr, team):
        self.team = team
        self.in_use = False
        sock = socket.socket()
        asyncore.dispatcher.__init__(self, sock)
        self.bind(addr)
        self.listen(BACKLOG)

    def handle_accept(self):
        sock, addr = self.accept()
        if self.in_use:
            sock.close()
        else:
            self.in_use = True
            Handler(sock, self.team, self.handle_closed_handler)

    def handle_closed_handler(self):
        self.in_use = False


class Handler(asynchat.async_chat):
    """Server which implements the BZRC protocol.

    Each team has its own server.
    """
    def __init__(self, sock, team, closed_callback):
        asynchat.async_chat.__init__(self, sock)
        self.team = team
        self.closed_callback = closed_callback
        self.set_terminator('\n')
        self.input_buffer = ''
        self.push('bzrobots 1\n')
        self.init_timestamp = time.time()
        self.established = False

    def handle_close(self):
        self.close()

    def collect_incoming_data(self, chunk):
        if self.input_buffer:
            self.input_buffer += chunk
        else:
            self.input_buffer = chunk

    def found_terminator(self):
        """Called when Asynchat finds an end-of-line.

        Note that Asynchat ensures that our input buffer contains everything
        up to but not including the newline character.
        """
        args = self.input_buffer.split()
        self.input_buffer = ''
        if args:
            if self.established:
                try:
                    command = getattr(self, 'bzrc_%s' % args[0])
                except AttributeError:
                    self.push('fail Invalid command\n')
                    return
                command(args)
            elif args == ['bzagent', '1']:
                self.established = True
            else:
                self.bad_handshake()

    def bad_handshake(self):
        """Called when the client gives an invalid handshake message."""
        self.push('fail Unrecognized handshake\n')
        self.close()

    def close(self):
        self.closed_callback()
        asynchat.async_chat.close(self)

    def invalid_args(self, args):
        self.ack(*args)
        self.push('fail Invalid parameter(s)\n')

    def ack(self, *args):
        timestamp = time.time() - self.init_timestamp
        arg_string = ' '.join(str(arg) for arg in args)
        self.push('ack %s %s\n' % (timestamp, arg_string))

    def bzrc_shoot(self, args):
        """Requests the tank to shoot."""
        try:
            command, tankid = args
            tankid = int(tankid)
        except ValueError, TypeError:
            self.invalid_args(args)
            return
        self.ack(command, tankid)
        self.team.shoot(tankid)

    def bzrc_angvel(self, args):
        """Sets the angular velocity of the tank."""
        try:
            command, tankid, value = args
            tankid = int(tankid)
            value = float(value)
        except ValueError, TypeError:
            self.invalid_args(args)
            return
        self.ack(command, tankid, value)
        self.team.angvel(tankid, value)

    def bzrc_shots(self, args):
        """Reports a list of shots.

        The response is a list of shot lines:
            shot [x] [y] [vx] [vy]
        """
        try:
            command, = args
        except ValueError, TypeError:
            self.invalid_args(args)
            return
        self.ack(command)
        self.push('begin\n')
        for team in self.team.game.teams:
            for shot in team.shots:
                x, y = shot.pos
                vx, vy = shot.vel
                self.push('shot %s %s %s %s\n' % (x, y, vx, vy))
        self.push('end\n')

    def bzrc_quit(self, args):
        """Disconnects the session.

        This is technically an extension to the BZRC protocol.  We should
        really backport this to BZFlag.
        """
        try:
            command, = args
        except ValueError, TypeError:
            self.invalid_args(args)
            return
        self.close()


# vim: et sw=4 sts=4