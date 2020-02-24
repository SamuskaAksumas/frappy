#  -*- coding: utf-8 -*-
# *****************************************************************************
# This program is free software; you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free Software
# Foundation; either version 2 of the License, or (at your option) any later
# version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
# details.
#
# You should have received a copy of the GNU General Public License along with
# this program; if not, write to the Free Software Foundation, Inc.,
# 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
#
# Module authors:
#   Enrico Faulhaber <enrico.faulhaber@frm2.tum.de>
#   Markus Zolliker <markus.zolliker@psi.ch>
#
# *****************************************************************************
"""provides tcp interface to the SECoP Server"""

import sys
import socket
import collections
import socketserver

from secop.datatypes import StringType, IntRange, BoolType
from secop.errors import SECoPError
from secop.lib import formatException, \
    formatExtendedStack, formatExtendedTraceback
from secop.properties import HasProperties, Property
from secop.protocol.interface import decode_msg, encode_msg_frame, get_msg
from secop.protocol.messages import ERRORPREFIX, \
    HELPREPLY, HELPREQUEST, HelpMessage


DEF_PORT = 10767
MESSAGE_READ_SIZE = 1024
HELP = HELPREQUEST.encode()

class OutputBufferOverflow(Exception):
    pass


class TCPRequestHandler(socketserver.BaseRequestHandler):

    def setup(self):
        self.log = self.server.log
        # Queue of msgObjects to send
        self._queue = collections.deque()  # do not use maxlen, as items might get lost
#        self.framing = self.server.framingCLS()
#        self.encoding = self.server.encodingCLS()

    def handle(self):
        """handle a new tcp-connection"""
        # copy state info
        mysocket = self.request
        clientaddr = self.client_address
        serverobj = self.server
        self.log.info("handling new connection from %s:%d" % clientaddr)
        data = b''

        # notify dispatcher of us
        serverobj.dispatcher.add_connection(self)

        # copy relevant settings from Interface
        detailed_errors = serverobj.detailed_errors

        mysocket.settimeout(.3)
        #        mysocket.setblocking(False)
        # start serving
        while True:
            # send replys first, then listen for requests, timing out after 0.1s
            while self._queue:
                # put message into encoder to get frame(s)
                # put frame(s) into framer to get bytestring
                # send bytestring
                outmsg = self._queue.popleft()
                if not outmsg:
                    outmsg = ('error', 'InternalError', ['<unknown origin>', 'trying to send none-data', {}])
                if len(outmsg) > 3:
                    outmsg = ('error', 'InternalError', ['<unknown origin>', 'bad message format', {'msg': outmsg}])
                outdata = encode_msg_frame(*outmsg)
                try:
                    mysocket.sendall(outdata)
                except Exception as e:
                    self.log.error('error on sendall: %r', e)
                    return

            # XXX: improve: use polling/select here?
            try:
                newdata = mysocket.recv(MESSAGE_READ_SIZE)
                if not newdata:
                    # no timeout error, but no new data -> connection closed
                    return
                data = data + newdata
            except socket.timeout:
                continue
            except socket.error as e:
                self.log.exception(e)
                return
            # XXX: should use select instead of busy polling
            if not data:
                continue
            # put data into (de-) framer,
            # put frames into (de-) coder and if a message appear,
            # call dispatcher.handle_request(self, message)
            # dispatcher will queue the reply before returning
            while True:
                origin, data = get_msg(data)
                if origin is None:
                    break  # no more messages to process
                origin = origin.strip()
                if origin in (HELP, b''):  # empty string -> send help message
                    for idx, line in enumerate(HelpMessage.splitlines()):
                        # not sending HELPREPLY here, as there should be only one reply for every request
                        self.queue_async_reply(('_', '%d' % (idx+1), line))
                    # ident matches request
                    self.queue_async_reply((HELPREPLY, None, None))
                    continue
                try:
                    msg = decode_msg(origin)
                except Exception as err:
                    # we have to decode 'origin' here
                    # use latin-1, as utf-8 or ascii may lead to encoding errors
                    msg = origin.decode('latin-1').split(' ', 3) + [None]  # make sure len(msg) > 1
                    result = (ERRORPREFIX + msg[0], msg[1], ['InternalError', str(err),
                                                             {'exception': formatException(),
                                                              'traceback': formatExtendedStack()}])
                    print('--------------------')
                    print(formatException())
                    print('--------------------')
                    print(formatExtendedTraceback(sys.exc_info()))
                    print('====================')
                else:
                    try:
                        result = serverobj.dispatcher.handle_request(self, msg)
                    except OutputBufferOverflow:
                        raise
                    except SECoPError as err:
                        result = (ERRORPREFIX + msg[0], msg[1], [err.name, str(err),
                                                                 {'exception': formatException(),
                                                                  'traceback': formatExtendedStack()}])
                    except Exception as err:
                        # create Error Obj instead
                        result = (ERRORPREFIX + msg[0], msg[1], ['InternalError', str(err),
                                                                 {'exception': formatException(),
                                                                  'traceback': formatExtendedStack()}])
                        print('--------------------')
                        print(formatException())
                        print('--------------------')
                        print(formatExtendedTraceback(sys.exc_info()))
                        print('====================')

                if not result:
                    self.log.error('empty result upon msg %s' % repr(msg))
                if result[0].startswith(ERRORPREFIX) and not detailed_errors:
                    # strip extra information
                    result[2][2].clear()
                self.queue_async_reply(result)

    def queue_async_reply(self, data):
        """called by dispatcher for async data units"""
        if data:
            # avoid queue growing to infinity. the needed size of the queue might be
            # a multiple of the total number of parameters -> use a big number
            if len(self._queue) > 10000:
                self.log.error('output message buffer overflow')
                raise OutputBufferOverflow()
            self._queue.append(data)
        else:
            self.log.error('should async_queue empty data!')

    def finish(self):
        """called when handle() terminates, i.e. the socket closed"""
        self.log.info('closing connection from %s:%d' % self.client_address)
        # notify dispatcher
        self.server.dispatcher.remove_connection(self)
        # close socket
        try:
            self.request.shutdown(socket.SHUT_RDWR)
        except Exception:
            pass
        finally:
            self.request.close()


class TCPServer(HasProperties, socketserver.ThreadingTCPServer):
    daemon_threads = True
    allow_reuse_address = True

    properties = {
        'bindto': Property('hostname or ip address for binding', StringType(),
                           default='localhost:%d' % DEF_PORT, export=False),
        'bindport': Property('port number to bind', IntRange(1, 65535),
                             default=DEF_PORT, export=False),
        'detailed_errors': Property('Flag to enable detailed Errorreporting.', BoolType(),
                                    default=False, export=False),
    }

    # XXX: create configurables from Metaclass!
    configurables = properties

    def __init__(self, name, logger, options, srv):  # pylint: disable=super-init-not-called
        self.dispatcher = srv.dispatcher
        self.name = name
        self.log = logger
        # do not call HasProperties.__init__, as this will supercall ThreadingTCPServer
        self.initProperties()
        bindto = options.pop('bindto', 'localhost')
        bindport = int(options.pop('bindport', DEF_PORT))
        detailed_errors = options.pop('detailed_errors', False)
        if ':' in bindto:
            bindto, _port = bindto.rsplit(':')
            bindport = int(_port)

        self.setProperty('bindto', bindto)
        self.setProperty('bindport', bindport)
        self.setProperty('detailed_errors', detailed_errors)
        self.checkProperties()

        self.allow_reuse_address = True
        self.log.info("TCPServer %s binding to %s:%d" % (name, self.bindto, self.bindport))
        socketserver.ThreadingTCPServer.__init__(
            self, (self.bindto, self.bindport), TCPRequestHandler, bind_and_activate=True)
        self.log.info("TCPServer initiated")
