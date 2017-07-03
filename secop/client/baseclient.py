#  -*- coding: utf-8 -*-
# *****************************************************************************
#
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
#
# *****************************************************************************
"""Define Client side proxies"""

import json
import socket
import serial
from select import select
import threading
import Queue

import mlzlog

from secop.datatypes import get_datatype
from secop.lib import mkthread, formatException
from secop.lib.parsing import parse_time, format_time
from secop.protocol.encoding import ENCODERS
from secop.protocol.framing import FRAMERS
from secop.protocol.messages import *
from secop.protocol.errors import EXCEPTIONS


class TCPConnection(object):
    # disguise a TCP connection as serial one

    def __init__(self, host, port):
        self._host = host
        self._port = int(port)
        self._thread = None
        self.callbacks = []  # called if SEC-node shuts down
        self.connect()

    def connect(self):
        self._readbuffer = Queue.Queue(100)
        io = socket.create_connection((self._host, self._port))
        io.setblocking(False)
        io.settimeout(0.3)
        self._io = io
        if self._thread and self._thread.is_alive():
            return
        self._thread = mkthread(self._run)

    def _run(self):
        try:
            data = u''
            while True:
                try:
                    newdata = u''
                    dlist = [self._io.fileno()]
                    rlist, wlist, xlist = select(dlist, dlist, dlist, 1)
                    if dlist[0] in rlist + wlist:
                        newdata = self._io.recv(1024)
                    if dlist[0] in xlist:
                        print "Problem: exception on socket, reconnecting!"
                        for cb, arg in self.callbacks:
                            cb(arg)
                        return
                except socket.timeout:
                    pass
                except Exception as err:
                    print err, "reconnecting"
                    for cb, arg in self.callbacks:
                        cb(arg)
                    return
                data += newdata
                while '\n' in data:
                    line, data = data.split('\n', 1)
                    try:
                        self._readbuffer.put(line.strip('\r'),
                                             block=True,
                                             timeout=1)
                    except Queue.Full:
                        self.log.debug('rcv queue full! dropping line: %r' %
                                       line)
        finally:
            self._thread = None

    def readline(self, block=False):
        """blocks until a full line was read and returns it"""
        i = 10
        while i:
            try:
                return self._readbuffer.get(block=True, timeout=1)
            except Queue.Empty:
                continue
            if not block:
                i -= 1

    def readable(self):
        return not self._readbuffer.empty()

    def write(self, data):
        self._io.sendall(data)

    def writeline(self, line):
        self.write(line + '\n')

    def writelines(self, *lines):
        for line in lines:
            self.writeline(line)


class Value(object):
    t = None
    u = None
    e = None
    fmtstr = '%s'

    def __init__(self, value, qualifiers={}):
        self.value = value
        self.__dict__.update(qualifiers)
        if 't' in qualifiers:
            self.t = parse_time(qualifiers['t'])

    def __repr__(self):
        r = []
        if self.t is not None:
            r.append("timestamp=%r" % format_time(self.t))
        if self.u is not None:
            r.append('unit=%r' % self.u)
        if self.e is not None:
            r.append(('error=%s' % self.fmtstr) % self.e)
        if r:
            return (self.fmtstr + '(%s)') % (self.value, ', '.join(r))
        return self.fmtstr % self.value


class Client(object):
    equipment_id = 'unknown'
    secop_id = 'unknown'
    describing_data = {}
    stopflag = False

    def __init__(self, opts, autoconnect=True):
        self.log = mlzlog.log.getChild('client', True)
        self._cache = dict()
        if 'device' in opts:
            # serial port
            devport = opts.pop('device')
            baudrate = int(opts.pop('baudrate', 115200))
            self.contactPoint = "serial://%s:%s" % (devport, baudrate)
            self.connection = serial.Serial(
                devport, baudrate=baudrate, timeout=1)
            self.connection.callbacks = []
        else:
            host = opts.pop('connectto', 'localhost')
            port = int(opts.pop('port', 10767))
            self.contactPoint = "tcp://%s:%d" % (host, port)
            self.connection = TCPConnection(host, port)
        # maps an expected reply to a list containing a single Event()
        # upon rcv of that reply, entry is appended with False and
        # the data of the reply.
        # if an error is received, the entry is appended with True and an
        # appropriate Exception.
        # Then the Event is set.
        self.expected_replies = {}

        # maps spec to a set of callback functions (or single_shot callbacks)
        self.callbacks = dict()
        self.single_shots = dict()

        # mapping the modulename to a dict mapping the parameter names to their values
        # note: the module value is stored as the value of the parameter value
        # of the module

        self._syncLock = threading.RLock()
        self._thread = threading.Thread(target=self._run)
        self._thread.daemon = True
        self._thread.start()

        if autoconnect:
            self.startup()

    def _run(self):
        while not self.stopflag:
            try:
                self._inner_run()
            except Exception as err:
                self.log.exception(err)
                raise

    def _inner_run(self):
        data = ''
        self.connection.writeline('*IDN?')

        while not self.stopflag:
            line = self.connection.readline()
            self.log.debug('got answer %r' % line)
            if line.startswith(('SECoP', 'SINE2020&ISSE,SECoP')):
                self.log.info('connected to: ' + line.strip())
                self.secop_id = line
                continue
            msgtype, spec, data = self.decode_message(line)
            if msgtype in ('event', 'update', 'changed'):
                # handle async stuff
                self._handle_event(spec, data)
            # handle sync stuff
            self._handle_sync_reply(msgtype, spec, data)

    def _handle_sync_reply(self, msgtype, spec, data):
        # handle sync stuff
        if msgtype == "error":
            # find originating msgtype and map to expected_reply_type
            # errormessages carry to offending request as the first
            # result in the resultist
            _msgtype, _spec, _data = self.decode_message(data[0])
            _reply = self._get_reply_from_request(_msgtype)

            entry = self.expected_replies.get((_reply, _spec), None)
            if entry:
                self.log.error("request %r resulted in Error %r" %
                               (data[0], spec))
                entry.extend([True, EXCEPTIONS[spec](*data)])
                entry[0].set()
                return
            self.log.error("got an unexpected error %s %r" % (spec, data[0]))
            return
        if msgtype == "describing":
            entry = self.expected_replies.get((msgtype, ''), None)
        else:
            entry = self.expected_replies.get((msgtype, spec), None)

        if entry:
            self.log.debug("got expected reply '%s %s'" % (msgtype, spec)
                           if spec else "got expected reply '%s'" % msgtype)
            entry.extend([False, msgtype, spec, data])
            entry[0].set()
            return

    def encode_message(self, requesttype, spec='', data=None):
        """encodes the given message to a string
        """
        req = [str(requesttype)]
        if spec:
            req.append(str(spec))
        if data is not None:
            req.append(json.dumps(data))
        req = ' '.join(req)
        return req

    def decode_message(self, msg):
        """return a decoded message tripel"""
        msg = msg.strip()
        if ' ' not in msg:
            return msg, '', None
        msgtype, spec = msg.split(' ', 1)
        data = None
        if ' ' in spec:
            spec, json_data = spec.split(' ', 1)
            try:
                data = json.loads(json_data)
            except ValueError:
                # keep as string
                data = json_data
                # print formatException()
        return msgtype, spec, data

    def _handle_event(self, spec, data):
        """handles event"""
        self.log.debug('handle_event %r %r' % (spec, data))
        if ':' not in spec:
            self.log.warning("deprecated specifier %r" % spec)
            spec = '%s:value' % spec
        modname, pname = spec.split(':', 1)
        self._cache.setdefault(modname, {})[pname] = Value(*data)
        if spec in self.callbacks:
            for func in self.callbacks[spec]:
                try:
                    mkthread(func, modname, pname, data)
                except Exception as err:
                    self.log.exception('Exception in Callback!', err)
        run = set()
        if spec in self.single_shots:
            for func in self.single_shots[spec]:
                try:
                    mkthread(func, data)
                except Exception as err:
                    self.log.exception('Exception in Single-shot Callback!',
                                       err)
                run.add(func)
            self.single_shots[spec].difference_update(run)

    def _getDescribingModuleData(self, module):
        return self.describingModulesData[module]

    def _getDescribingParameterData(self, module, parameter):
        return self._getDescribingModuleData(module)['parameters'][parameter]

    def _issueDescribe(self):
        _, self.equipment_id, self.describing_data = self._communicate(
            'describe')

        for module, moduleData in self.describing_data['modules'].items():
            for parameter, parameterData in moduleData['parameters'].items():
                datatype = get_datatype(parameterData['datatype'])
                self.describing_data['modules'][module]['parameters'] \
                    [parameter]['datatype'] = datatype

    def register_callback(self, module, parameter, cb):
        self.log.debug('registering callback %r for %s:%s' %
                       (cb, module, parameter))
        self.callbacks.setdefault('%s:%s' % (module, parameter), set()).add(cb)

    def unregister_callback(self, module, parameter, cb):
        self.log.debug('unregistering callback %r for %s:%s' %
                       (cb, module, parameter))
        self.callbacks.setdefault('%s:%s' % (module, parameter),
                                  set()).discard(cb)

    def register_shutdown_callback(self, func, arg):
        self.connection.callbacks.append((func, arg))

    def _get_reply_from_request(self, requesttype):
        # maps each (sync) request to the corresponding reply
        # XXX: should go to the encoder! and be imported here
        REPLYMAP = {
            "describe": "describing",
            "do": "done",
            "change": "changed",
            "activate": "active",
            "deactivate": "inactive",
            "read": "update",
            #"*IDN?": "SECoP,",  # XXX: !!!
            "ping": "pong",
        }
        return REPLYMAP.get(requesttype, requesttype)

    def communicate(self, msgtype, spec='', data=None):
        # only return the data portion....
        return self._communicate(msgtype, spec, data)[2]

    def _communicate(self, msgtype, spec='', data=None):
        self.log.debug('communicate: %r %r %r' % (msgtype, spec, data))
        if self.stopflag:
            raise RuntimeError('alreading stopping!')
        if msgtype == "*IDN?":
            return self.secop_id

        if msgtype not in ('*IDN?', 'describe', 'activate', 'deactivate', 'do',
                           'change', 'read', 'ping', 'help'):
            raise EXCEPTIONS['Protocol'](args=[
                self.encode_message(msgtype, spec, data),
                dict(
                    errorclass='Protocol',
                    errorinfo='%r: No Such Messagetype defined!' % msgtype, ),
            ])

        # sanitize input + handle syntactic sugar
        msgtype = str(msgtype)
        spec = str(spec)
        if msgtype == 'change' and ':' not in spec:
            spec = spec + ':target'
        if msgtype == 'read' and ':' not in spec:
            spec = spec + ':value'

        # check if a such a request is already out
        rply = self._get_reply_from_request(msgtype)
        if (rply, spec) in self.expected_replies:
            raise RuntimeError(
                "can not have more than one requests of the same type at the same time!"
            )

        # prepare sending request
        event = threading.Event()
        self.expected_replies[(rply, spec)] = [event]
        self.log.debug('prepared reception of %r msg' % rply)

        # send request
        msg = self.encode_message(msgtype, spec, data)
        self.connection.writeline(msg)
        self.log.debug('sent msg %r' % msg)

        # wait for reply. timeout after 10s
        if event.wait(10):
            self.log.debug('checking reply')
            entry = self.expected_replies.pop((rply, spec))
            # entry is: event, is_error, exc_or_msgtype [,spec, date]<- if !err
            is_error = entry[1]
            if is_error:
                # if error, entry[2] contains the rigth Exception to raise
                raise entry[2]
            # valid reply: entry[2:5] contain msgtype, spec, data
            return tuple(entry[2:5])

        # timed out
        del self.expected_replies[(rply, spec)]
        # XXX: raise a TimedOut ?
        raise RuntimeError("timeout upon waiting for reply to %r!" % msgtype)

    def quit(self):
        # after calling this the client is dysfunctional!
        self.communicate('deactivate')
        self.stopflag = True
        if self._thread and self._thread.is_alive():
            self.thread.join(self._thread)

    def handle_async(self, msg):
        self.log.info("Got async update %r" % msg)
        device = msg.device
        param = msg.param
        value = msg.value
        self._cache.getdefault(device, {})[param] = value
        # XXX: further notification-callbacks needed ???

    def startup(self, async=False):
        self._issueDescribe()
        # always fill our cache
        self.communicate('activate')
        # deactivate updates if not wanted
        if not async:
            self.communicate('deactivate')

    def queryCache(self, module, parameter=None):
        result = self._cache.get(module, {})

        if parameter is not None:
            result = result[parameter]

        return result

    def getParameter(self, module, parameter):
        return self.communicate('read', '%s:%s' % (module, parameter))

    def setParameter(self, module, parameter, value):
        datatype = self._getDescribingParameterData(module,
                                                    parameter)['datatype']

        value = datatype.export(datatype.validate(value))
        self.communicate('change', '%s:%s' % (module, parameter), value)

    @property
    def describingData(self):
        return self.describing_data

    @property
    def describingModulesData(self):
        return self.describingData['modules']

    @property
    def equipmentId(self):
        return self.equipment_id

    @property
    def protocolVersion(self):
        return self.secop_id

    @property
    def modules(self):
        return self.describing_data['modules'].keys()

    def getParameters(self, module):
        return self.describing_data['modules'][module]['parameters'].keys()

    def getModuleProperties(self, module):
        return self.describing_data['modules'][module]['properties']

    def getModuleBaseClass(self, module):
        return self.getModuleProperties(module)['interface']

    def getCommands(self, module):
        return self.describing_data['modules'][module]['commands'].keys()

    def getProperties(self, module, parameter):
        return self.describing_data['modules'][module]['parameters'][parameter]

    def syncCommunicate(self, *msg):
        res = self._communicate(*msg)
        try:
            res = self.encode_message(*res)
        except Exception:
            res = str(res)
        return res

    def ping(self, pingctr=[0]):
        pingctr[0] = pingctr[0] + 1
        self.communicate("ping", pingctr[0])
