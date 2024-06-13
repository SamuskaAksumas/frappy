# *****************************************************************************
# PILS PLC client library
# Copyright (c) 2019-2021 by the authors, see LICENSE
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
#   Georg Brandl <g.brandl@fz-juelich.de>
#
# *****************************************************************************

"""Implements communication via ADS, the Beckhoff universal protocol."""

import errno
import re
import socket
import time
from struct import Struct, pack

from zapf import ApiError, CommError
from zapf.proto import Protocol

# Allow omitting the first 4 NetID bytes if it's the same as the IP address,
# but always require the port.
ADS_ADDR_RE = re.compile(r'ads://(.*?)/(\d+\.\d+\.\d+\.\d+(\.\d+\.\d+)?)?:(\d+)$')

# TCP port to use for ADS.
ADS_PORT = 0xBF02

# AMS header including AMS/TCP header.
HEADER = Struct('<HIQQHHIII')
# Reply to device info command.
DEVINFO = Struct('<BBH16s')

HEADER_SIZE = HEADER.size
AMS_HEADER_SIZE = HEADER_SIZE - 6

# ADS commands.
ADS_DEVINFO = 1
ADS_READ    = 2
ADS_WRITE   = 3
ADS_STATUS  = 4

# Formats for payload.
ADS_ADR_LEN = Struct('<III')
ADS_ERRID = Struct('<I')

# Index group for %M memory area.
INDEXGROUP_M = 0x4020

# UDP port
BECKHOFF_UDP_PORT = 0xBF03
# UDP message to set a route.
UDP_MESSAGE = Struct(
    '<I'     # magic
    '4x'     # pad
    'I'      # operation
    'Q'      # source netaddr
    'I'      # nitems=5
    'HH25s'  # desig=12 (routename), strlen=25, content
    'HH6s'   # desig=7 (netid), len=6, content
    'HH14s'  # desig=13 (username), len=14, content
    'HH2s'   # desig=2 (password), len=2, content
    'HH16s'  # desig=5 (host), len=16, content
)
# UDP magic header number.
UDP_MAGIC = 0x71146603
# UDP packet operations and data designators.
UDP_ADD_ROUTE = 6
UDP_PASSWORD = 2
UDP_HOST = 5
UDP_NETID = 7
UDP_ROUTENAME = 12
UDP_USERNAME = 13


def pack_net_id(netidstr, amsport):
    # convert a.b.c.d.e.f to a single integer
    amsnetid = i = 0
    for (i, x) in enumerate(netidstr.split('.')):
        amsnetid |= (int(x) << (8 * i))
    if i != 5:
        raise ApiError('incomplete NetID; use format a.b.c.d.e.f '
                       'with 6 integers in the range 0-255')

    # pack the whole address into a 64-bit integer
    return amsnetid | (amsport << 48)


class ADSProtocol(Protocol):
    OFFSETS = [0]

    def __init__(self, url, log):
        adr = ADS_ADDR_RE.match(url)
        if not adr:
            raise ApiError('invalid ADS address, must be '
                           'ads://host[:port]/amsnetid:amsport')
        host = adr.group(1)
        if ':' in host:
            host, port = host.split(':')
            port = int(port)
        else:
            port = ADS_PORT
        host_ip = socket.gethostbyname(host)

        if adr.group(3):    # full AMS netid specified
            netidstr = adr.group(2)
        elif adr.group(2):  # only first 4 specified
            netidstr = adr.group(2) + '.1.1'
        else:               # nothing specified, use IP
            netidstr = host_ip + '.1.1'
        amsport = int(adr.group(4))

        self._amsnetaddr = pack_net_id(netidstr, amsport)
        self._iphostport = (host, port)
        self._invoke_id = 1  # should be incremented for every request
        self._socket = None
        self._tried_route = False

        super().__init__(url, log)

    def _retry_connect_with_route(self):
        self.log.warning('connection aborted, trying to set a route...')
        self._tried_route = True
        self._set_route(self._socket.getsockname())
        self.connect()

    def connect(self):
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self._socket.connect(self._iphostport)
            self._socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self._myamsnetaddr = pack_net_id(
                self._socket.getsockname()[0] + '.1.1', 55189)  # guaranteed random
            reply = self._comm(ADS_DEVINFO, b'', DEVINFO.size)
            v1, v2, v3, name = DEVINFO.unpack(reply)
            name = name.partition(b'\0')[0].decode('latin1')
            hw_name = f'{name} {v1}.{v2}.{v3}'
            self.log.info('connected to %s', hw_name)
        except (OSError, ValueError) as err:
            if getattr(err, 'errno', None) == errno.ECONNRESET and not \
               self._tried_route and self._iphostport[1] == ADS_PORT:
                self._retry_connect_with_route()
                return
            raise CommError(f'could not connect to {self.url}: {err}') from err
        except CommError as err:
            # if we get a closed connection immediately, the route is not set
            # up correctly.  Try to fix that by setting a route via UDP.  If
            # the target TCP port is not default, we are not talking directly
            # to TwinCAT, so don't even try in that case.
            if str(err) == 'no data in read' and not self._tried_route and \
               self._iphostport[1] == ADS_PORT:
                self._retry_connect_with_route()
                return
            raise
        self._connect_callback(True)
        self.connected = True

    def disconnect(self):
        try:
            self._socket.shutdown(socket.SHUT_RDWR)
            self._socket.close()
        except (AttributeError, OSError):
            pass
        self._socket = None
        self._connect_callback(False)
        self.connected = False

    def read(self, addr, length):
        if not self.connected:
            self.reconnect()
        payload = ADS_ADR_LEN.pack(INDEXGROUP_M, addr, length)
        try:
            return self._comm(ADS_READ, payload, 4 + length)[4:]
        except OSError as err:
            self.log.exception('during read')
            self.disconnect()
            raise CommError(f'IO error during read: {err}') from err

    def write(self, addr, data):
        if not self.connected:
            self.reconnect()
        payload = ADS_ADR_LEN.pack(INDEXGROUP_M, addr, len(data)) + data
        try:
            self._comm(ADS_WRITE, payload, 0)
        except OSError as err:
            self.log.exception('during write')
            self.disconnect()
            raise CommError(f'IO error during write: {err}') from err

    def _comm(self, cmd, payload, exp_len):
        """One ADS request-reply cycle."""
        invoke_id, self._invoke_id = self._invoke_id, self._invoke_id + 1
        msg = HEADER.pack(0, AMS_HEADER_SIZE + len(payload),
                          self._amsnetaddr, self._myamsnetaddr, cmd, 0x4,
                          len(payload), 0, invoke_id)
        self._socket.sendall(msg + payload)
        return self._comm_reply(cmd, invoke_id, exp_len)

    def _comm_reply(self, cmd, invoke_id, exp_len):
        # read the reply header
        rephdr = HEADER.unpack(self._read_exact(HEADER_SIZE))

        # check consistency
        if rephdr[0] != 0:
            # inconsistent packet, close connection
            self.disconnect()
            raise CommError('leading null bytes missing, out of sync?')
        data_len = rephdr[1] - AMS_HEADER_SIZE  # payload incl. result field
        if rephdr[6] != data_len:
            # inconsistent packet, close connection
            self.disconnect()
            raise CommError('wrong length in reply header')

        # always read payload from socket, even if packet will be ignored
        data = self._read_exact(data_len)

        # check if packet is not addressed to us
        if rephdr[2] != self._myamsnetaddr:
            targetport = rephdr[2] >> 48
            # don't warn for Status queries for the system manager
            if not (targetport == 10000 and rephdr[4] == ADS_STATUS):
                self.log.warning(f'got packet for AMS port {targetport}, ignoring')
            # try again with next packet
            return self._comm_reply(cmd, invoke_id, exp_len)

        # check if packet has expected contents
        if rephdr[4] != cmd:
            raise CommError('wrong command in reply header')
        if rephdr[5] != 0x5:
            raise CommError('wrong flags in reply header')
        if rephdr[8] != invoke_id:
            raise CommError('wrong InvokeID on reply packet')
        if rephdr[7]:
            raise CommError('error set in reply header: '
                            f'{self._translate_error(rephdr[7])}')
        if data[:4] != b'\x00\x00\x00\x00':
            result = ADS_ERRID.unpack_from(data)[0]
            raise CommError('error set in reply data: '
                            f'{self._translate_error(result)}')
        if data_len != exp_len + 4:
            raise CommError('received unexpected length in reply '
                            f'(expected {exp_len}, got {data_len})')
        return data[4:]

    def _read_exact(self, length):
        """Blockingly read exactly `length` bytes from socket."""
        data = b''
        while len(data) < length:
            more = self._socket.recv(length - len(data))
            if not more:
                self.disconnect()
                raise CommError('connection closed by peer')
            data += more
        return data

    def _translate_error(self, errorcode):
        return ADS_ERRORS.get(errorcode, f'Unknown code {errorcode:#06x}')

    def _set_route(self, sockname):
        """Try to set up an ADS route on the target."""
        routename = 'zapf-' + sockname[0]
        udpsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        mynetid = pack('<Q', self._myamsnetaddr)[:6]
        # The default password is different for different TwinCAT versions.
        # Just try both.
        for password in (b'', b'1'):
            message = UDP_MESSAGE.pack(
                UDP_MAGIC,
                UDP_ADD_ROUTE,
                self._myamsnetaddr,
                5,
                UDP_ROUTENAME, 25, routename.encode(),
                UDP_NETID, 6, mynetid,
                UDP_USERNAME, 14, b'Administrator',
                UDP_PASSWORD, 2, password,
                UDP_HOST, 16, sockname[0].encode(),
            )
            udpsock.sendto(message, (self._iphostport[0], BECKHOFF_UDP_PORT))
        time.sleep(0.5)


# https://infosys.beckhoff.com/english.php?content=../content/1033/tc3_ads_intro_howto/374277003.html&id=2736996179007627436
ADS_ERRORS = {
    0x001: 'Internal error',
    0x002: 'No real-time',
    0x003: 'Allocation locked - memory error',
    0x004: 'Mailbox full - ADS message could not be sent',
    0x005: 'Wrong receive HMSG',
    0x006: 'Target port not found, possibly ADS server not started',
    0x007: 'Target machine not found, possibly missing ADS routes',
    0x008: 'Unknown command ID',
    0x009: 'Invalid task ID',
    0x00A: 'No IO',
    0x00B: 'Unknown AMS command',
    0x00C: 'Win32 error',
    0x00D: 'Port not connected',
    0x00E: 'Invalid AMS length',
    0x00F: 'Invalid AMS NetID',
    0x010: 'Low installation level',
    0x011: 'No debugging available',
    0x012: 'Port disabled - system service not started',
    0x013: 'Port already connected',
    0x014: 'AMS Sync Win32 error',
    0x015: 'AMS Sync timeout',
    0x016: 'AMS Sync error',
    0x017: 'AMS Sync no index map',
    0x018: 'Invalid AMS port',
    0x019: 'No memory',
    0x01A: 'TCP send error',
    0x01B: 'Host unreachable',
    0x01C: 'Invalid AMS fragment',
    0x01D: 'TLS send error - secure ADS connection failed',
    0x01E: 'Access denied - secure ADS access denied',

    0x500: 'Router: no locked memory',
    0x501: 'Router: memory size could not be changed',
    0x502: 'Router: mailbox full',
    0x503: 'Router: debug mailbox full',
    0x504: 'Router: port type is unknown',
    0x505: 'Router is not initialized',
    0x506: 'Router: desired port number is already assigned',
    0x507: 'Router: port not registered',
    0x508: 'Router: maximum number of ports reached',
    0x509: 'Router: port is invalid',
    0x50A: 'Router is not active',
    0x50B: 'Router: mailbox full for fragmented messages',
    0x50C: 'Router: fragment timeout occurred',
    0x50D: 'Router: port removed',

    0x700: 'General device error',
    0x701: 'Service is not supported by server',
    0x702: 'Invalid index group',
    0x703: 'Invalid index offset',
    0x704: 'Reading/writing not permitted',
    0x705: 'Parameter size not correct',
    0x706: 'Invalid parameter value(s)',
    0x707: 'Device is not in a ready state',
    0x708: 'Device is busy',
    0x709: 'Invalid OS context -> use multi-task data access',
    0x70A: 'Out of memory',
    0x70B: 'Invalid parameter value(s)',
    0x70C: 'Not found (files, ...)',
    0x70D: 'Syntax error in command or file',
    0x70E: 'Objects do not match',
    0x70F: 'Object already exists',
    0x710: 'Symbol not found',
    0x711: 'Symbol version invalid -> create a new handle',
    0x712: 'Server is in an invalid state',
    0x713: 'AdsTransMode not supported',
    0x714: 'Notification handle is invalid',
    0x715: 'Notification client not registered',
    0x716: 'No more notification handles',
    0x717: 'Notification size too large',
    0x718: 'Device not initialized',
    0x719: 'Device has a timeout',
    0x71A: 'Query interface failed',
    0x71B: 'Wrong interface required',
    0x71C: 'Class ID is invalid',
    0x71D: 'Object ID is invalid',
    0x71E: 'Request is pending',
    0x71F: 'Request is aborted',
    0x720: 'Signal warning',
    0x721: 'Invalid array index',
    0x722: 'Symbol not active -> release handle and try again',
    0x723: 'Access denied',
    0x724: 'No license found -> activate license',
    0x725: 'License expired',
    0x726: 'License exceeded',
    0x727: 'License invalid',
    0x728: 'Invalid system ID in license',
    0x729: 'License not time limited',
    0x72A: 'License issue time in the future',
    0x72B: 'License time period too long',
    0x72C: 'Exception in device specific code -> check each device',
    0x72D: 'License file read twice',
    0x72E: 'Invalid signature',
    0x72F: 'Invalid public key certificate',
    0x730: 'Public key not known from OEM',
    0x731: 'License not valid for this system ID',
    0x732: 'Demo license prohibited',
    0x733: 'Invalid function ID',
    0x734: 'Outside the valid range',
    0x735: 'Invalid alignment',
    0x736: 'Invalid platform level',
    0x737: 'Context - forward to passive level',
    0x738: 'Content - forward to dispatch level',
    0x739: 'Context - forward to real-time',

    0x740: 'General client error',
    0x741: 'Invalid parameter at service',
    0x742: 'Polling list is empty',
    0x743: 'Var connection already in use',
    0x744: 'Invoke ID in use',
    0x745: 'Timeout elapsed -> check route setting',
    0x746: 'Error in Win32 subsystem',
    0x747: 'Invalid client timeout value',
    0x748: 'ADS port not opened',
    0x749: 'No AMS address',
    0x750: 'Internal error in ADS sync',
    0x751: 'Hash table overflow',
    0x752: 'Key not found in hash',
    0x753: 'No more symbols in cache',
    0x754: 'Invalid response received',
    0x755: 'Sync port is locked',

    0x1000: 'Internal error in real-time system',
    0x1001: 'Timer value not valid',
    0x1002: 'Task pointer has invalid value 0',
    0x1003: 'Stack pointer has invalid value 0',
    0x1004: 'Requested task priority already assigned',
    0x1005: 'No free Task Control Block',
    0x1006: 'No free semaphores',
    0x1007: 'No free space in the queue',
    0x100D: 'External sync interrupt already applied',
    0x100E: 'No external sync interrupt applied',
    0x100F: 'External sync interrupt application failed',
    0x1010: 'Call of service function in wrong context',
    0x1017: 'Intel VT-x not supported',
    0x1018: 'Intel VT-x not enabled in BIOS',
    0x1019: 'Missing function in Intel VT-x',
    0x101A: 'Activation of Intel VT-x failed',
}
