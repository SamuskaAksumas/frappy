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

"""Implements communication via Modbus/TCP."""

import re
import socket
from struct import Struct, pack, unpack, unpack_from

from zapf import ApiError, CommError
from zapf.proto import Protocol

MB_ADDR_RE = re.compile(r'modbus://(.+?)(:(\d+))?(?:/(\d+)?)?$')

DEFAULT_PORT = 502

FUNC_READ_HOLDING = 3
FUNC_WRITE_MULTI = 16

HEADER = Struct('>H2xHBB')
HEADER_SIZE = HEADER.size
TWO_WORDS = Struct('>HH')


class ModbusProtocol(Protocol):
    OFFSETS = [0, 0x6000, 0x8000]

    def __init__(self, url, log):
        adr = MB_ADDR_RE.match(url)
        if not adr:
            raise ApiError('invalid Modbus address, must be '
                           'modbus://host[:port]/slave')
        host = adr.group(1)
        if adr.group(2):
            port = int(adr.group(3))
        else:
            port = DEFAULT_PORT
        self._slave = int(adr.group(4) or 0)
        self._iphostport = (host, port)
        self._socket = None
        self._trans_id = 1

        super().__init__(url, log)

    def connect(self):
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self._socket.connect(self._iphostport)
            self.log.info('connected to %s', self.url)
        except (OSError, ValueError) as err:
            raise CommError(f'could not connect to {self.url}: {err}') from err
        self._connect_callback(True)
        self.connected = True

    def disconnect(self):
        try:
            self._socket.shutdown(socket.SHUT_RDWR)
            self._socket.close()
        except OSError:
            pass
        self._socket = None
        self._connect_callback(False)
        self.connected = False

    def read(self, addr, length):
        if not self.connected:
            self.reconnect()
        assert addr % 2 == 0
        result = b''
        while length:
            plen = min(length, 250)
            nregs = (plen + 1) // 2
            payload = TWO_WORDS.pack((self.offset + addr) // 2, nregs)
            try:
                reply = self._comm(FUNC_READ_HOLDING, payload, 2*nregs + 1)
            except OSError as err:
                self.log.exception('during read')
                self.disconnect()
                raise CommError(f'IO error during read: {err}') from err
            reply_regs = unpack_from(f'>{nregs}H', reply, 1)
            result += pack(f'<{nregs}H', *reply_regs)[:plen]
            length -= plen
            addr += plen
        return result

    def write(self, addr, data):
        if not self.connected:
            self.reconnect()
        if len(data) > 250:
            raise CommError(f'cannot write {len(data)} bytes over Modbus')
        # odd addrs and data lengths should never be needed since the
        # protocol is defined with 16-bit units in mind
        assert addr % 2 == 0 and len(data) % 2 == 0
        nregs = len(data) // 2
        payload = pack(f'>HHB{nregs}H', (self.offset + addr) // 2,
                       nregs, len(data), *unpack(f'<{nregs}H', data))
        try:
            reply = self._comm(FUNC_WRITE_MULTI, payload, 4)
        except OSError as err:
            self.log.exception('during write')
            self.disconnect()
            raise CommError(f'IO error during write: {err}') from err
        if reply != payload[:4]:
            raise CommError('invalid data in write reply')

    def _comm(self, function, payload, exp_len):
        self._trans_id = (self._trans_id + 1) & 0xFFFF
        msg = HEADER.pack(self._trans_id, len(payload) + 2,
                          self._slave, function) + payload
        expected = HEADER_SIZE + exp_len
        self._socket.sendall(msg)
        reply = b''
        rephdr = None
        while len(reply) < expected:
            data = self._socket.recv(expected - len(reply))
            if not data:
                raise CommError('no data in read')
            reply += data
            if len(reply) >= HEADER_SIZE and not rephdr:
                rephdr = HEADER.unpack_from(reply)
                if rephdr[0] != self._trans_id:
                    raise CommError('invalid transaction ID in reply')
                if rephdr[2] != self._slave:
                    raise CommError('invalid slave ID in reply')
                if rephdr[3] != function:
                    self._translate_error(reply[HEADER_SIZE:])
                if rephdr[1] != exp_len + 2:
                    raise CommError('unexpected length in reply')
        if rephdr[1] != len(reply) - 6:
            raise CommError('invalid length in reply or packet truncated')
        return reply[HEADER_SIZE:]

    def _translate_error(self, data):
        raise CommError('modbus: ' +
                        EXC_STRINGS.get(data[0], f'exception {data[0]}'))


EXC_STRINGS = {
    1: 'illegal function',
    2: 'illegal data address',
    3: 'illegal data value',
    4: 'slave device failure',
    5: 'acknowledge',
    6: 'slave device busy',
    7: 'negative acknowledge',
    8: 'memory parity error',
    10: 'gateway path unavailable',
    11: 'gateway target device failed to respond',
}
