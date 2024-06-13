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

"""Implements communication via a Tango device."""

import re
from struct import pack, unpack

from zapf import ApiError, CommError, Error
from zapf.proto import Protocol

try:
    import PyTango
except ImportError:
    PyTango = None

TANGO_ADDR_RE = tangodev_re = re.compile(
    r'tango://([\w.-]+:[\d]+/)?([\w-]+/){2}[\w-]+(#dbase=(no|yes))?$')


class TangoProtocol(Protocol):
    # Since a Tango device can represent both Modbus and ADS, we have to try
    # all offsets.
    OFFSETS = [0, 0x6000, 0x8000]

    def __init__(self, url, log):
        if not PyTango:
            raise Error('install PyTango to use the tango:// protcol')

        if not TANGO_ADDR_RE.match(url):
            raise ApiError('invalid Tango address, must be '
                           'tango://[database:port/]domain/family/member[#dbase=no]')
        self._proxy = None

        super().__init__(url, log)

    def connect(self):
        try:
            self._proxy = PyTango.DeviceProxy(self.url)
            self._proxy.state
        except PyTango.DevFailed as err:
            raise CommError(f'cannot connect to Tango device: {err}') from err
        except AttributeError:
            raise CommError('Tango device exists, but seems to be not '
                            'running') from None
        try:
            self._proxy.ReadInputBytes
            self._proxy.WriteOutputBytes
            self.read = self._read_ads
            self.write = self._write_ads
        except AttributeError:
            try:
                self._proxy.ReadOutputWords
                self._proxy.WriteOutputWords
                self.read = self._read_modbus
                self.write = self._write_modbus
            except AttributeError:
                raise CommError('Tango device seems to have the '
                                'wrong interface (needs to be Profibus '
                                'or Modbus)') from None
        self._connect_callback(True)
        self.connected = True

    def disconnect(self):
        self._proxy = None
        self._connect_callback(False)
        self.connected = False

    # These methods are overwritten by the protocol-specific ones below.

    def read(self, addr, length):
        raise ApiError('not connected')

    def write(self, addr, data):
        raise ApiError('not connected')

    # I/O using an ADS Tango device ("Profibus" Entangle interface)

    def _read_ads(self, addr, length):
        if not self.connected:
            self.reconnect()
        try:
            result = self._proxy.ReadInputBytes((addr, length))  # no offset
        except PyTango.DevFailed as err:
            self.log.exception('during read')
            raise CommError(f'Tango error during read: {err}') from err
        return bytes(result)

    def _write_ads(self, addr, data):
        if not self.connected:
            self.reconnect()
        try:
            self._proxy.WriteOutputBytes(((addr,) + tuple(data)))
        except PyTango.DevFailed as err:
            self.log.exception('during write')
            raise CommError(f'Tango error during write: {err}') from err

    # I/O using a Modbus Tango device

    def _read_modbus(self, addr, length):
        if not self.connected:
            self.reconnect()
        assert addr % 2 == 0
        nregs = (length + 1) // 2
        try:
            result = self._proxy.ReadOutputWords(((self.offset + addr) // 2,
                                                  nregs))
        except PyTango.DevFailed as err:
            self.log.exception('during read')
            raise CommError(f'Tango error during read: {err}') from err
        return pack(f'<{nregs}H', *result)[:length]

    def _write_modbus(self, addr, data):
        if not self.connected:
            self.reconnect()
        assert addr % 2 == 0 and len(data) % 2 == 0
        nregs = len(data) // 2
        payload = ((self.offset + addr) // 2,) + unpack(f'<{nregs}H', data)
        try:
            self._proxy.WriteOutputWords(payload)
        except PyTango.DevFailed as err:
            self.log.exception('during write')
            raise CommError(f'Tango error during write: {err}') from err
