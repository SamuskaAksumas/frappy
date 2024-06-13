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

"""Implements communication to a simulated PLC running in-process."""

import threading
import time

from zapf import ApiError, CommError
from zapf.proto import Protocol
from zapf.simulator import runtime


class SimProtocol(Protocol):
    OFFSETS = [0x10000]

    def __init__(self, url, log):
        self._plcfunc = url
        self._cond = threading.Condition()
        self._thread = None
        self._stopflag = False
        self._exception = None

        super().__init__(url, log)

    def connect(self):
        if self._thread and self._thread.is_alive():
            return
        self._stopflag = False
        self._thread = threading.Thread(target=self._plc_thread, daemon=True,
                                        name=f'SimPLC {self.url}')
        self._thread.start()
        # wait for one cycle, and raise if an exception occurred
        with self._cond:
            self._cond.wait()
        if self._exception:
            raise self._exception
        self._connect_callback(True)
        self.connected = True

    def disconnect(self):
        if self._thread:
            self._stopflag = True
            self._thread.join()
        self._connect_callback(False)
        self.connected = False

    def read(self, addr, length):
        with self._cond:
            self._cond.wait()
            try:
                return self.my_mem.read(self.offset + addr, length)
            except RuntimeError as err:
                raise CommError(f'reading {addr}({length}): {err}') from err

    def write(self, addr, data):
        with self._cond:
            self._cond.wait()
            try:
                self.my_mem.write(self.offset + addr, data)
            except RuntimeError as err:
                raise CommError(f'writing {addr}({len(data)}): {err}') from err

    def _plc_thread(self):
        self.my_mem = runtime.Memory()
        try:
            if not self._plcfunc.startswith('sim://'):
                raise Exception('URL should start with sim://')
            with open(self._plcfunc[6:], encoding='utf-8') as fp:
                code = fp.read()
            globs = {}
            # TODO: execute with the correct filename to get
            # filename  and code in tracebacks
            exec(code, globs)  # pylint: disable=exec-used
            self._plcfunc = globs['Main']
        except Exception as err:
            self._exception = ApiError(
                'invalid sim address, must be '
                'sim:///path/to/file.py '
                'with a Main() function')
            self.log.error('%s, aborting simulation', self._exception)
            self._exception.__cause__ = err
        while not self._stopflag:
            with self._cond:
                try:
                    if not self._exception:
                        self._plcfunc()
                except Exception as err:
                    self.log.exception('error in PLC simulation thread, '
                                       'aborting simulation')
                    self._exception = err
                self._cond.notify()
            time.sleep(.0005)
