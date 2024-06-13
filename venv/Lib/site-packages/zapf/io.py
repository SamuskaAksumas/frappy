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

"""IO abstraction."""

import threading
from struct import Struct, calcsize, pack, unpack

from zapf import CommError
from zapf.indexer import Indexer
from zapf.proto import Protocol

# "Float order" converters.

def float_normal(x):
    return x


def float_swapped(x):
    return b''.join(x[i+2:i+4] + x[i:i+2] for i in range(0, len(x), 4))


class PlcIO:
    """IO caching object, one per PLC connection."""

    def __init__(self, proto, log):
        self.log = log
        if isinstance(proto, Protocol):
            self.proto = proto
        else:
            self.proto = Protocol.create(proto, log)
            self.proto.connect()
        self.cache = Cache(self.proto, log)
        self.magic = None
        self.indexer = Indexer(self, log)
        self.byteorder = '<'
        self.floatconv = float_normal

    def set_callback(self, callback):
        self.cache.callback = callback

    def set_update_params(self, cycle_secs, retries=-1):
        self.cache.cycle = cycle_secs
        self.cache.retries = retries

    def start_cache(self):
        self.cache.start()

    def stop_cache(self):
        self.cache.stop()

    def disconnect(self):
        """Close the connection. This PlcIO object should not be used anymore
        afterwards.
        """
        self.stop_cache()
        self.proto.disconnect()

    # Cache related API.

    def register_cache_range(self, addr, length):
        """Register a cached area."""
        if length < 1:
            return
        self.cache.update_range(addr, length)
        self.sync_cache(wait=False)

    def sync_cache(self, wait=True):
        """To be called after a write to trigger a sync and wait for it"""
        if self.cache.running:
            self.cache.updated.clear()
            self.cache.trigger.set()
            if wait:
                self.cache.updated.wait(self.cache.cycle)

    # Higher level stuff.

    def detect_magic(self):
        if self.magic is not None:
            return self.magic

        def scan_offset(offset):
            for byteorder in '<>':
                self.byteorder = byteorder
                for floatconv in (float_normal, float_swapped):
                    self.floatconv = floatconv
                    try:
                        magic = self.read_f32(offset)
                    except CommError:
                        return False  # don't retry here
                    if 2014.0 <= magic < 2046.0:
                        byteorder = {'<': 'little', '>': 'big'}[self.byteorder]
                        floatconv = ', swapped floats' \
                            if self.floatconv is float_swapped else ''
                        self.log.info(f'found PLC according to {magic:.2f}: '
                                      f'{byteorder} endian{floatconv}')
                        self.magic = magic
                        self.proto.offset = offset
                        return True
            return False

        for offset in self.proto.OFFSETS:
            if scan_offset(offset):
                return self.magic
        raise CommError('did not find any MAGIC number')

    def float_from_dword(self, dw):
        return unpack('<f', self.floatconv(pack('<I', dw)))[0]

    # Read operations: cached accesses.

    def read_bytes(self, addr, num):
        return self.cache.get(addr, num)

    def read_fmt(self, addr, fmt):
        fmt = Struct(self.byteorder + fmt)
        return fmt.unpack(self.cache.get(addr, fmt.size))

    def read_u16(self, addr):
        return unpack(self.byteorder + 'H', self.cache.get(addr, 2))[0]

    def read_u16s(self, addr, num):
        return unpack(self.byteorder + str(num) + 'H',
                      self.cache.get(addr, 2*num))

    def read_i16(self, addr):
        return unpack(self.byteorder + 'h', self.cache.get(addr, 2))[0]

    def read_i16s(self, addr, num):
        return unpack(self.byteorder + str(num) + 'h',
                      self.cache.get(addr, 2*num))

    def read_u32(self, addr):
        return unpack(self.byteorder + 'I', self.cache.get(addr, 4))[0]

    def read_u32s(self, addr, num):
        return unpack(self.byteorder + str(num) + 'I',
                      self.cache.get(addr, 4*num))

    def read_i32(self, addr):
        return unpack(self.byteorder + 'i', self.cache.get(addr, 4))[0]

    def read_i32s(self, addr, num):
        return unpack(self.byteorder + str(num) + 'i',
                      self.cache.get(addr, 4*num))

    def read_u64(self, addr):
        return unpack(self.byteorder + 'Q', self.cache.get(addr, 8))[0]

    def read_u64s(self, addr, num):
        return unpack(self.byteorder + str(num) + 'Q',
                      self.cache.get(addr, 8*num))

    def read_i64(self, addr):
        return unpack(self.byteorder + 'q', self.cache.get(addr, 8))[0]

    def read_i64s(self, addr, num):
        return unpack(self.byteorder + str(num) + 'q',
                      self.cache.get(addr, 8*num))

    def read_f32(self, addr):
        return unpack(self.byteorder + 'f',
                      self.floatconv(self.cache.get(addr, 4)))[0]

    def read_f32s(self, addr, num):
        return unpack(self.byteorder + str(num) + 'f',
                      self.floatconv(self.cache.get(addr, 4*num)))

    def read_pctrl(self, addr, fmt=None, cached=True):
        # read a pctrl struct (controlword + value)
        if fmt is not None:
            fmt_ = self.byteorder + 'H' + fmt
        else:
            fmt_ = self.byteorder + 'H'
        size = calcsize(fmt_)
        assert size in (2, 6, 10)
        if cached:
            data = self.cache.get(addr, size)
        else:
            with self.cache.lock:
                data = self.proto.read(addr, size)
        if fmt == 'f':
            # honor floatconv, if required
            data = data[:2] + self.floatconv(data[2:])
        return (*unpack(fmt_, data), None)[:2]

    # Note: floatconv is not used for 64-bit floats, since the PLCs that would
    # need floatconv don't support them.

    def read_f64(self, addr):
        return unpack(self.byteorder + 'd', self.cache.get(addr, 8))[0]

    def read_f64s(self, addr, num):
        return unpack(f'{self.byteorder}{num}d', self.cache.get(addr, 8*num))

    # Write operations: go direct, but trigger a cache sync afterwards.

    def write_i16(self, addr, data):
        self.write_bytes(addr, pack(self.byteorder + 'h', data))

    def write_u16(self, addr, data):
        self.write_bytes(addr, pack(self.byteorder + 'H', data))

    def write_i32(self, addr, data):
        self.write_bytes(addr, pack(self.byteorder + 'i', data))

    def write_u32(self, addr, data):
        self.write_bytes(addr, pack(self.byteorder + 'I', data))

    def write_i64(self, addr, data):
        self.write_bytes(addr, pack(self.byteorder + 'q', data))

    def write_u64(self, addr, data):
        self.write_bytes(addr, pack(self.byteorder + 'Q', data))

    def write_f32(self, addr, data):
        self.write_bytes(addr,
                         self.floatconv(pack(self.byteorder + 'f', data)))

    def write_f64(self, addr, data):
        self.write_bytes(addr, pack(self.byteorder + 'd', data))

    def write_u16_f32(self, addr, u16, f32):
        self.write_bytes(addr, pack(self.byteorder + 'H', u16) +
                         self.floatconv(pack(self.byteorder + 'f', f32)))

    def write_f32_u16(self, addr, f32, u16):
        self.write_bytes(addr, self.floatconv(pack(self.byteorder + 'f', f32)) +
                         pack(self.byteorder + 'H', u16))

    def write_f32_u32(self, addr, f32, u32):
        self.write_bytes(addr, self.floatconv(pack(self.byteorder + 'f', f32)) +
                         pack(self.byteorder + 'I', u32))

    def write_pctrl(self, addr, fmt=None, pctrl=None, pvalue=None):
        if None in (fmt, pvalue):
            if pctrl is not None:
                self.write_u16(addr, pctrl)
            return
        if pctrl is None:
            fmt_ = self.byteorder + fmt
            addr += 2
            data = pack(fmt_, pvalue)
            if fmt == 'f':
                # honor floatconv, if required
                data = self.floatconv(data)
        else:
            fmt_ = self.byteorder + 'H' + fmt
            data = pack(fmt_, pctrl, pvalue)
            if fmt == 'f':
                # honor floatconv, if required
                data = data[:2] + self.floatconv(data[2:])
        self.write_bytes(addr, data)

    def write_bytes(self, addr, data):
        with self.cache.lock:
            self.proto.write(addr, data)
        self.sync_cache()

    def write_fmt(self, addr, fmt, *data):
        with self.cache.lock:
            data = pack(self.byteorder + fmt, *data)
            self.proto.write(addr, data)
        self.sync_cache()

    def write_u16s(self, addr, data):
        packed = pack(f'{self.byteorder}{len(data)}H', *data)
        with self.cache.lock:
            self.proto.write(addr, packed)
        self.sync_cache()

    def write_f32s_u16(self, addr, data, u16):
        packed = b''
        for val in data:
            packed += self.floatconv(pack(f'{self.byteorder}f', val))
        packed += pack(f'{self.byteorder}H', u16)
        with self.cache.lock:
            self.proto.write(addr, packed)
        self.sync_cache()

    def write_f32s_u32(self, addr, data, u32):
        packed = b''
        for val in data:
            packed += self.floatconv(pack(f'{self.byteorder}f', val))
        packed += pack(f'{self.byteorder}I', u32)
        with self.cache.lock:
            self.proto.write(addr, packed)
        self.sync_cache()

    def write_f64s_u32(self, addr, data, u32):
        packed = pack(f'{self.byteorder}{len(data)}dI', *data, u32)
        with self.cache.lock:
            self.proto.write(addr, packed)
        self.sync_cache()


class Cache:
    """The actual cache."""

    def __init__(self, proto, log):
        self.log = log
        self.proto = proto
        self.cycle = 0.5
        self.retries = -1
        self.callback = lambda: None
        # data bytes
        self._data = None
        # range to cache
        self._range = (0, 0)   # empty
        # range that is *actually* cached
        self._cached_range = (0, 0)  # empty
        self.lock = threading.RLock()
        self.trigger = threading.Event()  # trigger
        self.updated = threading.Event()  # ack
        self._retried = 0
        self.running = False
        self._thread = None

    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self.running = True
        self._thread = threading.Thread(target=self.run, daemon=True,
                                        name=f'Caching {self.proto.url}')
        self._thread.start()

    def stop(self):
        if self._thread:
            self.running = False
            self._thread.join()

    def update_range(self, addr, length):
        with self.lock:
            # first area to cache?
            if self._range[0] == self._range[1]:
                first = addr
                above = addr + length
            else:
                first = min(self._range[0], addr)
                above = max(self._range[1], addr + length)
            # cache range stores (start, end+1)
            self._range = first, above

    def get(self, addr, length):
        with self.lock:
            if addr >= self._cached_range[0] and \
               addr + length <= self._cached_range[1]:
                addr -= self._cached_range[0]
                return self._data[addr:addr + length]
            return self.proto.read(addr, length)

    def run(self):
        """Loop which updates the cache whenever cycle seconds have passed
        after the last update, or trigger was set.
        """
        while self.running:
            self.trigger.wait(self.cycle)
            self.trigger.clear()
            self.sync()

    def sync(self):
        """Update our cache and set the cache.updated event."""
        with self.lock:
            if self._range != (0, 0):
                try:
                    cached_data = self.proto.read(
                        self._range[0], self._range[1] - self._range[0])
                    if self._data is None:
                        self.log.info('initial cache update ok')
                    self._retried = 0
                except Exception as err:
                    self.log.warning('problem while trying to sync cache: %s',
                                     err)
                    self._cached_range = (0, 0)
                    self._data = None
                    self._retried += 1
                    if self._retried == self.retries:
                        self.log.info('cache sync aborted')
                        self.running = False
                        self.updated.set()
                        return
                else:
                    self._cached_range = self._range
                    self._data = cached_data
                    self.callback()
        self.updated.set()
