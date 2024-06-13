#  -*- coding: utf-8 -*-
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

import collections
import struct
import threading
from itertools import islice

from zapf.simulator.util import NumProxy

# The Memory is a thread-local singleton.  It sets itself as `mem` on this
# object on construction.
local = threading.local()


class Memory:
    # Allocates addresses for variables.

    def __init__(self):
        # %M* is at 0x10000
        # %I* is at 0x20000
        # %Q* is at 0x30000
        # dyn is at 0x40000 up
        self.dyn_addr = 0x40000
        self.allocations = []
        self.allocations_sorted = False
        self.allocs_by_size = {}
        local.mem = self

    def sort_allocations(self):
        self.allocs_by_size.clear()
        self.allocations.sort(key=lambda a: a[:2])
        # Allocations are sorted by size first, then by address - this lets us
        # find the "innermost" allocation for a given byte range.  But in order
        # to speed up this search, we cache the index where to start the search
        # for any given size.
        last = 0
        for (i, (osize, _, _)) in enumerate(self.allocations):
            if osize > last:
                for nsize in range(last + 1, osize + 1):
                    self.allocs_by_size[nsize] = i
                last = osize
        self.allocations_sorted = True

    def new(self, size):
        addr = self.dyn_addr
        self.dyn_addr += size
        return addr

    def map(self, addr, obj):
        """Map a new allocation."""
        self.allocations.append((obj.sizeof(), addr, obj))
        self.allocations_sorted = False

    def get(self, addr, size):
        """Return the object that addr belongs to, and the offset in it,
        if there are at least size bytes left.
        """
        if not self.allocations_sorted:
            self.sort_allocations()
        # Find the smallest allocation that encompasses the whole given range.
        # If the size is not in "alloc_size_start", the requested size is
        # bigger than any single object, which is not allowed.
        if size in self.allocs_by_size:
            allocs = islice(self.allocations, self.allocs_by_size[size], None)
            for (osize, oaddr, obj) in allocs:
                if oaddr <= addr and addr + size <= oaddr + osize:
                    return (obj, addr - oaddr)
        raise RuntimeError('no value or addressing across value boundary')

    def read(self, addr, size):
        """Read memory."""
        (obj, offset) = self.get(addr, size)
        return obj.mem_read()[offset : offset + size]

    def write(self, addr, data):
        """Write memory."""
        (obj, offset) = self.get(addr, len(data))
        obj.mem_write(offset, data)


class Value:
    """Represents a place in memory for a variable."""

    @classmethod
    def alloc(cls, value, at=None):
        """Allocates an address for the value (if not given)."""
        if at is None:
            at = local.mem.new(cls.sizeof())
        val = cls(value, at)
        local.mem.map(at, val)
        return val

    def alloc_self(self, at=None):
        """Shortcut for self.alloc(self)."""
        return self.alloc(self, at)

    @classmethod
    def default(cls, at=None):
        """Allocates a value with a default value."""
        return cls.alloc(cls.DEFAULT, at)

    @classmethod
    def unwrap(cls, value):
        """Retrieves the inner value from an rvalue assigned to this lvalue."""
        if isinstance(value, cls):
            return value.value
        return value

    @classmethod
    def sizeof(cls):
        """Returns the size of the value in memory."""
        raise NotImplementedError

    def __init__(self, value, addr):
        self.addr = addr
        self.assign(value)

    def __repr__(self):
        return repr(self.value)

    def assign(self, value):
        self.value = self.__class__.unwrap(value)

    def mem_read(self):
        raise NotImplementedError

    def mem_write(self, offset, data):
        raise NotImplementedError


class Integral(NumProxy, Value):
    DEFAULT = 0
    WIDTH = 0
    SIGNED = False
    MEMFMT = ''

    @classmethod
    def sizeof(cls):
        return cls.WIDTH // 8

    @classmethod
    def unwrap(cls, value):
        if isinstance(value, Integral):
            value = value.value
        limit = 1 << cls.WIDTH
        value %= limit
        if cls.SIGNED and value >= limit // 2:
            value -= limit
        # if cls.SIGNED:
        #     limit = 1 << (cls.WIDTH - 1)
        #     if not -limit <= value < limit:
        #         raise RuntimeError('out of range assignment to %s: %s' % (
        #             cls.__name__, value))
        # else:
        #     if value < 0 or value >= 1 << cls.WIDTH:
        #         raise RuntimeError('out of range assignment to %s: %s' % (
        #             cls.__name__, value))
        return value

    def mem_read(self):
        return struct.pack(self.MEMFMT, self.value)

    def mem_write(self, offset, data):
        if offset != 0:
            raise RuntimeError(f'partial {self.__class__.__name__} write '
                               f'(offset {offset}, datalen {len(data)})')
        self.value, = struct.unpack(self.MEMFMT, data)

    def __getitem__(self, i):
        # Bit access: a[[i]]
        return (self.value >> i[0]) & 1

    def __setitem__(self, i, val):
        mask = ~(1 << i[0])
        self.value = (self.value & mask) | ((val & 1) << i[0])


class byte(Integral):
    WIDTH = 8
    SIGNED = False
    MEMFMT = 'B'


class word(Integral):
    WIDTH = 16
    SIGNED = False
    MEMFMT = 'H'


class dword(Integral):
    WIDTH = 32
    SIGNED = False
    MEMFMT = 'I'


class lword(Integral):
    WIDTH = 64
    SIGNED = False
    MEMFMT = 'Q'


class boolean(Integral):
    WIDTH = 1
    SIGNED = False
    MEMFMT = 'B'

    @classmethod
    def sizeof(cls):
        return 1


class real(NumProxy, Value):
    DEFAULT = 0.0

    @classmethod
    def sizeof(cls):
        return 4

    def mem_read(self):
        return struct.pack('f', self.value)

    def mem_write(self, offset, data):
        if offset != 0:
            raise RuntimeError(f'partial {self.__class__.__name__} write '
                               f'(offset {offset}, datalen {len(data)})')
        self.value, = struct.unpack('f', data)


class lreal(NumProxy, Value):
    DEFAULT = 0.0

    @classmethod
    def sizeof(cls):
        return 8

    def mem_read(self):
        return struct.pack('d', self.value)

    def mem_write(self, offset, data):
        if offset != 0:
            raise RuntimeError(f'partial {self.__class__.__name__} write '
                               f'(offset {offset}, datalen {len(data)})')
        self.value, = struct.unpack('d', data)


class anystring(Value):
    SLEN = 0
    DEFAULT = ''

    @classmethod
    def sizeof(cls):
        return cls.SLEN

    @classmethod
    def unwrap(cls, value):
        if isinstance(value, anystring):
            value = value.value
        if len(value.encode()) >= cls.SLEN:
            raise RuntimeError(f'string too long ({cls.SLEN-1} bytes max)')
        return value

    def mem_read(self):
        return self.value.encode() + b'\0' * (self.SLEN - len(self.value.encode()))

    def mem_write(self, offset, data):
        if offset != 0:
            raise RuntimeError(f'partial {self.__class__.__name__} write '
                               f'(offset {offset}, datalen {len(data)})')
        self.value = data.split('\0', 1)[0]

    def __len__(self):
        return self.value.__len__()


def string(slen):
    # string(X) adds a trailing null byte, so size is X+1
    return type(f'string_{slen}', (anystring,), dict(SLEN=slen + 1))


class anyarray(Value):
    LENGTH = 0
    IMIN = 0
    INNER = None
    DEFAULT = []

    @classmethod
    def sizeof(cls):
        return cls.LENGTH * cls.INNER.sizeof()

    def __init__(self, value, addr):
        self.addr = addr
        self.value = []
        value = self.__class__.unwrap(value)
        step = self.INNER.sizeof()
        if len(value) > self.LENGTH:
            raise RuntimeError(f'too many values in array assignment '
                               f'(max {self.LENGTH})')
        for i in range(self.LENGTH):
            self.value.append(self.INNER.alloc(
                value[i] if i < len(value) else self.INNER.DEFAULT,
                at=addr + i * step))

    def assign(self, value):
        raise RuntimeError('array assign')

    def __getitem__(self, i):
        if not self.IMIN <= i < self.IMIN + self.LENGTH:
            raise RuntimeError(f'array access out of range (index {i}, '
                               f'min {self.IMIN}, length {self.LENGTH})')
        return self.value[i - self.IMIN]

    def __setitem__(self, i, val):
        if not self.IMIN <= i < self.IMIN + self.LENGTH:
            raise RuntimeError(f'array access out of range (index {i}, '
                               f'min {self.IMIN}, length {self.LENGTH})')
        self.value[i - self.IMIN].assign(val)

    def __len__(self):
        return self.LENGTH

    def __repr__(self):
        return f'<{", ".join(map(repr, self.value))}>'

    def mem_read(self):
        return b''.join(v.mem_read() for v in self.value)

    def mem_write(self, offset, data):
        step = self.INNER.sizeof()
        if offset % step != 0:
            raise RuntimeError('partial array element write')
        if len(data) % step != 0:
            raise RuntimeError('partial array element write')
        i = offset // step
        while data:
            d, data = data[:step], data[step:]
            self.value[i].mem_write(0, d)
            i += 1


def array(innertype, imin, imax):
    length = imax - imin + 1
    return type(f'array_{length}', (anyarray,), dict(LENGTH=length,
                                                     IMIN=imin,
                                                     INNER=innertype))


class Var:
    """Represents a field in a struct (which includes the "variable struct"
    automatically generated for a function decorated with @program).
    """

    def __init__(self, dtype, default=None, *, at=None):
        self.dtype = dtype
        self.default = default if default is not None else dtype.DEFAULT
        self.at = None
        if at is not None:
            if at.startswith('%MB'):
                self.at = int(at[3:]) + 0x10000
            elif at.startswith('%IB'):
                self.at = int(at[3:]) + 0x20000
            elif at.startswith('%QB'):
                self.at = int(at[3:]) + 0x30000
            else:
                raise RuntimeError(f'addr spec {at!r} not supported')

    def __get__(self, obj, obj_class):
        if obj is None:
            return self
        return obj.__dict__[self]

    def __set__(self, obj, value):
        if obj is None:
            return
        obj.__dict__[self].assign(self.dtype.unwrap(value))


class StructMeta(type):
    @classmethod
    def __prepare__(cls, _name, _bases):
        return collections.OrderedDict()

    def __init__(cls, _name, _bases, attrs):
        cls.VARS = []
        cls.OFFSET = {}
        size = 0
        for (varname, var) in attrs.items():
            if isinstance(var, Var):
                cls.VARS.append((varname, var))
                cls.OFFSET[varname] = size
                # XXX alignment!
                size += var.dtype.sizeof()
        cls.SIZE = size


class Struct(Value, metaclass=StructMeta):
    DEFAULT = Ellipsis

    @classmethod
    def sizeof(cls):
        return cls.SIZE

    def __init__(self, value=None, addr=None, **pvars):
        self.addr = addr
        for (name, var) in self.VARS:
            self.__dict__[var] = var.dtype.alloc(
                getattr(value, name, pvars.pop(name, var.default)),
                at=var.at or (addr + self.OFFSET[name]
                              if addr is not None else None))
        if pvars:
            raise RuntimeError(f'unknown variable(s) in struct: {pvars}')

    def assign(self, value):
        if value is Ellipsis:
            # XXX set defaults necessary?
            # keep defaults
            return
        if isinstance(value, self.__class__):
            # XXX reassign individual values!
            raise RuntimeError('struct reassign')
        raise RuntimeError('trying to assign struct of wrong kind')

    def __repr__(self):
        items = []
        for (name, var) in self.VARS:
            items.append((name, self.__dict__[var]))
        return self.__class__.__name__ + ' { ' + \
            ', '.join(f'{x[0]} => {x[1]!r}' for x in items) + ' }'

    def mem_read(self):
        return b''.join(self.__dict__[var].mem_read()
                        for (_, var) in self.VARS)

    def mem_write(self, offset, data):
        for (name, var) in self.VARS:
            var_skip = offset - self.OFFSET[name]
            var_size = var.dtype.sizeof()
            # skip elements before the first offset
            if var_skip >= var_size:
                continue
            write_size = min(var_size - var_skip, len(data))
            d, data = data[:write_size], data[write_size:]
            self.__dict__[var].mem_write(var_skip, d)
            offset += write_size
            if not data:
                break


class Globals(Struct):
    pass


def program(**pvars):
    def deco(func):
        var_struct = type(f'{func.__name__}_vars', (Struct,), pvars)
        instance = var_struct()

        def new_func():
            func(instance)

        new_func.is_program = True
        return new_func
    return deco
