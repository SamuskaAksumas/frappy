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
#   Enrico Faulhaber <enrico.faulhaber@frm2.tum.de>
#
# *****************************************************************************

"""Some helper types and functions."""

import itertools


class EmptyBitField:
    """Integer-like with support for accessing and setting bit ranges.

    This is a baseclass. To use, create a nonempty subclass with BitFields().
    """

    def __init__(self, value=0):
        self._value = int(value)

    def __call__(self, value):
        self._value = int(value)

    def __int__(self):
        return int(self._value)


def BitFields(**fields):
    """A class that lets one access individual bit ranges in a single integer.

    The fields given as keyword arguments are either single bit number or an
    inclusive range of bits given as a (start, end) tuple.  Ranges may overlap.

    Usage example:

    >>> BF = BitFields(LSB=0, x=(3, 0), y=(7, 0), MSB=7)
    >>> bf = BF()
    >>> bf(35)         # update internal state to new value
    >>> bf.LSB         # check LSB (bit0)
    1
    >>> bf.x = 2       # modify bits 3..0 to be 2 = 0b0010
    >>> bf.y           # check bits 0..7 = 0x22 = 34
    34
    >>> int(bf)        # print internal state
    34
    """
    items = {}
    for name, spec in fields.items():
        if isinstance(spec, tuple):
            mask = (1 << (max(spec) - min(spec) + 1)) - 1
            shift = min(spec)
        else:
            mask = 1
            shift = spec

        def getbits(self, shift=shift, mask=mask):
            # pylint: disable=protected-access
            return (self._value >> shift) & mask

        def setbits(self, value, shift=shift, mask=mask):
            # pylint: disable=protected-access
            masked = (self._value >> shift) & mask
            self._value ^= (masked ^ (value & mask)) << shift

        items[name] = property(getbits, setbits, doc=name)
    return type('BitFields', (EmptyBitField,), items)


class UncasedMap(dict):
    """A dict subclass that allows two-way lookup of unique key-value pairs.

    In addition to the initial keys of the given mapping (by key-value pairs
    or a ``**kwds`` dictionary), the keys are also added in lowercase form,
    and there is a backwards mapping from value to key.
    """

    def __init__(self, *args, **kwds):
        dict.__init__(self)
        self._keys = []
        for k, v in itertools.chain(kwds.items(), args):
            self[k] = v
            self[k.lower()] = v
            self[v] = k
            self._keys.append(k)

    def __getattr__(self, k):
        return self[k]
