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


class NumProxy:

    def __add__(self, other):
        if isinstance(other, NumProxy):
            other = other.value
        return self.value + other

    def __radd__(self, other):
        if isinstance(other, NumProxy):
            other = other.value
        return other + self.value

    def __sub__(self, other):
        if isinstance(other, NumProxy):
            other = other.value
        return self.value - other

    def __rsub__(self, other):
        if isinstance(other, NumProxy):
            other = other.value
        return other - self.value

    def __mul__(self, other):
        if isinstance(other, NumProxy):
            other = other.value
        return self.value * other

    def __rmul__(self, other):
        if isinstance(other, NumProxy):
            other = other.value
        return other * self.value

    def __floordiv__(self, other):
        if isinstance(other, NumProxy):
            other = other.value
        return self.value // other

    def __rfloordiv__(self, other):
        if isinstance(other, NumProxy):
            other = other.value
        return other // self.value

    def __truediv__(self, other):
        if isinstance(other, NumProxy):
            other = other.value
        return self.value / other

    def __rtruediv__(self, other):
        if isinstance(other, NumProxy):
            other = other.value
        return other / self.value

    def __mod__(self, other):
        if isinstance(other, NumProxy):
            other = other.value
        return self.value % other

    def __rmod__(self, other):
        if isinstance(other, NumProxy):
            other = other.value
        return other % self.value

    def __divmod__(self, other):
        if isinstance(other, NumProxy):
            other = other.value
        return divmod(self.value, other)

    def __rdivmod__(self, other):
        if isinstance(other, NumProxy):
            other = other.value
        return divmod(other, self.value)

    def __and__(self, other):
        if isinstance(other, NumProxy):
            other = other.value
        return self.value & other

    def __rand__(self, other):
        if isinstance(other, NumProxy):
            other = other.value
        return other & self.value

    def __or__(self, other):
        if isinstance(other, NumProxy):
            other = other.value
        return self.value | other

    def __ror__(self, other):
        if isinstance(other, NumProxy):
            other = other.value
        return other | self.value

    def __xor__(self, other):
        if isinstance(other, NumProxy):
            other = other.value
        return self.value ^ other

    def __rxor__(self, other):
        if isinstance(other, NumProxy):
            other = other.value
        return other ^ self.value

    def __rshift__(self, other):
        if isinstance(other, NumProxy):
            other = other.value
        return self.value >> other

    def __rrshift__(self, other):
        if isinstance(other, NumProxy):
            other = other.value
        return other >> self.value

    def __lshift__(self, other):
        if isinstance(other, NumProxy):
            other = other.value
        return self.value << other

    def __rlshift__(self, other):
        if isinstance(other, NumProxy):
            other = other.value
        return other << self.value

    def __lt__(self, other):
        if isinstance(other, NumProxy):
            other = other.value
        return self.value < other

    def __le__(self, other):
        if isinstance(other, NumProxy):
            other = other.value
        return self.value <= other

    def __ge__(self, other):
        if isinstance(other, NumProxy):
            other = other.value
        return self.value >= other

    def __gt__(self, other):
        if isinstance(other, NumProxy):
            other = other.value
        return self.value > other

    def __eq__(self, other):
        if isinstance(other, NumProxy):
            other = other.value
        return self.value == other

    def __ne__(self, other):
        if isinstance(other, NumProxy):
            other = other.value
        return self.value != other

    def __bool__(self):
        return self.value.__bool__()

    def __neg__(self):
        return self.value.__neg__()

    def __pos__(self):
        return self.value.__pos__()

    def __abs__(self):
        return self.value.__abs__()

    def __invert__(self):
        return self.value.__invert__()

    def __index__(self):
        return self.value.__index__()

    def __int__(self):
        return self.value.__int__()

    def __float__(self):
        return self.value.__float__()

    def __hash__(self):
        return self.value.__hash__()
