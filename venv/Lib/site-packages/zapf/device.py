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

"""Basic device abstraction for PILS devices."""

import time
from collections import namedtuple
from struct import calcsize

from zapf import ApiError, CommError, SpecError
from zapf.spec import DevStatus, ParamCMDs, ParamControl, StatusStruct16, \
    StatusStruct32, decode_bitfield
from zapf.spec.v_2021_09 import get_valueinfo
from zapf.table import Table

START16 = DevStatus.START << 12
START32 = DevStatus.START << 28


class Device:
    """Base class for all specific PILS devices.

    :param name: The device name.  This is not currently used by Zapf.
    :param addr: The base device address.
    :param typecode: The device's typecode.
    :param info: The "additional info" dictionary returned by the `.Scanner`.
    :param io: A `.PlcIO` instance to use for communication with the PLC.
    :param log: A `logging.Logger` to use for device related logging.
    """

    @classmethod
    def class_for(cls, typecode):
        return TYPECODE_MAP.get(typecode, (None,))[0]

    valuetype = None

    def __init__(self, name, addr, typecode, info, io, log):
        self.name = name
        self.typecode = typecode
        self.device_kind = typecode >> 8
        self.total_size = 2 * (typecode & 0xFF)
        self.info = info
        self.aux_spec = info['aux']
        self.errid_spec = info['errid']
        self._params = info['params']
        self._funcs = info['funcs']

        self._enum_r = info['enum_r']  # int -> str
        self._enum_w = info['enum_w']  # str -> int
        if info['enum_w']:
            self._enum_w_rev = dict((v, k) for (k, v) in self._enum_w.items())
        else:
            self._enum_w_rev = None

        typeinfo = TYPECODE_MAP[typecode]
        self.value_fmt = typeinfo.value_fmt
        self.value_size = calcsize(self.value_fmt)
        self.status_size = typeinfo.status_size
        self.num_values = typeinfo.num_values
        self.num_params = typeinfo.num_params

        self.addr = addr
        next_addr = addr + self.num_values * self.value_size
        if typeinfo.has_target:
            self.target_addr = next_addr
            next_addr += self.num_values * self.value_size
        else:
            self.target_addr = None
        if self.status_size:
            self.status_addr = next_addr
        else:
            self.status_addr = None
        if typeinfo.has_pctrl:
            self.pctrl_addr = self.status_addr + self.status_size
        else:
            self.pctrl_addr = None
        if typeinfo.num_params:
            self.param_addr = self.status_addr + (
                4 if (self.typecode >> 13) == 3 else 2 + self.status_size
            )
        else:
            self.param_addr = None

        if self.device_kind != 5:  # not applicable for MessageIO
            self.value_info = get_valueinfo(self.info, self.value_size)
        else:
            self.value_info = None

        self.log = log
        self.io = io
        self._init()

        io.register_cache_range(self.addr, self.total_size)
        self.tables = {}
        for (tname, tinfo) in info['tables'].items():
            self.tables[tname] = Table(io, self, tname, tinfo)

    def read_status(self):
        """Read the status information of the device.

        This is a tuple of ``(state, reason, aux, error_id)``.

        ``state`` is the 4-bit state (see `.DevStatus` for possible values).
        ``reason`` is the 4-bit reason code (see `.ReasonMap`).  ``aux``
        contains the up to 24 AUX bits.  ``error_id`` is a device-defined
        16-bit integer.

        For devices that do not have an error ID field, it is returned as zero.
        """
        if self.status_addr is None:
            if self.target_addr and self.target_addr != self.addr and \
               self.read_value() != self.read_target():
                return (DevStatus.BUSY, 0, 0, 0)
            return (DevStatus.IDLE, 0, 0, 0)
        if self.status_size == 2:
            s = StatusStruct16(self.io.read_u16(self.status_addr))
            return (s.STATE, s.REASON, s.AUX, 0)
        elif self.status_size == 4:
            s = StatusStruct32(self.io.read_u32(self.status_addr))
            return (s.STATE, s.REASON, s.AUX, 0)
        elif self.status_size == 6:
            value, err_id = self.io.read_fmt(self.status_addr, 'IH')
            s = StatusStruct32(value)
            return (s.STATE, s.REASON, s.AUX, err_id)
        raise SpecError('invalid status_size')

    def decode_aux(self, aux):
        """Decode the aux value returned by `.read_status`.

        Return a string for human consumption.
        """
        return decode_bitfield(aux, self.aux_spec)

    def decode_errid(self, errid):
        """Decode the errid value returned by `.read_status`.

        Return a string for human consumption.
        """
        return decode_bitfield(errid, self.errid_spec)

    def change_status(self, initial_states=(), final_state=0):
        """Change the state of the device.

        Since :ref:`not all state transitions are allowed <pils:state-graph>`,
        you can list the allowed states in *initial_states*.  If that list is
        given and the current state is not contained, ``False`` is returned.
        Else the *final_state* is written and ``True`` is returned.

        If the device does not have a status, ``False`` is always returned.
        """
        if self.status_addr is None:
            return False
        if initial_states:
            state = self.read_status()[0]
            if state not in initial_states:
                return False
        status = StatusStruct16()
        status.STATE = final_state
        if self.status_size == 2:
            self.io.write_u16(self.status_addr, int(status))
        else:
            self.io.write_u32(self.status_addr, int(status) << 16)
        return True

    def reset(self):
        """Reset the device.

        This is normally equivalent to `change_status(..., RESET)` with all
        error or unknown states as the initial states.  However, special
        devices like MessageIO can have a different reset strategy.
        """
        error_stati = set(range(16)) - {
            DevStatus.RESET,
            DevStatus.IDLE,
            DevStatus.WARN,
            DevStatus.START,
            DevStatus.BUSY,
            DevStatus.STOP,
            DevStatus.DIAGNOSTIC_ERROR,
        }
        return self.change_status(error_stati, DevStatus.RESET)

    def list_tables(self):
        """Return a list of the names of the device's :ref:`tables
        <pils:table-descriptor>`.

        Table objects are accessible as `self.tables[name]`.

        If the device supports no tables, an empty list is returned.
        """
        return list(self.tables)

    # to implement:

    def _init(self):
        raise NotImplementedError

    def read_value_raw(self):
        """Read the main value of the device.

        The type of the returned value differs depending on the device type:

        * int for discrete and enum devices
        * float for analog devices
        * list of float for vector devices
        """
        raise NotImplementedError

    def read_value(self):
        """Read the main value of the device like `read_value_raw`,
        but interpret and stringify values for enum types.
        """
        value = self.read_value_raw()
        if self._enum_r:
            value = str(self._enum_r.get(value, value))
        return value

    def read_target_raw(self):
        """Read the target of the device.

        The returned value is of the same type as `read_value_raw`.

        If the device is read-only and has no target, `.ApiError` is raised.
        """
        raise ApiError('reading target of a read-only device')

    def read_target(self):
        """Read the target value of the device like `read_target_raw`,
        but interpret and stringify values for enum types.
        """
        value = self.read_target_raw()
        if self._enum_w_rev:
            value = str(self._enum_w_rev.get(value, value))
        return value

    def change_target_raw(self, value):
        """Change the target of the device.

        This will atomically write the new target (whose type must be the same
        as returned by `read_value`) and set the state to `START`.

        If the device is read-only and has no target, `.ApiError` is raised.
        """
        raise ApiError('writing target of a read-only device')

    def change_target(self, value):
        """Like `change_target_raw`, but accepts string values for enum types
        and checks value limits.
        """
        if self._enum_w and isinstance(value, str):
            if value not in self._enum_w:
                raise ApiError(f'{value} is not one of '
                               f'{", ".join(self._enum_w)}')
            value = self._enum_w[value]
        elif (value < self.value_info.min_value or
              value > self.value_info.max_value):
            raise ApiError(f'{value} is not within device limits '
                           f'{self.value_info.min_value}..'
                           f'{self.value_info.max_value}')
        self.change_target_raw(value)

    def list_params(self):
        """Return a list of the names of the device's :ref:`parameters
        <pils:parameters-functions>`.

        If the device supports no parameters, an empty list is returned.
        """
        return []

    def get_param_raw(self, name):
        """Read the given parameter.

        The value type is int or float, enums are not translated.

        The return value is a tuple of ``(param_cmd, value)`` where
        ``param_cmd`` represents the reply of the parameter state machine,
        see `.ParamCMDs`.

        If the device supports no parameters, `.ApiError` is raised.
        """
        raise ApiError('reading parameter of a device without params')

    def get_param(self, name):
        """Read the given parameter.

        The type of the parameter depends on the parameter, it can be int,
        float, or string (enum).

        The reply from the parameter state machine is translated; if setting
        was not successful, `.ApiError` or `.CommError` is raised.

        If the device supports no parameters, `.ApiError` is raised.
        """
        cmd, value = self.get_param_raw(name)
        if cmd == ParamCMDs.ERR_NO_IDX:
            raise ApiError(f'parameter {name!r} does not exist')
        elif cmd == ParamCMDs.ERR_RO:
            raise ApiError(f'parameter {name!r} is read-only')
        elif cmd == ParamCMDs.ERR_RETRY:
            raise CommError('parameter interface is busy, retry later')
        value_info = self.get_param_valueinfo(name)
        if value_info.enum_r:
            value = str(value_info.enum_r.get(value, value))
        return value

    def get_param_valueinfo(self, name):
        """Return the given parameter's type info.

        If the device supports no parameters, `.ApiError` is raised.
        """
        raise ApiError('reading parameter type of a device without params')

    def set_param_raw(self, name, value):
        """Set the given parameter.

        The type of *value* must be int or float, enums are not translated.

        The return value is a tuple of ``(param_cmd, value)`` where
        ``param_cmd`` represents the reply of the parameter state machine,
        see `.ParamCMDs`, and ``value`` is the read-back value of the
        parameter (which might differ from the written value due to clamping,
        rounding or other correction the PLC makes).

        If the device supports no parameters, `.ApiError` is raised.
        """
        raise ApiError('writing parameter of a device without params')

    def set_param(self, name, value):
        """Set the given parameter.

        The type of *value* depends on the parameter, it must be int, float, or
        string (enum).  Value limits are checked.

        The reply from the parameter state machine is translated; if setting
        was not successful, `.ApiError` or `.CommError` is raised.  Else,
        the readback value is returned.

        If the device supports no parameters, `.ApiError` is raised.
        """
        value_info = self.get_param_valueinfo(name)
        if value_info.enum_w and isinstance(value, str):
            if value not in value_info.enum_w:
                raise ApiError(f'{value} is not one of {name!r} enum values '
                               f'{", ".join(self._enum_w)}')
            value = value_info.enum_w[value]
        elif (value < value_info.min_value or
              value > value_info.max_value):
            raise ApiError(f'{value} is not within {name!r} limits '
                           f'{value_info.min_value}..'
                           f'{value_info.max_value}')
        cmd, value = self.set_param_raw(name, value)
        if cmd == ParamCMDs.ERR_NO_IDX:
            raise ApiError(f'parameter {name} does not exist')
        elif cmd == ParamCMDs.ERR_RO:
            raise ApiError(f'parameter {name} is read-only')
        elif cmd == ParamCMDs.ERR_RETRY:
            raise CommError('parameter interface is busy, retry later')
        return value

    def list_funcs(self):
        """Return a list of the names of the device's `special functions
        <pils:parameters-functions>`.

        If the device supports no special functions, an empty list is returned.
        """
        return []

    def exec_func(self, name, value=None):
        """Execute the given special function with an argument.

        The return value is a tuple of ``(param_cmd, value)`` where
        ``param_cmd`` represents the reply of the parameter state machine,
        see `.ParamCMDs`, and ``value`` is the return value of the function.

        If the device supports no special functions, `.ApiError` is raised.
        """
        raise ApiError('executing function of a device without functions')


class DiscreteDevice(Device):
    """Base class used for discrete devices."""

    def _init(self):
        pass

    def read_value_raw(self):
        return self.io.read_fmt(self.addr, self.value_fmt)[0]

    def read_target_raw(self):
        if self.target_addr is None:
            raise ApiError('reading target of a read-only device')
        return self.io.read_fmt(self.target_addr, self.value_fmt)[0]

    def change_target_raw(self, value):
        if self.target_addr is None:
            raise ApiError('writing target of a read-only device')
        if self.status_addr is None:
            self.io.write_fmt(self.target_addr, self.value_fmt, value)
        elif self.value_size == 2:
            if self.status_size == 2:
                self.io.write_fmt(
                    self.target_addr, self.value_fmt + 'H', value, START16
                )
            elif self.status_size == 4:
                self.io.write_fmt(
                    self.target_addr, self.value_fmt + 'I', value, START32
                )
        else:
            self.io.write_fmt(self.target_addr, self.value_fmt + 'I',
                              value, START32)


class SimpleDiscreteIn(DiscreteDevice):
    """Class for :ref:`pils:dev-simplediscreteinput`,
    :ref:`pils:dev-simplediscrete32input`,
    :ref:`pils:dev-simplediscrete64input`."""


class SimpleDiscreteOut(DiscreteDevice):
    """Class for :ref:`pils:dev-simplediscreteoutput`,
    :ref:`pils:dev-simplediscrete32output`,
    :ref:`pils:dev-simplediscrete64output`."""


class DiscreteIn(DiscreteDevice):
    """Class for :ref:`pils:dev-discreteinput`,
    :ref:`pils:dev-discrete32input`, :ref:`pils:dev-discrete64input`."""


class DiscreteOut(DiscreteDevice):
    """Class for :ref:`pils:dev-discreteoutput`,
    :ref:`pils:dev-discrete32output`, :ref:`pils:dev-discrete64output`."""


class Keyword(DiscreteDevice):
    """Class for :ref:`pils:dev-keyword`, :ref:`pils:dev-keyword32`,
    :ref:`pils:dev-keyword64`."""

    def _init(self):
        self.target_addr = self.addr


class StatusWord(Keyword):
    """Class for :ref:`pils:dev-statusword`, :ref:`pils:dev-extstatusword`."""

    def _init(self):
        Keyword._init(self)
        self.status_addr = self.addr


class AnalogDevice(Device):
    """Base class for analog valued devices (except vector devices)."""

    def _init(self):
        pass

    def read_value_raw(self):
        if self.value_size == 4:
            return self.io.read_f32(self.addr)
        elif self.value_size == 8:
            return self.io.read_f64(self.addr)
        raise SpecError('invalid value_size')

    def read_target_raw(self):
        if self.target_addr is None:
            raise ApiError('reading target of a read-only device')
        if self.value_size == 4:
            return self.io.read_f32(self.target_addr)
        elif self.value_size == 8:
            return self.io.read_f64(self.target_addr)
        raise SpecError('invalid value_size')

    def change_target_raw(self, value):
        if self.target_addr is None:
            raise ApiError('writing target of a read-only device')
        if self.status_addr is None:
            if self.value_size == 4:
                self.io.write_f32(self.target_addr, value)
            elif self.value_size == 8:
                self.io.write_f64(self.target_addr, value)
        elif self.value_size == 4:
            if self.status_size == 2:
                self.io.write_f32_u16(self.target_addr, value, START16)
            else:
                self.io.write_f32_u32(self.target_addr, value, START32)
        elif self.value_size == 8:
            self.io.write_fmt(self.target_addr, 'dI', value, START32)


class SimpleAnalogIn(AnalogDevice):
    """Class for :ref:`pils:dev-simpleanaloginput`,
    :ref:`pils:dev-simpleanalog64input`."""


class SimpleAnalogOut(AnalogDevice):
    """Class for :ref:`pils:dev-simpleanalogoutput`,
    :ref:`pils:dev-simpleanalog64output`."""


class AnalogIn(AnalogDevice):
    """Class for :ref:`pils:dev-analoginput`, :ref:`pils:dev-analog64input`."""


class AnalogOut(AnalogDevice):
    """Class for :ref:`pils:dev-analogoutput`,
    :ref:`pils:dev-analog64output`."""


class RealValue(AnalogDevice):
    """Class for :ref:`pils:dev-realvalue`, :ref:`pils:dev-realvalue64`."""

    def _init(self):
        self.target_addr = self.addr


class FlatParams:
    def _init(self):
        if self.num_params != len(self._params):
            raise SpecError('mismatch between parameter count between typecode '
                            'and parameter indices from indexer '
                            f'({self.num_params}/{len(self._params)})')
        for pn, param in self._params.items():
            if param['idx'] >= self.num_params:
                raise SpecError(f'parameter {pn} has a too big index (must '
                                f'be < {self.num_param})')
        # map parameter name to (addr, typeinfo)
        param_map = {}
        for pn, par in self._params.items():
            idx = par['idx']
            val_info = get_valueinfo(par, self.value_size)
            addr = self.param_addr + idx * self.value_size
            param_map[pn] = (addr, val_info)
            param_map[idx] = (addr, val_info)
        self._param_map = param_map

    def list_params(self):
        return list(self._params)

    def get_param_valueinfo(self, name):
        return self._param_map[name][1]

    def get_param_raw(self, name):
        addr, val_info = self._param_map.get(name, (None,) * 2)
        if addr:
            return ParamCMDs.DONE, self.io.read_fmt(addr, val_info.fmt)[0]
        return ParamCMDs.ERR_NO_IDX, None

    def set_param_raw(self, name, value):
        addr, val_info = self._param_map.get(name, (None,) * 2)
        if not addr:
            return ParamCMDs.ERR_NO_IDX, None
        if val_info.readonly:
            return ParamCMDs.ERR_RO, None
        self.io.write_fmt(addr, val_info.fmt, value or 0)
        return ParamCMDs.DONE, self.io.read_fmt(addr, val_info.fmt)[0]


class FlatIn(FlatParams, AnalogDevice):
    """Class for :ref:`pils:dev-flatinput`, :ref:`pils:dev-flat64input`."""


class FlatOut(FlatParams, AnalogDevice):
    """Class for :ref:`pils:dev-flatoutput`, :ref:`pils:dev-flat64output`."""


class ParamInterface:
    param_timeout = 1

    def _init(self):
        self.param_sm = ParamControl()
        func_map = {}
        param_map = {}
        for pn, func in self._funcs.items():
            idx = func['idx']
            arg_info = res_info = None
            if func['argument']:
                arg_info = get_valueinfo(func['argument'], self.value_size)
            if func['result']:
                res_info = get_valueinfo(func['result'], self.value_size)
            func_map[idx] = (idx, arg_info, res_info)
            func_map[pn] = (idx, func['description'], arg_info, res_info)
        for pn, par in self._params.items():
            idx = par['idx']
            val_info = get_valueinfo(par, self.value_size)
            param_map[idx] = (idx, val_info)
            param_map[pn] = (idx, val_info)
        self._param_map = param_map
        self._func_map = func_map

    def get_param_index(self, name):
        return self._param_map[name][0]

    def get_func_index(self, name):
        return self._func_map[name][0]

    def get_param_valueinfo(self, name):
        return self._param_map[name][1]

    def get_func_valueinfos(self, name):
        return self._func_map[name][1:4]

    def get_pctrl(self, fmt=None, cached=True):
        pctrl, self.param_value = self.io.read_pctrl(self.pctrl_addr, fmt, cached)
        self.param_sm(pctrl)
        return self.param_sm, self.param_value

    def set_pctrl(self, cmd=None, idx=None, value=None, fmt=None):
        if None in (value, fmt):
            value = fmt = None
        if cmd is None and idx is None:
            pctrl = None
        else:
            if cmd is None or idx is None:
                # aarg! need read-modify-write cycle
                pctrl = self.io.read_pctrl(self.pctrl_addr, cached=False)[0]
                self.param_sm(pctrl)
            if cmd is not None:
                self.param_sm.CMD = cmd
            if idx is not None:
                self.param_sm.IDX = idx
            pctrl = int(self.param_sm)
        self.io.write_pctrl(self.pctrl_addr, fmt, pctrl, value)

    def wait_sm_available(self, fmt=None, cached=False):
        timesout = time.monotonic() + self.param_timeout
        while time.monotonic() <= timesout:
            self.get_pctrl(fmt, cached)
            if self.param_sm.available:
                return True
            # wait for cache update, do not burn CPU cycles...
            if cached:
                self.io.cache.updated.clear()
                self.io.cache.updated.wait(self.io.cache.cycle)
        return False

    def _get_param(self, idx, fmt):
        # XXX? With lock?
        if self.wait_sm_available():
            for _ in range(10):
                self.set_pctrl(ParamCMDs.DO_READ, idx)
                if self.wait_sm_available(fmt) and self.param_sm.IDX == idx:
                    return self.param_sm.CMD, self.param_value
        return ParamCMDs.ERR_RETRY, None

    def _set_param(self, idx, fmt, value):
        # XXX? With lock?
        if self.wait_sm_available():
            for _ in range(10):
                self.set_pctrl(ParamCMDs.DO_WRITE, idx, value, fmt)
                if self.wait_sm_available(fmt) and self.param_sm.IDX == idx:
                    return self.param_sm.CMD, self.param_value
        return ParamCMDs.ERR_RETRY, None

    def get_param_raw(self, name):
        if name not in self._params:
            return ParamCMDs.ERR_NO_IDX, None
        idx, val_info = self._param_map[name]
        return self._get_param(idx, val_info.fmt)

    def set_param_raw(self, name, value):
        if name not in self._params:
            return ParamCMDs.ERR_NO_IDX, None
        idx, val_info = self._param_map[name]
        if val_info.readonly:
            return ParamCMDs.ERR_RO, None
        self._set_param(idx, val_info.fmt, value)
        return self._get_param(idx, fmt=val_info.fmt)

    def list_params(self):
        return list(self._params)

    def list_funcs(self):
        return list(self._funcs)

    def exec_func(self, name, value=None):
        if name not in self._funcs:
            return ParamCMDs.ERR_NO_IDX, None
        idx, _, arg_info, res_info = self._func_map[name]
        # XXX? With lock?
        if self.wait_sm_available():
            if arg_info:
                self.set_pctrl(ParamCMDs.DO_WRITE, idx, value or 0,
                               fmt=arg_info.fmt)
            else:
                self.set_pctrl(ParamCMDs.DO_WRITE, idx)
            # now wait until setting parameter is finished and return
            # read-back-value
            self.wait_sm_available(res_info.fmt if res_info else None)
            return self.param_sm.CMD, self.param_value
        return ParamCMDs.ERR_RETRY, None


class ParamIn(ParamInterface, AnalogDevice):
    """Class for :ref:`pils:dev-paraminput`, :ref:`pils:dev-param64input`."""


class ParamOut(ParamInterface, AnalogDevice):
    """Class for :ref:`pils:dev-paramoutput`, :ref:`pils:dev-param64output`."""


class VectorDevice(Device):
    """Base class for Vector devices."""

    def _init(self):
        pass

    def read_value_raw(self):
        if self.value_size == 4:
            return self.io.read_f32s(self.addr, self.num_values)
        elif self.value_size == 8:
            return self.io.read_f64s(self.addr, self.num_values)
        raise SpecError('invalid value_size')

    def read_value(self):
        values = self.read_value_raw()
        if self._enum_r:
            values = [str(self._enum_r.get(value, value)) for value in values]
        return values


class VectorIn(ParamInterface, VectorDevice):
    """Class for :ref:`pils:dev-vectorinput`, :ref:`pils:dev-vector64input`."""


class VectorOut(ParamInterface, VectorDevice):
    """Class for :ref:`pils:dev-vectoroutput`,
    :ref:`pils:dev-vector64output`."""

    def read_target_raw(self):
        if self.value_size == 4:
            return self.io.read_f32s(self.target_addr, self.num_values)
        elif self.value_size == 8:
            return self.io.read_f64s(self.target_addr, self.num_values)
        raise SpecError('invalid value_size')

    def read_target(self):
        values = self.read_target_raw()
        if self._enum_w_rev:
            values = [str(self._enum_w_rev.get(value, value)) for value in values]
        return values

    def change_target_raw(self, value):
        if self.value_size == 4:
            if self.status_size == 2:
                self.io.write_f32s_u16(self.target_addr, value, START16)
            else:
                self.io.write_f32s_u32(self.target_addr, value, START32)
        elif self.value_size == 8:
            self.io.write_f64s_u32(self.target_addr, value, START32)

    def change_target(self, value):
        if self._enum_w:
            values = []
            for item in value:
                if isinstance(item, str):
                    if item not in self._enum_w:
                        raise ApiError(f'item {item} is not one of '
                                       f'{", ".join(self._enum_w)}')
                    item = self._enum_w[item]
                values.append(item)
        else:
            values = value
        self.change_target_raw(values)


class MessageIO(Device):
    """Class for :ref:`pils:dev-messageio`."""

    def _init(self):
        # self.addr is the mailbox register.  Data bytes start at next word.
        self.data_addr = self.addr + 2
        self.max_msg_size = self.total_size - 2
        self.msg_fmt = f'H{self.max_msg_size}s'

    def reset(self):
        # reset by setting the mailbox to initial state
        self.io.write_u16(self.addr, 0)

    def read_value_raw(self):
        raise ApiError('MessageIO has no value')

    def read_target_raw(self):
        raise ApiError('MessageIO has no target')

    def change_target_raw(self, value):
        raise ApiError('MessageIO has no target')

    def read_status(self):
        """Read the status information of the device.

        This is a tuple of ``(state, reason, aux, error_id)``.

        ``state`` is the 4-bit state (see `.DevStatus` for possible values).
        ``reason`` is the 4-bit reason code (see `.ReasonMap`).  ``aux``
        contains the up to 24 AUX bits.  ``error_id`` is a device-defined
        16-bit integer.

        For devices that do not have an error ID field, it is returned as zero.
        """
        mailbox = self.io.read_u16(self.addr)
        # sensible mapping of reset, idle, busy, busy, busy, busy, busy, error
        return {0: DevStatus.RESET, 1: DevStatus.IDLE, 7: DevStatus.ERROR}.get(
            mailbox >> 13, DevStatus.BUSY), 0, 0, 0

    def _get_mailbox_state(self, mailbox=None):
        """Check mailbox state and raise if that indicates an error."""
        if mailbox is None:
            mailbox = self.io.read_u16(self.addr)
        mailbox_state = mailbox >> 13
        if mailbox_state == 0:
            raise CommError('mailbox in Reset state! is the PLC running?')
        elif mailbox_state == 7:
            raise CommError('mailbox in Error state! try reset to clean up')
        return mailbox_state

    def _wait_for(self, state):
        """Wait for a certain state in the mailbox, with timeout."""
        for tri in range(10):
            mailbox_state = self._get_mailbox_state()
            if mailbox_state == state:
                break
            time.sleep(0.010 * tri)  # how to do better?
        else:
            mailbox_state = self._get_mailbox_state()
            if mailbox_state == state:
                return
            # is it worth decoding the mailbox states to strings?
            raise CommError(
                f'time-out while waiting for mailbox state {state} '
                f'(stuck at {mailbox_state}?)'
            )

    def _wait_for_not(self, state):
        """Wait for a certain state no longer in the mailbox, with timeout."""
        for tri in range(10):
            mailbox_state = self._get_mailbox_state()
            if mailbox_state != state:
                break
            time.sleep(0.010 * tri)  # how to do better?
        else:
            mailbox_state = self._get_mailbox_state()
            if mailbox_state != state:
                return
            # is it worth decoding the mailbox states to strings?
            raise CommError('time-out while waiting for mailbox leaving '
                            f'state {state}')

    def communicate(self, request):
        if isinstance(request, bytes):
            # split into approprate sized chunks
            requests = []
            while request:
                part, request = (
                    request[:self.max_msg_size],
                    request[self.max_msg_size:],
                )
                requests.append(part)
            result = self._communicate(requests)
            return b''.join(result)
        return self._communicate(request)

    def _communicate(self, requests):
        """Initiate a request-reply cycle with the PLC by sending the given
        request (a byte array) and return the response.
        """
        for r in requests:
            if len(r) > self.max_msg_size:
                raise ApiError('MessageIO: length of message parts can not '
                               f'exceed {self.max_msg_size} bytes, found '
                               f'{len(r)} bytes')

        # wait for initial state of mailbox
        self._wait_for(1)

        # need to send partial parts first
        while requests:
            r = requests.pop(0)
            if requests:
                # send partial part
                self.io.write_fmt(self.addr, self.msg_fmt, (2 << 13) + len(r), r)
                # wait for ACK
                self._wait_for(3)
            else:
                # send final part
                self.io.write_fmt(self.addr, self.msg_fmt, (4 << 13) + len(r), r)
                # wait until plc reacts
                self._wait_for_not(4)  # should go to 5 or 1 by PLC

        replies = []
        # collect partial replies first
        mailbox, data = self.io.read_fmt(self.addr, self.msg_fmt)
        mailbox_state = self._get_mailbox_state(mailbox)
        replies.append(data[: mailbox & 0x1FF])
        while mailbox_state == 5:
            # ack partial reply
            self.io.write_u16(self.addr, 6 << 13)
            self._wait_for_not(6)
            mailbox, data = self.io.read_fmt(self.addr, self.msg_fmt)
            mailbox_state = self._get_mailbox_state(mailbox)
            replies.append(data[: mailbox & 0x1FF])
        # final part already collected above...
        if mailbox_state == 1:  # last part transferred
            return replies
        else:
            raise CommError(f'bad mailbox state! expected 1, got {mailbox_state}')


# Note: has_target means that a target field is there, not that the device
# is considered read-only. For this use readonly....
Type = namedtuple(
    'Type',
    'devcls value_fmt num_values has_target readonly status_size num_params has_pctrl')


TYPECODE_MAP = {
    0x1201: Type(SimpleDiscreteIn,  'h', 1, False, True,  0, 0, False),
    0x1202: Type(SimpleDiscreteIn,  'i', 1, False, True,  0, 0, False),
    0x1204: Type(SimpleDiscreteIn,  'q', 1, False, True,  0, 0, False),
    0x1302: Type(SimpleAnalogIn,    'f', 1, False, True,  0, 0, False),
    0x1304: Type(SimpleAnalogIn,    'd', 1, False, True,  0, 0, False),
    0x1401: Type(Keyword,           'H', 1, False, False, 0, 0, False),
    0x1402: Type(Keyword,           'I', 1, False, False, 0, 0, False),
    0x1404: Type(Keyword,           'Q', 1, False, False, 0, 0, False),
    0x1502: Type(RealValue,         'f', 1, False, False, 0, 0, False),
    0x1504: Type(RealValue,         'd', 1, False, False, 0, 0, False),
    0x1602: Type(SimpleDiscreteOut, 'h', 1, True,  False, 0, 0, False),
    0x1604: Type(SimpleDiscreteOut, 'i', 1, True,  False, 0, 0, False),
    0x1608: Type(SimpleDiscreteOut, 'q', 1, True,  False, 0, 0, False),
    0x1704: Type(SimpleAnalogOut,   'f', 1, True,  False, 0, 0, False),
    0x1708: Type(SimpleAnalogOut,   'd', 1, True,  False, 0, 0, False),
    # changeable via start/Stop/reset/...
    0x1801: Type(StatusWord,        'H', 1, False, True,  2, 0, False),
    # changeable via start/Stop/reset/...
    0x1802: Type(StatusWord,        'I', 1, False, True,  4, 0, False),
    0x1a02: Type(DiscreteIn,        'h', 1, False, True,  2, 0, False),
    0x1a04: Type(DiscreteIn,        'i', 1, False, True,  4, 0, False),
    0x1a08: Type(DiscreteIn,        'q', 1, False, True,  6, 0, False),
    0x1b03: Type(AnalogIn,          'f', 1, False, True,  2, 0, False),
    0x1b04: Type(AnalogIn,          'f', 1, False, True,  4, 0, False),
    0x1b08: Type(AnalogIn,          'd', 1, False, True,  6, 0, False),
    0x1e03: Type(DiscreteOut,       'h', 1, True,  False, 2, 0, False),
    0x1e04: Type(DiscreteOut,       'h', 1, True,  False, 4, 0, False),
    0x1e06: Type(DiscreteOut,       'i', 1, True,  False, 4, 0, False),
    0x1e0c: Type(DiscreteOut,       'q', 1, True,  False, 6, 0, False),
    0x1f05: Type(AnalogOut,         'f', 1, True,  False, 2, 0, False),
    0x1f06: Type(AnalogOut,         'f', 1, True,  False, 4, 0, False),
    0x1f0c: Type(AnalogOut,         'd', 1, True,  False, 6, 0, False),

    0x4006: Type(ParamIn,           'f', 1, False, True,  2, 1, True),
    0x4008: Type(ParamIn,           'f', 1, False, True,  6, 1, True),
    0x400c: Type(ParamIn,           'd', 1, False, True,  6, 1, True),
    0x5008: Type(ParamOut,          'f', 1, True,  False, 2, 1, True),
    0x500a: Type(ParamOut,          'f', 1, True,  False, 6, 1, True),
    0x5010: Type(ParamOut,          'd', 1, True,  False, 6, 1, True),
}

for n in range(16):
    lfin32 =  0x2000 | (n << 8) | ( 6 + 2 * n)
    fin32 =   0x6000 | (n << 8) | ( 6 + 2 * n)
    fin64 =   0x2000 | (n << 8) | (12 + 4 * n)
    lfout32 = 0x3000 | (n << 8) | ( 8 + 2 * n)
    fout32 =  0x7000 | (n << 8) | ( 8 + 2 * n)
    fout64 =  0x3000 | (n << 8) | (16 + 4 * n)
    TYPECODE_MAP[lfin32]  = Type(FlatIn, 'f', 1, False, True, 2, n + 1, False)
    TYPECODE_MAP[fin32]   = Type(FlatIn, 'f', 1, False, True, 4, n + 1, False)
    TYPECODE_MAP[fin64]   = Type(FlatIn, 'd', 1, False, True, 6, n + 1, False)
    TYPECODE_MAP[lfout32] = Type(FlatOut, 'f', 1, True, False, 2, n + 1, False)
    TYPECODE_MAP[fout32]  = Type(FlatOut, 'f', 1, True, False, 4, n + 1, False)
    TYPECODE_MAP[fout64]  = Type(FlatOut, 'd', 1, True, False, 6, n + 1, False)

    if n == 0:
        continue

    vin32 =  0x4000 | (n << 8) | ( 6 + 2 * n)
    vin64 =  0x4000 | (n << 8) | (12 + 4 * n)
    vout32 = 0x5000 | (n << 8) | ( 8 + 4 * n)
    vout64 = 0x5000 | (n << 8) | (16 + 8 * n)
    TYPECODE_MAP[vin32]      = Type(VectorIn,  'f', n + 1, False, True, 2, 1, True)
    TYPECODE_MAP[vin32 + 2]  = Type(VectorIn,  'f', n + 1, False, True, 6, 1, True)
    TYPECODE_MAP[vin64]      = Type(VectorIn,  'd', n + 1, False, True, 6, 1, True)
    TYPECODE_MAP[vout32]     = Type(VectorOut, 'f', n + 1, True, False, 2, 1, True)
    TYPECODE_MAP[vout32 + 2] = Type(VectorOut, 'f', n + 1, True, False, 6, 1, True)
    TYPECODE_MAP[vout64]     = Type(VectorOut, 'd', n + 1, True, False, 6, 1, True)

for n in range(4, 253, 2):
    msgio = 0x0500 | (n >> 1)
    TYPECODE_MAP[msgio] = Type(MessageIO, f'{n-2}s', 1, False, True, 0, 0, False)


def typecode_description(typecode):
    typeinfo = TYPECODE_MAP[typecode]
    name = typeinfo.devcls.__name__
    valuesize = calcsize(typeinfo.value_fmt)
    if typeinfo.num_params > 0 and not typeinfo.has_pctrl:
        name += f'/{typeinfo.num_params}'
    if typeinfo.num_values > 1:
        name += f'/{typeinfo.num_values}'
    if (typecode in (0x1B03, 0x1F05)) or \
       (typeinfo.status_size == 2 and typeinfo.num_params > 0):
        name += ' (32 bit, legacy)'
    elif typecode == 0x1E03:
        name += ' (16 bit, legacy)'
    elif (typecode) >> 8 == 5:
        name += f' ({valuesize} bytes)'
    else:
        name += f' ({valuesize*8} bit)'
    return name

FMT_LIMITS = {
    'h' : (-(2**15), 2**15-1),
    'H' : (0, 2**16-1),
    'i' : (-(2**31), 2**31-1),
    'I' : (0, 2**32-1),
    'q' : (-(2**63), 2**63-1),
    'Q' : (0, 2**64-1),
    'f' : (-3.403823e38, 3.403823e38),
    'd' : (-1.7976931348623157e+308, 1.7976931348623157e+308),
}
