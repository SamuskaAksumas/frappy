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
#   Alexander Lenz <alexander.lenz@frm2.tum.de>
#   Enrico Faulhaber <enrico.faulhaber@frm2.tum.de>
#
# *****************************************************************************

# This is based upon the entangle-nicos integration
"""
This module contains the MLZ SECoP - TANGO integration.

Here we support devices which fulfill the official
MLZ TANGO interface for the respective device classes.
"""

import re
import sys
from time import sleep, time as currenttime
import threading

import PyTango
import numpy

from secop.lib import lazy_property, mkthread
from secop.protocol import status
from secop.datatypes import *
from secop.errors import SECoPServerError, ConfigError, ProgrammingError, CommunicationError, HardwareError
from secop.modules import PARAM, CMD, OVERRIDE, Device, Readable, Driveable


# Only export these classes for 'from secop_mlz import *'
__all__ = [
    'AnalogInput', 'Sensor',
    'AnalogOutput', 'Actuator', 'Motor',
    'TemperatureController', 'PowerSupply',
    'DigitalInput', 'NamedDigitalInput', 'PartialDigitalInput',
    'DigitalOutput', 'NamedDigitalOutput', 'PartialDigitalOutput',
    'StringIO',
]

EXC_MAPPING = {
    PyTango.CommunicationFailed: CommunicationError,
    PyTango.WrongNameSyntax: ConfigError,
    PyTango.DevFailed: HardwareError,
}

REASON_MAPPING = {
    'Entangle_ConfigurationError': ConfigError,
    'Entangle_WrongAPICall': ProgrammingError,
    'Entangle_CommunicationFailure': CommunicationError,
    'Entangle_InvalidValue': ValueError,
    'Entangle_ProgrammingError': ProgrammingError,
    'Entangle_HardwareFailure': HardwareError,
}

# Tango DevFailed reasons that should not cause a retry
FATAL_REASONS = set((
    'Entangle_ConfigurationError',
    'Entangle_UnrecognizedHardware',
    'Entangle_WrongAPICall',
    'Entangle_InvalidValue',
    'Entangle_NotSupported',
    'Entangle_ProgrammingError',
    'DB_DeviceNotDefined',
    'API_DeviceNotDefined',
    'API_CantConnectToDatabase',
    'API_TangoHostNotSet',
    'API_ServerNotRunning',
    'API_DeviceNotExported',
))


def describe_dev_error(exc):
    """Return a better description for a Tango exception.

    Most Tango exceptions are quite verbose and not suitable for user
    consumption.  Map the most common ones, that can also happen during normal
    operation, to a bit more friendly ones.
    """
    # general attributes
    reason = exc.reason.strip()
    fulldesc = reason + ': ' + exc.desc.strip()
    # reduce Python tracebacks
    if '\n' in exc.origin and 'File ' in exc.origin:
        origin = exc.origin.splitlines()[-2].strip()
    else:
        origin = exc.origin.strip()

    # we don't need origin info for Tango itself
    if origin.startswith(('DeviceProxy::', 'DeviceImpl::', 'Device_3Impl::',
                          'Device_4Impl::', 'Connection::', 'TangoMonitor::')):
        origin = None

    # now handle specific cases better
    if reason == 'API_AttrNotAllowed':
        m = re.search(r'to (read|write) attribute (\w+)', fulldesc)
        if m:
            if m.group(1) == 'read':
                fulldesc = 'reading %r not allowed in current state'
            else:
                fulldesc = 'writing %r not allowed in current state'
            fulldesc %= m.group(2)
    elif reason == 'API_CommandNotAllowed':
        m = re.search(r'Command (\w+) not allowed when the '
                      r'device is in (\w+) state', fulldesc)
        if m:
            fulldesc = 'executing %r not allowed in state %s' \
                % (m.group(1), m.group(2))
    elif reason == 'API_DeviceNotExported':
        m = re.search(r'Device ([\w/]+) is not', fulldesc)
        if m:
            fulldesc = 'Tango device %s is not exported, is the server ' \
                'running?' % m.group(1)
    elif reason == 'API_CorbaException':
        if 'TRANSIENT_CallTimedout' in fulldesc:
            fulldesc = 'Tango client-server call timed out'
        elif 'TRANSIENT_ConnectFailed' in fulldesc:
            fulldesc = 'connection to Tango server failed, is the server ' \
                'running?'
    elif reason == 'API_CantConnectToDevice':
        m = re.search(r'connect to device ([\w/]+)', fulldesc)
        if m:
            fulldesc = 'connection to Tango device %s failed, is the server ' \
                'running?' % m.group(1)
    elif reason == 'API_CommandTimedOut':
        if 'acquire serialization' in fulldesc:
            fulldesc = 'Tango call timed out waiting for lock on server'

    # append origin if wanted
    if origin:
        fulldesc += ' in %s' % origin
    return fulldesc


class PyTangoDevice(Device):
    """
    Basic PyTango device.

    The PyTangoDevice uses an internal PyTango.DeviceProxy but wraps command
    execution and attribute operations with logging and exception mapping.
    """

    PARAMS = {
        'comtries': PARAM('Maximum retries for communication',
                          datatype=IntRange(1, 100), default=3, readonly=False, group='communication'),
        'comdelay': PARAM('Delay between retries', datatype=FloatRange(0), unit='s', default=0.1,
                          readonly=False, group='communication'),

        'tangodevice': PARAM('Tango device name',
                             datatype=StringType(), readonly=True,
#                             export=True,   # for testing only
                             export=False,
                             ),
    }

    tango_status_mapping = {
        PyTango.DevState.ON: status.OK,
        PyTango.DevState.ALARM: status.WARN,
        PyTango.DevState.OFF: status.ERROR,
        PyTango.DevState.FAULT: status.ERROR,
        PyTango.DevState.MOVING: status.BUSY,
    }

    @lazy_property
    def _com_lock(self):
        return threading.Lock()

    def _com_retry(self, info, function, *args, **kwds):
        """Try communicating with the hardware/device.

        PARAMeter "info" is passed to _com_return and _com_raise methods that
        process the return value or exception raised after maximum tries.
        """
        tries = self.comtries
        with self._com_lock:
            while True:
                tries -= 1
                try:
                    result = function(*args, **kwds)
                    return self._com_return(result, info)
                except Exception as err:
                    if tries == 0:
                        self._com_raise(err, info)
                    else:
                        name = getattr(function, '__name__', 'communication')
                        self._com_warn(tries, name, err, info)
                    sleep(self.comdelay)

    def init(self):
        # Wrap PyTango client creation (so even for the ctor, logging and
        # exception mapping is enabled).
        self._createPyTangoDevice = self._applyGuardToFunc(
            self._createPyTangoDevice, 'constructor')
        super(PyTangoDevice, self).init()

    @lazy_property
    def _dev(self):
        return self._createPyTangoDevice(self.tangodevice)

    def _hw_wait(self):
        """Wait until hardware status is not BUSY."""
        while PyTangoDevice.doStatus(self, 0)[0] == status.BUSY:
            sleep(self._base_loop_delay)

    def _getProperty(self, name, dev=None):
        """
        Utility function for getting a property by name easily.
        """
        if dev is None:
            dev = self._dev
        # Entangle and later API
        if dev.command_query('GetProperties').in_type == PyTango.DevVoid:
            props = dev.GetProperties()
            return props[props.index(name) + 1] if name in props else None
        # old (pre-Entangle) API
        return dev.GetProperties([name, 'device'])[2]

    def _createPyTangoDevice(self, address):  # pylint: disable=E0202
        """
        Creates the PyTango DeviceProxy and wraps command execution and
        attribute operations with logging and exception mapping.
        """
        device = PyTango.DeviceProxy(address)
        # detect not running and not exported devices early, because that
        # otherwise would lead to attribute errors later
        try:
            device.State
        except AttributeError:
            raise CommunicationError(
                self, 'connection to Tango server failed, '
                'is the server running?')
        return self._applyGuardsToPyTangoDevice(device)

    def _applyGuardsToPyTangoDevice(self, dev):
        """
        Wraps command execution and attribute operations of the given
        device with logging and exception mapping.
        """
        dev.command_inout = self._applyGuardToFunc(dev.command_inout)
        dev.write_attribute = self._applyGuardToFunc(dev.write_attribute,
                                                     'attr_write')
        dev.read_attribute = self._applyGuardToFunc(dev.read_attribute,
                                                    'attr_read')
        dev.attribute_query = self._applyGuardToFunc(dev.attribute_query,
                                                     'attr_query')
        return dev

    def _applyGuardToFunc(self, func, category='cmd'):
        """
        Wrap given function with logging and exception mapping.
        """
        def wrap(*args, **kwds):
            # handle different types for better debug output
            if category == 'cmd':
                self.log.debug('[PyTango] command: %s%r', args[0], args[1:])
            elif category == 'attr_read':
                self.log.debug('[PyTango] read attribute: %s', args[0])
            elif category == 'attr_write':
                self.log.debug('[PyTango] write attribute: %s => %r',
                               args[0], args[1:])
            elif category == 'attr_query':
                self.log.debug('[PyTango] query attribute properties: %s',
                               args[0])
            elif category == 'constructor':
                self.log.debug('[PyTango] device creation: %s', args[0])
            elif category == 'internal':
                self.log.debug('[PyTango integration] internal: %s%r',
                               func.__name__, args)
            else:
                self.log.debug('[PyTango] call: %s%r', func.__name__, args)

            info = category + ' ' + args[0] if args else category
            return self._com_retry(info, func, *args, **kwds)

        # hide the wrapping
        wrap.__name__ = func.__name__

        return wrap

    def _com_return(self, result, info):
        """Process *result*, the return value of communication.

        Can raise an exception to initiate a retry.  Default is to return
        result unchanged.
        """
        # XXX: explicit check for loglevel to avoid expensive reprs
        if isinstance(result, PyTango.DeviceAttribute):
            the_repr = repr(result.value)[:300]
        else:
            # This line explicitly logs '=> None' for commands which
            # does not return a value. This indicates that the command
            # execution ended.
            the_repr = repr(result)[:300]
        self.log.debug('\t=> %s', the_repr)
        return result

    def _tango_exc_desc(self, err):
        exc = str(err)
        if err.args:
            exc = err.args[0]  # Can be str or DevError
            if isinstance(exc, PyTango.DevError):
                return describe_dev_error(exc)
        return exc

    def _tango_exc_reason(self, err):
        if err.args and isinstance(err.args[0], PyTango.DevError):
            return err.args[0].reason.strip()
        return ''

    def _com_warn(self, retries, name, err, info):
        """Gives the opportunity to warn the user on failed tries.

        Can also call _com_raise to abort early.
        """
        if self._tango_exc_reason(err) in FATAL_REASONS:
            self._com_raise(err, info)
        if retries == self.comtries - 1:
            self.log.warning('%s failed, retrying up to %d times: %s',
                             info, retries, self._tango_exc_desc(err))

    def _com_raise(self, err, info):
        """Process the exception raised either by communication or _com_return.

        Should raise a NICOS exception.  Default is to raise
        CommunicationError.
        """
        reason = self._tango_exc_reason(err)
        exclass = REASON_MAPPING.get(
            reason, EXC_MAPPING.get(type(err), CommunicationError))
        fulldesc = self._tango_exc_desc(err)
        self.log.debug('PyTango error: %s', fulldesc)
        raise exclass(self, fulldesc)

    def read_status(self, maxage=0):
        # Query status code and string
        tangoState = self._dev.State()
        tangoStatus = self._dev.Status()

        # Map status
        myState = self.tango_status_mapping.get(tangoState, status.UNKNOWN)

        return (myState, tangoStatus)

    def do_reset(self):
        self._dev.Reset()


class AnalogInput(PyTangoDevice, Readable):
    """
    The AnalogInput handles all devices only delivering an analogue value.
    """

    def init(self):
        super(AnalogInput, self).init()
        # query unit from tango and update value property
        attrInfo = self._dev.attribute_query('value')
        # prefer configured unit if nothing is set on the Tango device, else
        # update
        if attrInfo.unit != 'No unit':
            self.PARAMS['value'].unit = attrInfo.unit

    def read_value(self, maxage=0):
        return self._dev.value


class Sensor(AnalogInput):
    """
    The sensor interface describes all analog read only devices.

    The difference to AnalogInput is that the “value” attribute can be
    converted from the “raw value” to a physical value with an offset and a
    formula.
    """
    # note: we don't transport the formula to secop....
    #       we support the adjust method

    def do_setposition(self, value):
        self._dev.Adjust(value)


class AnalogOutput(PyTangoDevice, Driveable):
    """
    The AnalogOutput handles all devices which set an analogue value.

    The main application field is the output of any signal which may be
    considered as continously in a range. The values may have nearly any
    value between the limits. The compactness is limited by the resolution of
    the hardware.

    This class should be considered as a base class for motors, temperature
    controllers, ...
    """

    PARAMS = {
        'userlimits': PARAM(
            'User defined limits of device value',
            unit='main',
            datatype=TupleOf(
                FloatRange(),
                FloatRange()),
            default=(
                float('-Inf'),
                float('+Inf')),
            readonly=False,
            poll=10),
        'abslimits': PARAM(
            'Absolute limits of device value',
            unit='main',
            datatype=TupleOf(
                FloatRange(),
                FloatRange()),
        ),
        'precision': PARAM(
            'Precision of the device value (allowed deviation '
            'of stable values from target)',
            unit='main',
            datatype=FloatRange(1e-38),
            readonly=False,
        ),
        'window': PARAM(
            'Time window for checking stabilization if > 0',
            unit='s',
            default=60.0,
            datatype=FloatRange(
                0,
                900),
            readonly=False,
        ),
        'pollinterval': PARAM(
            '[min, max] sleeptime between polls',
            default=[
                0.5,
                5],
            readonly=False,
            datatype=TupleOf(
                FloatRange(
                    0,
                    20),
                FloatRange(
                    0.1,
                    120)),
        ),
    }
    OVERRIDES = {
        'value': OVERRIDE(poll=False),
    }

    def init(self):
        super(AnalogInput, self).init()
        # query unit from tango and update value property
        attrInfo = self._dev.attribute_query('value')
        # prefer configured unit if nothing is set on the Tango device, else
        # update
        if attrInfo.unit != 'No unit':
            self.PARAMS['value'].unit = attrInfo.unit

        # init history
        self._history = []  # will keep (timestamp, value) tuple
        mkthread(self._history_thread)

    def _history_thread(self):
        while True:
            # adaptive sleeping interval
            if self.status[0] == status.BUSY:
                sleep(min(self.pollinterval))
            else:
                sleep(min(max(self.pollinterval) / 2.,
                          max(self.window / 10., min(pollinterval))))
            try:
                self.read_value(0)  # also append to self._history
                # shorten history
                while len(self._history) > 2:
                    # if history would be too short, break
                    if self._history[-1][0] - \
                            self._history[1][0] < self.window:
                        break
                    # remove a stale point
                    self._history.pop(0)
            except Exception:
                pass

    def read_value(self, maxage=0):
        value = self._dev.value
        self._history.append((currenttime(), value))
        return value

    def read_target(self, maxage=0):
        attrObj = self._dev.read_attribute('value')
        return attrObj.w_value

    def _isAtTarget(self):
        if self.target is None:
            return True  # avoid bootstrapping problems
        # check subset of _history which is in window
        # also check if there is at least one value before window
        # to know we have enough datapoints
        hist = self._history[:]
        window_start = currenttime() - self.window
        hist_in_window = [v for (t, v) in hist if t >= window_start]
        stable = all(abs(v - self.target) <= self.precision
                     for v in hist_in_window)
        return 0 < len(hist_in_window) < len(hist) and stable

    @property
    def absmin(self):
        return self.abslimits[0]

    @property
    def absmax(self):
        return self.abslimits[1]

    def __getusermin(self):
        return self.userlimits[0]

    def __setusermin(self, value):
        self.userlimits = (value, self.userlimits[1])

    usermin = property(__getusermin, __setusermin)

    def __getusermax(self):
        return self.userlimits[1]

    def __setusermax(self, value):
        self.userlimits = (self.userlimits[0], value)

    usermax = property(__getusermax, __setusermax)

    del __getusermin, __setusermin, __getusermax, __setusermax

    def _checkLimits(self, limits):
        umin, umax = limits
        amin, amax = self.abslimits
        if umin > umax:
            raise ValueError(
                self, 'user minimum (%s) above the user '
                'maximum (%s)' % (umin, umax))
        if umin < amin - abs(amin * 1e-12):
            umin = amin
        if umax > amax + abs(amax * 1e-12):
            umax = amax
        return (umin, umax)

    def write_userlimits(self, value):
        return self._checkLimits(value)

    def do_start(self, value=FloatRange()):
        try:
            self._dev.value = value
        except HardwareError:
            # changing target value during movement is not allowed by the
            # Tango base class state machine. If we are moving, stop first.
            if self.read_status(0)[0] == status.BUSY:
                self.stop()
                self._hw_wait()
                self._dev.value = value
            else:
                raise

    def do_stop(self):
        self._dev.Stop()


class Actuator(AnalogOutput):
    """
    The actuator interface describes all analog devices which DO something in a
    defined way.

    The difference to AnalogOutput is that there is a speed attribute, and the
    value attribute is converted from the “raw value” with a formula and
    offset.
    """
    # for secop: support the speed and ramp parameters

    PARAMS = {
        'speed': PARAM(
            'The speed of changing the value',
            unit='main/s',
            readonly=False,
            datatype=FloatRange(0)),
        'ramp': PARAM(
            'The speed of changing the value',
            unit='main/min',
            readonly=False,
            datatype=FloatRange(0),
            poll=30),
    }

    def read_speed(self):
        return self._dev.speed

    def write_speed(self, value):
        self._dev.speed = value

    def read_ramp(self):
        return self.read_speed() * 60

    def write_ramp(self, value):
        self.write_speed(value / 60.)
        return self.speed * 60

    def do_setposition(self, value):
        self._dev.Adjust(value)


class Motor(Actuator):
    """
    This class implements a motor device (in a sense of a real motor
    (stepper motor, servo motor, ...)).

    It has the ability to move a real object from one place to another place.
    """

    PARAMS = {
        'refpos': PARAM(
            'Reference position',
            datatype=FloatRange(),
            unit='main'),
        'accel': PARAM(
            'Acceleration',
            datatype=FloatRange(),
            readonly=False,
            unit='main/s^2'),
        'decel': PARAM(
            'Deceleration',
            datatype=FloatRange(),
            readonly=False,
            unit='main/s^2'),
    }

    def read_refpos(self):
        return float(self._getProperty('refpos'))

    def read_accel(self):
        return self._dev.accel

    def write_accel(self, value):
        self._dev.accel = value

    def read_decel(self):
        return self._dev.decel

    def write_decel(self, value):
        self._dev.decel = value

    def do_reference(self):
        self._dev.Reference()
        return self.read_value()


class TemperatureController(Actuator):
    """
    A temperature control loop device.
    """

    PARAMS = {
        'p': PARAM('Proportional control PARAMeter', datatype=FloatRange(),
                   readonly=False, group='pid',
                   ),
        'i': PARAM('Integral control PARAMeter', datatype=FloatRange(),
                   readonly=False, group='pid',
                   ),
        'd': PARAM('Derivative control PARAMeter', datatype=FloatRange(),
                   readonly=False, group='pid',
                   ),
        'pid': PARAM('pid control PARAMeters', datatype=TupleOf(FloatRange(), FloatRange(), FloatRange()),
                     readonly=False, group='pid', poll=30,
                     ),
        'setpoint': PARAM('Current setpoint', datatype=FloatRange(), poll=1,
                          ),
        'heateroutput': PARAM('Heater output', datatype=FloatRange(), poll=1,
                              ),
        'ramp': PARAM('Temperature ramp', unit='main/min',
                      datatype=FloatRange(), readonly=False, poll=30),
    }

    OVERRIDES = {
        # We want this to be freely user-settable, and not produce a warning
        # on startup, so select a usually sensible default.
        'precision': OVERRIDE(default=0.1),
    }

    def read_ramp(self):
        return self._dev.ramp

    def write_ramp(self, value):
        self._dev.ramp = value
        return self._dev.ramp

    def read_p(self):
        return self._dev.p

    def write_p(self, value):
        self._dev.p = value

    def read_i(self):
        return self._dev.i

    def write_i(self, value):
        self._dev.i = value

    def read_d(self):
        return self._dev.d

    def write_d(self, value):
        self._dev.d = value

    def read_pid(self):
        self.read_p()
        self.read_i()
        self.read_d()
        return self.p, self.i, self.d

    def write_pid(self, value):
        self._dev.p = value[0]
        self._dev.i = value[1]
        self._dev.d = value[2]

    def read_setpoint(self):
        return self._dev.setpoint

    def read_heateroutput(self):
        return self._dev.heaterOutput


class PowerSupply(Actuator):
    """
    A power supply (voltage and current) device.
    """

    PARAMS = {
        'ramp': PARAM('Current/voltage ramp', unit='main/min',
                      datatype=FloatRange(), readonly=False, poll=30,),
        'voltage': PARAM('Actual voltage', unit='V',
                         datatype=FloatRange(), poll=5),
        'current': PARAM('Actual current', unit='A',
                         datatype=FloatRange(), poll=5),
    }

    def read_ramp(self):
        return self._dev.ramp

    def write_ramp(self, value):
        self._dev.ramp = value

    def read_voltage(self):
        return self._dev.voltage

    def read_current(self):
        return self._dev.current


class DigitalInput(PyTangoDevice, Readable):
    """
    A device reading a bitfield.
    """

    OVERRIDES = {
        'value': OVERRIDE(datatype=IntRange(0)),
    }

    def read_value(self, maxage=0):
        return self._dev.value


class NamedDigitalInput(DigitalInput):
    """
    A DigitalInput with numeric values mapped to names.
    """

    PARAMS = {
        'mapping': PARAM('A dictionary mapping state names to integers',
                         datatype=StringType(), export=False),  # XXX:!!!
    }

    def init(self):
        super(NamedDigitalInput, self).init()
        try:
            self.PARAMS['value'].datatype = EnumType(**eval(self.mapping))
        except Exception as e:
            raise ValueError('Illegal Value for mapping: %r' % e)

    def read_value(self, maxage=0):
        value = self._dev.value
        return value  # mapping is done by datatype upon export()


class PartialDigitalInput(NamedDigitalInput):
    """
    Base class for a TANGO DigitalInput with only a part of the full
    bit width accessed.
    """

    PARAMS = {
        'startbit': PARAM(
            'Number of the first bit',
            datatype=IntRange(0),
            default=0),
        'bitwidth': PARAM(
            'Number of bits',
            datatype=IntRange(0),
            default=1),
    }

    def init(self):
        super(PartialDigitalInput, self).init()
        self._mask = (1 << self.bitwidth) - 1
        #self.PARAMS['value'].datatype = IntRange(0, self._mask)

    def read_value(self, maxage=0):
        raw_value = self._dev.value
        value = (raw_value >> self.startbit) & self._mask
        return value  # mapping is done by datatype upon export()


class DigitalOutput(PyTangoDevice, Driveable):
    """
    A devices that can set and read a digital value corresponding to a
    bitfield.
    """

    OVERRIDES = {
        'value': OVERRIDE(datatype=IntRange(0)),
        'target': OVERRIDE(datatype=IntRange(0)),
    }

    def read_value(self, maxage=0):
        return self._dev.value  # mapping is done by datatype upon export()

    def write_target(self, value):
        self._dev.value = value
        self.read_value()

    def read_target(self, maxage=0):
        attrObj = self._dev.read_attribute('value')
        return attrObj.w_value


class NamedDigitalOutput(DigitalOutput):
    """
    A DigitalOutput with numeric values mapped to names.
    """

    PARAMS = {
        'mapping': PARAM('A dictionary mapping state names to integers',
                         datatype=StringType(), export=False),  # XXX: !!!
    }

    def init(self):
        super(NamedDigitalOutput, self).init()
        try:  # XXX: !!!
            self.PARAMS['value'].datatype = EnumType(**eval(self.mapping))
        except Exception as e:
            raise ValueError('Illegal Value for mapping: %r' % e)


class PartialDigitalOutput(NamedDigitalOutput):
    """
    Base class for a TANGO DigitalOutput with only a part of the full
    bit width accessed.
    """

    PARAMS = {
        'startbit': PARAM(
            'Number of the first bit',
            datatype=IntRange(0),
            default=0),
        'bitwidth': PARAM(
            'Number of bits',
            datatype=IntRange(0),
            default=1),
    }

    def init(self, mode):
        super(PartialDigitalOutput, self).init()
        self._mask = (1 << self.bitwidth) - 1
        #self.PARAMS['value'].datatype = IntRange(0, self._mask)
        #self.PARAMS['target'].datatype = IntRange(0, self._mask)

    def read_value(self, maxage=0):
        raw_value = self._dev.value
        value = (raw_value >> self.startbit) & self._mask
        return value  # mapping is done by datatype upon export()

    def write_target(self, target):
        curvalue = self._dev.value
        newvalue = (curvalue & ~(self._mask << self.startbit)) | \
                   (target << self.startbit)
        self._dev.value = newvalue
        self.read_value()


class StringIO(PyTangoDevice, Device):
    """
    StringIO abstracts communication over a hardware bus that sends and
    receives strings.
    """

    PARAMS = {
        'bustimeout': PARAM(
            'Communication timeout',
            datatype=FloatRange(),
            readonly=False,
            unit='s',
            group='communication'),
        'endofline': PARAM(
            'End of line',
            datatype=StringType(),
            readonly=False,
            group='communication'),
        'startofline': PARAM(
            'Start of line',
            datatype=StringType(),
            readonly=False,
            group='communication'),
    }

    def read_bustimeout(self):
        return self._dev.communicationTimeout

    def write_bustimeout(self, value):
        self._dev.communicationTimeout = value

    def read_endofline(self):
        return self._dev.endOfLine

    def write_endofline(self, value):
        self._dev.endOfLine = value

    def read_startofline(self):
        return self._dev.startOfLine

    def write_startofline(self, value):
        self._dev.startOfLine = value

    CMDS = {
        'communicate': CMD(
            'Send a string and return the reply',
            arguments=[
                StringType()],
            result=StringType()),
        'flush': CMD(
            'Flush output buffer',
            arguments=[],
            result=None),
        'read': CMD(
            'read some characters from input buffer',
            arguments=[
                IntRange()],
            result=StringType()),
        'write': CMD(
            'write some chars to output',
            arguments=[
                StringType()],
            result=None),
        'readLine': CMD(
            'Read sol - a whole line - eol',
            arguments=[],
            result=StringType()),
        'writeLine': CMD(
            'write sol + a whole line + eol',
            arguments=[
                StringType()],
            result=None),
        'availablechars': CMD(
            'return number of chars in input buffer',
            arguments=[],
            result=IntRange(0)),
        'availablelines': CMD(
            'return number of lines in input buffer',
            arguments=[],
            result=IntRange(0)),
        'multicommunicate': CMD(
            'perform a sequence of communications',
            arguments=[
                ArrayOf(
                    TupleOf(
                        StringType(),
                        IntRange()))],
            result=ArrayOf(
                StringType())),
    }

    def do_communicate(self, value=StringType()):
        return self._dev.Communicate(value)

    def do_flush(self):
        self._dev.Flush()

    def do_read(self, value):
        return self._dev.Read(value)

    def do_write(self, value):
        return self._dev.Write(value)

    def do_readLine(self):
        return self._dev.ReadLine()

    def do_writeLine(self, value):
        return self._dev.WriteLine(value)

    def do_multiCommunicate(self, value):
        return self._dev.MultiCommunicate(value)

    def do_availablechars(self):
        return self._dev.availableChars

    def do_availablelines(self):
        return self._dev.availableLines
