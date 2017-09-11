#!/usr/bin/env python
#  -*- coding: utf-8 -*-
# *****************************************************************************
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
#   Enrico Faulhaber <enrico.faulhaber@frm2.tum.de>
# *****************************************************************************
"""testing devices"""

import time
import random
import threading

from secop.modules import Readable, Drivable, PARAM
from secop.datatypes import EnumType, FloatRange, IntRange, ArrayOf, StringType, TupleOf, StructOf, BoolType
from secop.protocol import status


class Switch(Drivable):
    """switch it on or off....
    """
    PARAMS = {
        'value': PARAM('current state (on or off)',
                       datatype=EnumType(on=1, off=0), default=0,
                       ),
        'target': PARAM('wanted state (on or off)',
                        datatype=EnumType(on=1, off=0), default=0,
                        readonly=False,
                        ),
        'switch_on_time': PARAM('seconds to wait after activating the switch',
                                datatype=FloatRange(0, 60), unit='s',
                                default=10, export=False,
                                ),
        'switch_off_time': PARAM('cool-down time in seconds',
                                 datatype=FloatRange(0, 60), unit='s',
                                 default=10, export=False,
                                 ),
    }

    def init(self):
        self._started = 0

    def read_value(self, maxage=0):
        # could ask HW
        # we just return the value of the target here.
        self._update()
        return self.value

    def read_target(self, maxage=0):
        # could ask HW
        return self.target

    def write_target(self, value):
        # could tell HW
        pass
        # note: setting self.target to the new value is done after this....
        # note: we may also return the read-back value from the hw here

    def read_status(self, maxage=0):
        self.log.info("read status")
        info = self._update()
        if self.target == self.value:
            return status.OK, ''
        return status.BUSY, info

    def _update(self):
        started = self.PARAMS['target'].timestamp
        info = ''
        if self.target > self.value:
            info = 'waiting for ON'
            if time.time() > started + self.switch_on_time:
                info = 'is switched ON'
                self.value = self.target
        elif self.target < self.value:
            info = 'waiting for OFF'
            if time.time() > started + self.switch_off_time:
                info = 'is switched OFF'
                self.value = self.target
        if info:
            self.log.debug(info)
        return info


class MagneticField(Drivable):
    """a liquid magnet
    """
    PARAMS = {
        'value': PARAM('current field in T',
                       unit='T', datatype=FloatRange(-15, 15), default=0,
                       ),
        'target': PARAM('target field in T',
                        unit='T', datatype=FloatRange(-15, 15), default=0,
                        readonly=False,
                        ),
        'ramp': PARAM('ramping speed',
                      unit='T/min', datatype=FloatRange(0, 1), default=0.1,
                      readonly=False,
                      ),
        'mode': PARAM('what to do after changing field',
                      default=1, datatype=EnumType(persistent=1, hold=0),
                      readonly=False,
                      ),
        'heatswitch': PARAM('name of heat switch device',
                            datatype=StringType(), export=False,
                            ),
    }

    def init(self):
        self._state = 'idle'
        self._heatswitch = self.DISPATCHER.get_module(self.heatswitch)
        _thread = threading.Thread(target=self._thread)
        _thread.daemon = True
        _thread.start()

    def read_value(self, maxage=0):
        return self.value

    def write_target(self, value):
        # could tell HW
        return round(value, 2)
        # note: setting self.target to the new value is done after this....
        # note: we may also return the read-back value from the hw here

    def read_status(self, maxage=0):
        return (status.OK, '') if self._state == 'idle' else (status.BUSY,
                                                              self._state)

    def _thread(self):
        loopdelay = 1
        while True:
            ts = time.time()
            if self._state == 'idle':
                if self.target != self.value:
                    self.log.debug('got new target -> switching heater on')
                    self._state = 'switch_on'
                    self._heatswitch.write_target('on')
            if self._state == 'switch_on':
                # wait until switch is on
                if self._heatswitch.read_value() == 'on':
                    self.log.debug('heatswitch is on -> ramp to %.3f' %
                                   self.target)
                    self._state = 'ramp'
            if self._state == 'ramp':
                if self.target == self.value:
                    self.log.debug('at field! mode is %r' % self.mode)
                    if self.mode:
                        self.log.debug('at field -> switching heater off')
                        self._state = 'switch_off'
                        self._heatswitch.write_target('off')
                    else:
                        self.log.debug('at field -> hold')
                        self._state = 'idle'
                        self.status = self.read_status()  # push async
                else:
                    step = self.ramp * loopdelay / 60.
                    step = max(min(self.target - self.value, step), -step)
                    self.value += step
            if self._state == 'switch_off':
                # wait until switch is off
                if self._heatswitch.read_value() == 'off':
                    self.log.debug('heatswitch is off at %.3f' % self.value)
                    self._state = 'idle'
            self.read_status()  # update async
            time.sleep(max(0.01, ts + loopdelay - time.time()))
        self.log.error(self, 'main thread exited unexpectedly!')


class CoilTemp(Readable):
    """a coil temperature
    """
    PARAMS = {
        'value': PARAM('Coil temperatur',
                       unit='K', datatype=FloatRange(), default=0,
                       ),
        'sensor': PARAM("Sensor number or calibration id",
                        datatype=StringType(), readonly=True,
                        ),
    }

    def read_value(self, maxage=0):
        return round(2.3 + random.random(), 3)


class SampleTemp(Drivable):
    """a sample temperature
    """
    PARAMS = {
        'value': PARAM('Sample temperature',
                       unit='K', datatype=FloatRange(), default=10,
                       ),
        'sensor': PARAM("Sensor number or calibration id",
                        datatype=StringType(), readonly=True,
                        ),
        'ramp': PARAM('moving speed in K/min',
                      datatype=FloatRange(0, 100), unit='K/min', default=0.1,
                      readonly=False,
                      ),
    }

    def init(self):
        _thread = threading.Thread(target=self._thread)
        _thread.daemon = True
        _thread.start()

    def write_target(self, value):
        # could tell HW
        return round(value, 2)
        # note: setting self.target to the new value is done after this....
        # note: we may also return the read-back value from the hw here

    def _thread(self):
        loopdelay = 1
        while True:
            ts = time.time()
            if self.value == self.target:
                if self.status != status.OK:
                    self.status = status.OK, ''
            else:
                self.status = status.BUSY, 'ramping'
                step = self.ramp * loopdelay / 60.
                step = max(min(self.target - self.value, step), -step)
                self.value += step
            time.sleep(max(0.01, ts + loopdelay - time.time()))
        self.log.error(self, 'main thread exited unexpectedly!')


class Label(Readable):
    """

    """
    PARAMS = {
        'system': PARAM("Name of the magnet system",
                        datatype=StringType, export=False,
                        ),
        'subdev_mf': PARAM("name of subdevice for magnet status",
                           datatype=StringType, export=False,
                           ),
        'subdev_ts': PARAM("name of subdevice for sample temp",
                           datatype=StringType, export=False,
                           ),
        'value': PARAM("final value of label string",
                       datatype=StringType,
                       ),
    }

    def read_value(self, maxage=0):
        strings = [self.system]

        dev_ts = self.DISPATCHER.get_module(self.subdev_ts)
        if dev_ts:
            strings.append('at %.3f %s' %
                           (dev_ts.read_value(), dev_ts.PARAMS['value'].unit))
        else:
            strings.append('No connection to sample temp!')

        dev_mf = self.DISPATCHER.get_module(self.subdev_mf)
        if dev_mf:
            mf_stat = dev_mf.read_status()
            mf_mode = dev_mf.mode
            mf_val = dev_mf.value
            mf_unit = dev_mf.PARAMS['value'].unit
            if mf_stat[0] == status.OK:
                state = 'Persistent' if mf_mode else 'Non-persistent'
            else:
                state = mf_stat[1] or 'ramping'
            strings.append('%s at %.1f %s' % (state, mf_val, mf_unit))
        else:
            strings.append('No connection to magnetic field!')

        return '; '.join(strings)


class DatatypesTest(Readable):
    """
    """
    PARAMS = {
        'enum': PARAM(
            'enum', datatype=EnumType(
                'boo', 'faar', z=9), readonly=False, default=1), 'tupleof': PARAM(
            'tuple of int, float and str', datatype=TupleOf(
                IntRange(), FloatRange(), StringType()), readonly=False, default=(
                1, 2.3, 'a')), 'arrayof': PARAM(
            'array: 2..3 times bool', datatype=ArrayOf(
                BoolType(), 2, 3), readonly=False, default=[
                1, 0, 1]), 'intrange': PARAM(
            'intrange', datatype=IntRange(
                2, 9), readonly=False, default=4), 'floatrange': PARAM(
            'floatrange', datatype=FloatRange(
                -1, 1), readonly=False, default=0, ), 'struct': PARAM(
            'struct(a=str, b=int, c=bool)', datatype=StructOf(
                a=StringType(), b=IntRange(), c=BoolType()), ), }