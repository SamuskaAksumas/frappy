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

"""Contains all necessary constants and mappings from the spec for 2015.02 only"""

# 2015.02
from zapf.spec import FLOAT32_MAX

INDEXER_DEV = 0

INFO_STRUCT      = 0
INFO_SIZE        = 1
INFO_ADDR        = 2
INFO_UNIT        = 3
INFO_NAME        = 4
INFO_VERSION     = 5
INFO_AUTHOR1     = 6
INFO_AUTHOR2     = 7
INFO_DESCRIPTOR  = 8
INFO_PARAMS      = 15
INFO_AUX1        = 16
INFO_CYCLE       = 127


UNIT_CODES    = ('', 'V', 'A', 'W', 'm', 'g', 'Hz', 'T', 'K', 'degC', 'degF',
                 'bar', 'deg', 'Ohm', 'm/s', 'm^2/s', 'm^3/s', 's', 'cts',
                 'bar/s', 'bar/s^2', 'F', 'H', 'l/min')
UNIT_EXPONENT = {0: '', 2: 'h', 3: 'k', 6: 'M', 9: 'G', 12: 'T', 15: 'P',
                 18: 'E', -1: 'd', -2: 'c', -3: 'm', -6: 'u', -9: 'n',
                 -12: 'f', -15: 'a'}
UNIT_SPECIAL  = {(0, -2): '%', (16, -3): 'l/s'}


def convert_unit(code, exponent):
    if (code, exponent) in UNIT_SPECIAL:
        return UNIT_SPECIAL[code, exponent]
    try:
        unit = UNIT_CODES[code]
    except IndexError:
        unit = 'unit'
    if '^' in unit:
        return f'10^{exponent} {unit}'
    return UNIT_EXPONENT.get(exponent, f'10^{exponent} ') + unit


# The first (numbered) parameter whose value is float instead of integer.
FIRST_FLOAT_PARAM = 30


def is_function(idx):
    return 128 <= idx < 192 or 224 <= idx < 240


# helpers for converting a 2015.02 into a 2021.09 spec

PARAMETER_Specs = {
    1:  dict(name='Mode', basetype='int'),
    2:  dict(name='MicroSteps', basetype='int', minval=1),
    3:  dict(name='CoderBits', basetype='int', minval=1),
    4:  dict(name='LegacyExtStatus', basetype='int'),
    5:  dict(name='LegacyHwOffset', basetype='int'),

    30: dict(name='LegacyAbsMin', unit='#'),
    31: dict(name='LegacyAbsMax', unit='#'),
    32: dict(name='UserMin', unit='#'),
    33: dict(name='UserMax', unit='#'),
    34: dict(name='WarnMin', unit='#'),
    35: dict(name='WarnMax', unit='#'),
    36: dict(name='TimeoutTime', minval=0, unit='s'),
    37: dict(name='MaxTravelDist', minval=0, unit='#'),
    38: dict(name='AccelTime', minval=0, unit='s'),
    40: dict(name='Offset', unit='#'),
    43: dict(name='BlockSize', minval=0, unit='#'),
    44: dict(name='Opening', minval=0, unit='#'),
    51: dict(name='PidP', minval=0, maxval=100, unit='%/(#)'),
    52: dict(name='PidI', minval=0, maxval=900, unit='s'),
    53: dict(name='PidD', minval=0, maxval=100, unit='1/s'),
    55: dict(name='DragError', minval=0, unit='#'),
    56: dict(name='Hysteresis', minval=0, unit='#'),
    57: dict(name='Holdback', minval=0, unit='#'),
    58: dict(name='HomingSpeed', minval=0, unit='#/s'),
    59: dict(name='Jerk', minval=0, unit='#/s'),
    60: dict(name='Speed', minval=0, unit='#/s'),
    61: dict(name='Accel', minval=0, unit='#/s^2'),
    62: dict(name='IdleCurrent', minval=0, maxval=100, unit='A'),
    63: dict(name='RampCurrent', minval=0, maxval=100, unit='A'),
    64: dict(name='MoveCurrent', minval=0, maxval=100, unit='A'),
    65: dict(name='StopCurrent', minval=0, maxval=100, unit='A'),
    66: dict(name='LegacyAbortDecel'),
    67: dict(name='LegacyMicrosteps'),
    68: dict(name='Slope', unit='steps/(#)'),
    69: dict(name='HomePosition', unit='#'),
    70: dict(name='Setpoint', access='obs', unit='#'),
}

# special functions
FUNC_Specs = {
    128: dict(name='FactoryReset', argument=dict(basetype='int')),
    130: dict(name='LegacyAbort'),
    131: dict(name='LegacyHomeNeg'),
    132: dict(name='LegacyHomePos'),
    133: dict(name='Home'),
    137: dict(name='SetPosition', argument=dict(unit='#')),
    142: dict(name='ContMove', argument=dict(unit='#/s')),
}

# fill up undef'd + custom idx with generic entries (param224, etc..)
for i in range(1, 240):
    if not is_function(i):
        if i not in PARAMETER_Specs:
            PARAMETER_Specs[i] = dict(name=f'Param{i}')
    else:
        if i not in FUNC_Specs:
            FUNC_Specs[i] = dict(name=f'Command{i}')
for i in range(240, 256):
    PARAMETER_Specs[i] = dict(name=f'Reserved{i}')

# augment with default values
for k, v in PARAMETER_Specs.items():
    v.setdefault('basetype', 'float')
    def_minval, def_maxval = (-FLOAT32_MAX, FLOAT32_MAX) \
        if v['basetype'] == 'float' else (-(2 ** 31), 2 ** 31 - 1)
    v.setdefault('unit', '')
    v.setdefault('min_value', def_minval)
    v.setdefault('max_value', def_maxval)
    v['width'] = 0  # ALWAYS same as device value
    v.setdefault('access', 'rw')
    v.setdefault('type', 'param')
    v.setdefault('description', '')

for k, v in FUNC_Specs.items():
    if 'argument' in v:
        arg = v['argument']
        arg.setdefault('basetype', 'float')
        arg['width'] = 0
        arg['access'] = 'rw'
        arg.setdefault('unit', '')
        arg.setdefault('min_value', -FLOAT32_MAX)
        arg.setdefault('max_value', FLOAT32_MAX)
        v.setdefault('type', 'param')
    v.setdefault('argument', None)
    v.setdefault('result', None)  # does not occur
    v.setdefault('description', '')
