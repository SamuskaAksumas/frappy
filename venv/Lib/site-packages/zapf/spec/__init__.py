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
#   Enrico Faulhaber <enrico.faulhaber@frm2.tum.de>
#
# *****************************************************************************

from zapf import SpecError
from zapf.util import BitFields, UncasedMap

SUPPORTED_MAGICS = ['2015_02', '2021_09']

OFFSET_ADDR = 4
FLOAT32_MAX = 3.402823e38
FLOAT64_MAX = 1.7976931348623158e308
StatusStruct16 = BitFields(STATE=(15, 12), REASON=(11, 8), AUX=(7, 0))
StatusStruct32 = BitFields(STATE=(31, 28), REASON=(27, 24), AUX=(23, 0))
ParamControl = BitFields(available=(15, 15), CMD=(15, 13), IDX=(12, 0))

ReasonMap = [
    '',
    'inhibit',
    'timeout',
    '(inhibit, timeout)',
    'lower limit reached',
    '(inhibit, lower limit)',
    '(timeout, lower limit)',
    '(inhibit, timeout, lower limit)',
    'upper limit reached',
    '(inhibit, upper limit)',
    '(timeout, upper limit)',
    '(inhibit, timeout, upper limit)',
    'both limits reached',
    '(inhibit, both limits)',
    '(timeout, both limits)',
    '(inhibit, timeout, both limits)',
]

ReasonSpec = [[0, 4, '', {i: ReasonMap[i] for i in range(0, 16)}]]


ParamCMDs = UncasedMap(
    INIT=0,
    DO_READ=1,
    DO_WRITE=2,
    BUSY=3,
    DONE=4,
    ERR_NO_IDX=5,
    ERR_RO=6,
    ERR_RETRY=7,
)

DevStatus = UncasedMap(
    RESET=0,
    IDLE=1,
    DISABLED=2,
    WARN=3,
    START=5,
    BUSY=6,
    STOP=7,
    ERROR=8,
    DIAGNOSTIC_ERROR=13,
)

StatusSpec = [[0, 4, '', {i: DevStatus[i]
                          for i in (0, 1, 2, 3, 5, 6, 7, 8, 13)}]]

ErridSpec = [[0, 16, 'errid', {0: ''}]]

FMT_TO_BASETYPE = {
    'h': ('int', 16),
    'H': ('uint', 16),
    'i': ('int', 32),
    'I': ('uint', 32),
    'q': ('int', 64),
    'Q': ('uint', 64),
    'e': ('float', 16),
    'f': ('float', 32),
    'd': ('float', 64),
}

# map basetype to fmt, min_value, max_value
BASETYPE_TO_FMT = {
    ('int', 16):   ('h', -32768, 32767),
    ('uint', 16):  ('H', 0, 65535),
    ('int', 32):   ('i', -2147483648, 2147483647),
    ('uint', 32):  ('I', 0, 4294967295),
    ('int', 64):   ('q', -9223372036854775808, 9223372036854775807),
    ('uint', 64):  ('Q', 0, 18446744073709551615),
    ('float', 16): ('e', -65504, 65504),
    ('float', 32): ('f', -FLOAT32_MAX, FLOAT32_MAX),
    ('float', 64): ('d', -FLOAT64_MAX, FLOAT64_MAX),
    ('enum', 16):  ('H', 0, 65535),
    ('enum', 32):  ('I', 0, 65535),  # enum values are always only 16 bit
    ('enum', 64):  ('Q', 0, 65535),  # independent of their field width
}

# possible flags:
READONLY = 'readonly'
LOWLEVEL = 'lowlevel'


def decode_bitfield(value, spec=None):
    res = []
    if spec is None:
        return value
    for entry in spec:
        if len(entry) == 2:
            lsb, key = entry
            if value & (1 << lsb):
                res.append(key)
        elif len(entry) == 4:
            lsb, width, bfname, enums = entry
            mask = (1 << width) - 1
            bfvalue = (value >> lsb) & mask
            bfvalstr = enums.get(bfvalue, str(bfvalue))
            if bfvalstr:
                if bfname:
                    res.append(f'{bfname}={bfvalstr}')
                else:
                    res.append(bfvalstr)
        else:
            raise SpecError('invalid flag/bitfield specification')
    return ', '.join(res)
