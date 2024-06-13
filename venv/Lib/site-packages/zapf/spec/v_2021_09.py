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

"""Contains all necessary constants and mappings from the spec for 2012.09 only."""

import struct
from collections import namedtuple

from zapf import ApiError, DescError, SpecError
from zapf.spec import BASETYPE_TO_FMT, FLOAT32_MAX, FMT_TO_BASETYPE, \
    LOWLEVEL, READONLY

DEBUG_Descriptor = 32767

DEVICE_FLAGS = {  # bit number to name
    15: LOWLEVEL,
    14: READONLY,
}
for _ in range(14):
    DEVICE_FLAGS[_] = f'unknown flag {_}'

DESCRIPTOR_CLASS_PLC = 1
DESCRIPTOR_CLASS_DEVICE = 2
DESCRIPTOR_CLASS_STRING = 3
DESCRIPTOR_CLASS_ENUM = 4
DESCRIPTOR_CLASS_BF = 5
DESCRIPTOR_CLASS_PAR = 6
DESCRIPTOR_CLASS_FUNC = 6
DESCRIPTOR_CLASS_TABLE = 6
DESCRIPTOR_CLASS_DEBUG = 0xF

TABLE_ACCESS_PCTL   = 'paramctl'
TABLE_ACCESS_MEM_1D = 'memory-mapped:1d'
TABLE_ACCESS_MEM_2D = 'memory-mapped:2d'


class DescriptorMeta(type):
    def __new__(cls, name, base, attrs):
        newtype = type.__new__(cls, name, base, attrs)
        newtype.FMT = struct.Struct('<' + ''.join(f[0] for f in
                                                  newtype.FIELDS[:-1]))
        newtype.FMT_SIZE = newtype.FMT.size
        return newtype


class Descriptor(metaclass=DescriptorMeta):
    """Descriptor class, handles all bytes after the descriptor type."""

    # FIELDS: fmt, fieldname, resolve 'c'hain, 'd'escriptor or 's'tring,
    # descr_classes for resolve (descriptor type field is omitted here)
    FIELDS = []

    def __init__(self, descriptor_id, descriptor_type, data):
        self.descriptor_id = descriptor_id
        self.descriptor_type = descriptor_type
        self.is_resolved = False

        fmt_size = self.FMT_SIZE
        if len(data) < fmt_size + 1:  # trailing string is zero
            raise DescError(self, f'not enough data for decoding (need '
                            f'{fmt_size+1} bytes, got {len(data)})')
        if data[-1] != 0:
            raise DescError(self, 'data is not zero padded (last '
                            f'byte: {data[-1]} != 0)')

        # calculate potential padding (fields added later, increasing the
        # stringoffset of the descriptor type)
        padding_len = self.descriptor_type & 0xFF - fmt_size - 2
        if padding_len < 0:
            raise DescError(self, 'descriptor type does not have enough '
                            'data fields (padding would be < 0)')
        self.padding = data[fmt_size:fmt_size + padding_len]
        string = data[fmt_size + padding_len:]
        string = string[:string.index(0)].decode('utf-8', 'backslashreplace')
        setattr(self, self.FIELDS[-1][1], string)

        unpacked_data = self.FMT.unpack_from(data)
        for field, value in zip(self.FIELDS, unpacked_data):
            setattr(self, field[1], value)
        self.decode()

    def __repr__(self):
        return (f'{self.__class__.__name__}/{self.descriptor_type:#06x} '
                f'#{self.descriptor_id}')

    def get_chain(self, follow_chain=True):
        """Compose a value representing this descriptor with all references
        resolved and all chains condensed (if follow_chain is True), else just
        return THIS descriptor's info.
        """
        res = self.get_info()
        if self.FIELDS[0][1] == 'prev' and follow_chain:
            if self.prev:
                return self.prev.get_chain(follow_chain) + [res]
            return [res]
        return res

    def resolve(self, indexer):
        """Resolve all descriptor ID fields of this descriptor, except for
        the prev field.
        """
        for entry in self.FIELDS:
            if len(entry) < 4:
                continue
            _fmt, field, resolve_method, allowed_classes = entry
            value = getattr(self, field)
            if resolve_method in 'sc':
                value = self._query_chain(indexer, value, allowed_classes)
            if resolve_method == 'd':
                value = self._query_descriptor(indexer, value, allowed_classes)
            if resolve_method == 's':
                value = value.get_info() if value else ''
            setattr(self, field, value)
        self.is_resolved = True

    # helpers

    def _resolve_chain(self, indexer, descr_classes):
        """Resolve our 'prev-chain' recursively, checking descriptor classes
        on the way.
        """
        if self.FIELDS[0][1] != 'prev':
            raise RuntimeError('resolve_chain only works on descriptors '
                               'with a prev field')
        if isinstance(self.prev, Descriptor):
            return
        if not self.prev:
            self.prev = None
            return
        descr = indexer.query_descriptor(self.prev)
        if descr.descriptor_type >> 12 not in descr_classes:
            raise DescError(self, f'prev descriptor {descr} has unexpected '
                            'class, expected one of {descr_classes}')
        self.prev = descr
        # pylint: disable=protected-access
        descr._resolve_chain(indexer, descr_classes)

    def _query_chain(self, indexer, descr_id, descr_classes):
        """Query a chain of descriptors, returning the last descriptor."""
        if not descr_id:
            return None
        descr = indexer.query_descriptor(descr_id)
        if descr.descriptor_type >> 12 not in descr_classes:
            raise DescError(self, f'chain descriptor {descr} is of wrong '
                            f'type, expected one of {descr_classes}')
        # pylint: disable=protected-access
        descr._resolve_chain(indexer, descr_classes)
        return descr

    def _query_descriptor(self, indexer, descr_id, descr_classes):
        """Query a single descriptor, without building a chain."""
        if not descr_id:
            return None
        descr = indexer.query_descriptor(descr_id)
        if descr.descriptor_type >> 12 not in descr_classes:
            raise DescError(self, f'linked descriptor {descr} is of wrong '
                            f'type, expected one of {descr_classes}')
        return descr

    # methods to be overridden in derived descriptors

    def decode(self):
        """Decode packed/encoded information in the descriptor data.

        Typically rewrites attributes on self with the decoded data.
        """

    def get_info(self):
        """Return a high-level repr of this descriptor as a dictionary."""
        raise NotImplementedError


class PLCDescriptor(Descriptor):
    FIELDS = (('H', 'last_device'),
              ('H', 'description', 's', {DESCRIPTOR_CLASS_STRING}),
              ('H', 'version',     's', {DESCRIPTOR_CLASS_STRING}),
              ('H', 'author',      's', {DESCRIPTOR_CLASS_STRING}),
              ('H', 'slot_size'),
              ('H', 'num_devices'),
              ('H', 'flags'),
              ('s', 'name'),
              )
    last_device = description = version = author = slot_size = num_devices = \
        flags = name = None

    def decode(self):
        self.num_slots = 1 + (self.flags >> 12)
        self.flags = []

    def check(self):
        if self.slot_size < 32:
            raise DescError(self, f'slot size ({self.slot_size}) too small, '
                            'should at least be 32')
        if self.slot_size % 8:
            raise DescError(self, f'slot size ({self.slot_size}) must be '
                            'divisble by 8')

    def get_info(self):
        self.check()
        return dict(
            type='plc',
            description=self.description,
            version=self.version,
            author=self.author,
            slot_size=self.slot_size,
            num_devices=self.num_devices,
            flags=self.flags,
            num_slots=self.num_slots,
            name=self.name,
        )


class DeviceDescriptor(Descriptor):
    FIELDS = (('H', 'prev'),
              ('H', 'description', 's', {DESCRIPTOR_CLASS_STRING}),
              ('H', 'main_param',  'd', {DESCRIPTOR_CLASS_PAR}),
              ('H', 'aux',         'c', {DESCRIPTOR_CLASS_BF}),
              ('H', 'accessibles', 'c', {DESCRIPTOR_CLASS_PAR,
                                         DESCRIPTOR_CLASS_FUNC,
                                         DESCRIPTOR_CLASS_TABLE}),
              ('H', 'errid',       'c', {DESCRIPTOR_CLASS_BF}),
              ('H', 'typecode'),
              ('H', 'address'),
              ('H', 'flags'),
              ('s', 'name'),
              )
    prev = description = main_param = aux = accessibles = errid = typecode = \
        address = flags = name = None

    def decode(self):
        flags = []
        for flag, text in DEVICE_FLAGS.items():
            if self.flags & (1 << flag):
                flags.append(text)
        self.flags = flags

    def check(self):
        if self.address % 2:
            raise DescError(self, f'address {self.address} should be at '
                            'least even')
        # XXX: check chain of accessibles

    def get_info(self):
        self.check()
        # fixup main_param
        main_param = self.main_param.get_info() if self.main_param else {}
        main_param.pop('type', None)
        main_param.pop('idx', None)
        main_param.pop('name', None)
        main_param.pop('description', None)

        mp_basetype = main_param.pop('basetype', None)
        mp_width = main_param.pop('width', 0)
        # pylint: disable=import-outside-toplevel
        from zapf.device import TYPECODE_MAP
        try:
            typecode_info = TYPECODE_MAP[self.typecode]
            basetype, width = FMT_TO_BASETYPE[typecode_info.value_fmt]
            readonly = typecode_info.readonly
            # correct basetype if param descriptor is enum
            if mp_basetype == 'enum' and basetype in ('int', 'uint'):
                basetype = 'enum'
            # it is ok to have a mismatch int<->uint
            if mp_basetype and mp_basetype != basetype and \
               'float' in (mp_basetype, basetype):
                raise DescError(self, f'type {mp_basetype} declared in main '
                                'param descriptor does not match typecode')
            if mp_width and mp_width != width:
                raise DescError(self, f'width {mp_width} declared in main '
                                'param descriptor does not match typecode')
        except KeyError:
            basetype, width, readonly = None, 0, True
        access = 'ro' if readonly or 'readonly' in self.flags else 'rw'
        absmin = main_param.pop('min_value', -FLOAT32_MAX)
        absmax = main_param.pop('max_value', FLOAT32_MAX)
        unit = main_param.pop('unit', '')
        enum_r = main_param.pop('enum_r', None)
        enum_w = main_param.pop('enum_w', None)
        params, funcs, tables = {}, {}, {}
        descr = self.accessibles

        # XXX: need stricter checking of alloced ressources (IDX, mem area, etc...)
        def fixup(entry):
            # fix unit
            if 'unit' in entry:
                entry['unit'] = entry['unit'].replace('#', unit)
            return entry

        while descr:
            if isinstance(descr, (NumericParameterDescriptor, EnumParameterDescriptor)):
                if descr.name not in params:
                    entry = descr.get_info()
                    name = entry.pop('name')
                    params[name] = fixup(entry)
            elif isinstance(descr, SpecialFunctionDescriptor):
                if descr.name not in funcs:
                    entry = descr.get_info()
                    name = entry.pop('name')
                    # fix unit
                    d = entry['argument']
                    if d:
                        entry['argument'] = fixup(d)
                    d = entry['result']
                    if d:
                        entry['argument'] = fixup(d)
                    funcs[name] = entry
            elif isinstance(descr, TableDescriptor):
                if descr.name not in tables:
                    entry = descr.get_info()
                    name = entry.pop('name')
                    tables[name] = fixup(entry)
            else:
                raise DescError(self, 'accessible chain must only contain param/'
                                f'function/table descriptors, not {descr}')
            descr = descr.prev

        # clean bitfield descriptions for aux/errid, if existing
        def clean_bitfield_chain(chain):
            res = []
            seen = {}
            for elem in reversed(chain):
                if len(elem) == 2:  # flag: bit, name
                    if elem[0] not in seen:
                        seen[elem[0]] = elem
                        res.append(elem)
                elif len(elem) == 4:  # bitfield: lsb, width, name, enum
                    # if any bit of the bitfield already seen -> skip bitfield
                    for i in range(elem[0], elem[0] + elem[1]):
                        if i not in seen:
                            continue
                        newer = seen[i]
                        if len(newer) == 4:  # also a bitfield
                            # -> update newer with the older entries, IF THEY FIT
                            # same startbit and newer is at least as wide
                            if newer[0] == elem[0] and newer[1] >= elem[1]:
                                for k, v in elem[3].items():  # iter over the enums
                                    # update if not already given later
                                    newer[3].setdefault(k, v)
                            res.remove(newer)
                            res.append(newer)
                        break
                    else:
                        for i in range(elem[0], elem[0] + elem[1]):
                            seen[i] = elem
                        res.append(elem)
            # now sort res ascending
            res.sort()
            return res

        aux = clean_bitfield_chain(self.aux.get_chain()) if self.aux else []
        errid = clean_bitfield_chain(self.errid.get_chain()) if self.errid else []
        # TODO: need a way to check/complain if a parameterless device has
        # parameter descriptors, also main parameter type of MessageIO
        return dict(
            description=self.description,
            aux=aux,
            params=params,
            funcs=funcs,
            tables=tables,
            errid=errid,
            typecode=self.typecode,
            address=self.address,
            flags=self.flags,
            absmin=absmin,
            absmax=absmax,
            unit=unit,
            basetype=basetype,
            width=width,
            access=access,
            enum_r=enum_r,
            enum_w=enum_w,
            name=self.name,
        )


class StringDescriptor(Descriptor):
    FIELDS = (('H', 'prev'),
              ('s', 'text'),
              )
    prev = text = None

    def get_info(self):
        if self.prev:
            if not isinstance(self.prev, StringDescriptor):
                raise DescError(self, 'string descriptors may not reference '
                                'other types of descriptors ({self.prev})')
            return self.prev.get_info() + self.text
        return self.text


class EnumDescriptor(Descriptor):
    FIELDS = (('H', 'prev'),
              ('H', 'val'),
              ('s', 'name'),
              )
    prev = val = name = None

    def get_info(self):
        if self.prev:
            if not isinstance(self.prev, EnumDescriptor):
                raise DescError(self, 'enum descriptors may not reference '
                                'other types of descriptors ({self.prev})')
            res = self.prev.get_info()
            res.update({self.val: self.name})
            return res
        return {self.val: self.name}


class BitfieldDescriptor(Descriptor):
    FIELDS = (('H', 'prev'),
              ('H', 'enum', 'c', {DESCRIPTOR_CLASS_ENUM}),
              ('B', 'lsb'),
              ('B', 'width'),
              ('s', 'name'),
              )
    prev = enum = lsb = width = name = None

    def decode(self):
        self.name = self.name.replace(' ', '_')

    def check(self):
        if self.width < 1:
            raise DescError(self, 'width of bitfield must be at least 1')
        mask = (1 << self.width) - 1
        if self.enum:
            if max(self.enum.get_info()) > mask:
                raise DescError(self, 'enum chain contains entries which won\'t '
                                f'fit into {self.width} bits')
        if self.prev:
            if not isinstance(self.prev, (BitfieldDescriptor, FlagDescriptor)):
                raise DescError(self, 'bitfield descriptors may only reference '
                                f'bitfield or flag descriptors, not {self.prev}')
        # XXX: check overrides correctly!

    def get_info(self):
        self.check()
        # XXX: handle overrides correctly!
        return (self.lsb, self.width, self.name,
                self.enum.get_info() if self.enum else {})


class FlagDescriptor(Descriptor):
    FIELDS = (('H', 'prev'),
              ('B', 'bit'),
              ('s', 'name'),
              )
    prev = bit = name = None

    def get_info(self):
        if self.prev:
            if not isinstance(self.prev, (BitfieldDescriptor, FlagDescriptor)):
                raise DescError(self, 'flag descriptors may only reference '
                                f'bitfield or flag descriptors, not {self.prev}')
        return (self.bit, self.name)


class NumericParameterDescriptor(Descriptor):
    FIELDS = (('H', 'prev'),
              ('H', 'description', 'c', {DESCRIPTOR_CLASS_STRING}),
              ('H', 'idx'),
              ('H', 'ptype'),
              ('H', 'unit'),
              ('f', 'min_value'),
              ('f', 'max_value'),
              ('s', 'name'),
              )
    prev = description = idx = ptype = unit = min_value = max_value = name = None

    def decode(self):
        self.unit = decode_unit(self.unit)
        self.idx += (self.ptype & 0xFF) << 16
        self.basetype = {0: 'uint', 1: 'float', 3: 'int'}[(self.ptype >> 14) & 0x3]
        self.access = {1: 'rw', 2: 'ro', 3: 'obs'}[(self.ptype >> 12) & 0x3]
        self.width = {0: 0, 1: 16, 2: 32, 3: 64}[(self.ptype >> 10) & 0x3]

    def check(self):
        if self.max_value < self.min_value:
            raise DescError(self, f'parameter max_value ({self.max_value}) must '
                            f'be bigger than min_value ({self.min_value})')
        if self.prev and isinstance(self.prev, Descriptor):
            if not isinstance(self.prev, (NumericParameterDescriptor,
                                          EnumParameterDescriptor,
                                          SpecialFunctionDescriptor,
                                          TableDescriptor)):
                raise DescError(self, 'param descriptors may only reference '
                                'param/function/table descriptors, '
                                f'not {self.prev}')

    def get_info(self):
        self.check()
        return dict(
            type='param',
            description=self.description.get_info() if self.description else '',
            idx=self.idx,
            basetype=self.basetype,
            width=self.width,
            access=self.access,
            unit=self.unit,
            min_value=self.min_value,
            max_value=self.max_value,
            name=self.name,
        )


class EnumParameterDescriptor(Descriptor):
    FIELDS = (('H', 'prev'),
              ('H', 'description', 'c', {DESCRIPTOR_CLASS_STRING}),
              ('H', 'enum_r',      'c', {DESCRIPTOR_CLASS_ENUM}),
              ('H', 'enum_w',      'c', {DESCRIPTOR_CLASS_ENUM}),
              ('H', 'idx'),
              ('H', 'ptype'),
              ('s', 'name'),
              )
    prev = description = enum_r = enum_w = idx = ptype = name = None

    def decode(self):
        self.idx += (self.ptype & 0xFF) << 16
        self.width = {0: 0, 1: 16, 2: 32, 3: 64}[(self.ptype >> 10) & 0x3]
        self.access = {1: 'rw', 2: 'ro', 3: 'obs'}[(self.ptype >> 12) & 0x3]
        if not self.enum_r:
            self.enum_r = self.enum_w
        if not self.enum_w:
            self.enum_w = self.enum_r

        if not self.enum_r:
            raise DescError(self, 'enums for an enum parameter may not '
                            'be empty')

    def check(self):
        enum_r = self.enum_r.get_info() if self.enum_r else {}
        enum_w = self.enum_w.get_info() if self.enum_w else {}
        if (self.ptype >> 14) & 0x3 != 2:
            raise DescError(self, 'enum descriptor parameter ptype needs '
                            f'to be enum, not {self.ptype}')
        for k, v in enum_w.items():
            if k not in enum_r:
                raise DescError(self, 'all write_enum entries must also be '
                                f'in the read_enums (missing {k!r})')
            if enum_r[k] != v:
                raise DescError(self, 'all write_enum entries must be the '
                                'same as in the read_enums (error at {k!r})')
        if self.prev:
            if not isinstance(self.prev, (NumericParameterDescriptor,
                                          EnumParameterDescriptor,
                                          SpecialFunctionDescriptor,
                                          TableDescriptor)):
                raise DescError(self, 'param descriptors may only reference '
                                'param/function/table descriptors, '
                                f'not {self.prev}')

    def get_info(self):
        self.check()
        # reverse enum_w
        enum_w = {}
        for k, v in (self.enum_w.get_info() if self.enum_w else {}).items():
            enum_w[v] = k
        return dict(
            type='param',
            description=self.description.get_info() if self.description else '',
            idx=self.idx,
            basetype='enum',
            width=self.width,
            access=self.access,
            enum_r=self.enum_r.get_info() if self.enum_r else {},
            enum_w=enum_w,
            name=self.name,
        )


class SpecialFunctionDescriptor(Descriptor):
    FIELDS = (('H', 'prev'),
              ('H', 'description', 'c', {DESCRIPTOR_CLASS_STRING}),
              ('H', 'argument',    'd', {DESCRIPTOR_CLASS_PAR}),
              ('H', 'result',      'd', {DESCRIPTOR_CLASS_PAR}),
              ('H', 'idx'),
              ('H', 'ptype'),
              ('s', 'name'),
              )
    prev = description = argument = result = idx = ptype = name = None

    def decode(self):
        self.idx += (self.ptype & 0xFF) << 16

    def check(self):
        # XXX: what to check?
        pass

    def get_info(self):
        self.check()
        if (self.ptype >> 12) & 0x3 != 0:
            raise DescError(self, 'function descriptors must have ptype = 0, '
                            f'not {self.ptype}')
        if self.prev:
            if not isinstance(self.prev, (NumericParameterDescriptor,
                                          EnumParameterDescriptor,
                                          SpecialFunctionDescriptor,
                                          TableDescriptor)):
                raise DescError(self, 'function descriptors may only reference '
                                'param/function/table descriptors, '
                                f'not {self.prev}')
        if self.argument and not isinstance(self.argument, (NumericParameterDescriptor,
                                                            EnumParameterDescriptor)):
            raise DescError(self, 'argument descriptors must be param '
                            f'descriptors, not {self.argument}')
        if self.result and not isinstance(self.result, (NumericParameterDescriptor,
                                                        EnumParameterDescriptor)):
            raise DescError(self, 'result descriptors must be one param '
                            f'descriptors, not {self.result}')
        arg = self.argument.get_info() if self.argument else {}
        arg.pop('type', None)
        arg.pop('idx', None)
        res = self.result.get_info() if self.result else {}
        res.pop('type', None)
        res.pop('idx', None)
        return dict(
            description=self.description.get_info(),
            argument=arg,
            result=res,
            idx=self.idx,
            name=self.name,
        )


class TableDescriptor(Descriptor):
    FIELDS = (('H', 'prev'),
              ('H', 'description', 'c', {DESCRIPTOR_CLASS_STRING}),
              ('H', 'header',      'c', {DESCRIPTOR_CLASS_PAR, DESCRIPTOR_CLASS_FUNC}),
              ('H', 'idx'),
              ('H', 'flags'),
              ('H', 'last_row'),
              ('H', 'last_column'),
              ('s', 'name'),
              )
    prev = description = header = idx = flags = last_row = last_column = name = None

    def decode(self):
        self.accesstype = {
            0: TABLE_ACCESS_PCTL,
            1: TABLE_ACCESS_MEM_1D,
            2: TABLE_ACCESS_MEM_2D,
        }[self.flags >> 14]

    def check(self):
        # XXX: check chain of header descr.
        if self.prev:
            if not isinstance(self.prev, (NumericParameterDescriptor,
                                          EnumParameterDescriptor,
                                          SpecialFunctionDescriptor,
                                          TableDescriptor)):
                raise DescError(self, 'table descriptors may only reference '
                                'param/function/table descriptors, '
                                f'not {self.prev}')

    def get_info(self):
        self.check()
        header = self.header.get_chain()  # keep as list to keep the order!
        # overriding logic (headerlist is already reversed)
        # + check header entries
        seen = {}
        undefined_columns = set(range(self.last_column + 1))
        for entry in header:
            # check bad idx
            if entry['idx'] > self.last_column:
                # or just ignore those???
                raise SpecError(f'table {self.name!r} header chain entry '
                                f'{entry["name"]!r} has idx '
                                f'{entry["idx"]} > {self.last_column}!')
            # overrides 'in-order' by 'idx' field (as spec say)
            seen[entry['idx']] = entry
            undefined_columns.discard(entry['idx'])
        if undefined_columns:
            raise SpecError(f'table {self.name!r} header chain contains '
                            f'no entry/ies for column(s) '
                            f'{", ".join(str(d) for d in undefined_columns)}')
        header = list(seen.values())
        return dict(
            type='table',
            description=self.description.get_info(),
            header=header,
            idx=self.idx,
            flags=self.flags,
            accesstype=self.accesstype,
            last_row=self.last_row,
            last_column=self.last_column,
            name=self.name,
        )


class DebugDescriptor(Descriptor):
    FIELDS = (('H', 'cycle'),
              ('H', 'indexer_size'),
              ('s', 'text'),
              )
    cycle = text = None

    def get_info(self):
        return f'{self.cycle:#06x}: {self.text!r}'


# Note: the fmt for the trailing string is computed and appended
# in indexer.query_descriptor
DESCRIPTOR_TYPES = {  # map DescriptorType to minstorage
    0x10: PLCDescriptor,
    0x20: DeviceDescriptor,
    0x30: StringDescriptor,
    0x40: EnumDescriptor,
    0x50: BitfieldDescriptor,
    0x51: FlagDescriptor,
    0x61: NumericParameterDescriptor,
    0x62: EnumParameterDescriptor,
    0x68: SpecialFunctionDescriptor,
    0x6C: TableDescriptor,
    0xFF: DebugDescriptor,
}

# info about a value (device value, parameter, sfunc arg/result, table column)
ValueInfo = namedtuple(
    'ValueInfo',
    'basetype fmt readonly unit min_value max_value enum_r enum_w description')


def get_valueinfo(entry, device_valuesize):
    key = (entry['basetype'], entry['width'] or 8*device_valuesize)
    fmt, min_value, max_value = BASETYPE_TO_FMT[key]
    if struct.calcsize(fmt) > device_valuesize:
        raise SpecError('no parameter can have a bigger width than the main '
                        'value of a device')
    return ValueInfo(entry['basetype'],
                     fmt,
                     entry['access'] != 'rw',
                     entry.get('unit', ''),
                     entry.get('min_value', min_value),
                     entry.get('max_value', max_value),
                     entry.get('enum_r', None),
                     entry.get('enum_w', None),
                     entry.get('description', ''),
                     )


BASE_UNITS = {
    0: '',
    1: '#',
    2: 'bar',
    3: 'counts',
    4: 'deg',
    5: 'g',
    6: 'm',
    7: 'm^2',
    8: 'm^3',
    9: 's',
    10: 'A',
    11: 'F',
    12: 'H',
    13: 'K',
    14: 'Ohm',
    15: 'T',
    16: 'V',
    17: 'W',
    18: 'degC',
    19: 'degF',
    20: 'bit',
    21: 'steps',
}

TIME_BASE = {
    0: '',
    1: '/(#)',
    2: '/s',
    3: '/s^2',
    4: '/s^3',
    5: '/min',
    6: '/h',
    7: '/d',
}

EXPONENTS = {
    0: '',
    2: 'h',
    3: 'k',
    6: 'M',
    9: 'G',
    12: 'T',
    15: 'P',
    -1: 'd',
    -2: 'c',
    -3: 'm',
    -6: 'u',
    -9: 'n',
    -12: 'f',
    -15: 'a',
}

SPECIALS = [  # final decoded, from lookup, exponent correction
    ('l', 'm^3', -3),  # 'l' is 10^-3 m^3
    ('%', '', -2),     # '%' is 10^-2
]


# for tests/testplc
def encode_unit(unit=''):
    if unit == '':
        return 0
    unit = unit.strip()
    for s in SPECIALS:
        if s[0] in unit:
            return (encode_unit(unit.replace(s[0], s[1])) + (s[2] << 11)) & 0xFFFF
    timebase = 0
    if '/' in unit:
        for tbc, tbv in TIME_BASE.items():
            if tbv and unit.endswith(tbv):
                timebase = tbc
                unit = unit.rsplit(tbv, 1)[0]
                unit = unit.strip()
                break
        else:
            raise ApiError(f'can\'t encode unit "{unit}"')
    baseunit = 0
    if unit.endswith('1'):
        unit = unit[1:]
    if unit:
        for bc, bv in BASE_UNITS.items():
            if bv and unit.endswith(bv):
                baseunit = bc
                unitprefix = unit.rsplit(bv, 1)[0]
                unitprefix = unitprefix.strip()
                for ec, ev, in EXPONENTS.items():
                    if unitprefix == ev:
                        return (baseunit + (timebase << 6) + (ec << 11)) & 0xFFFF
                for ec in range(-16, 16):
                    if unitprefix == f'10^{ec}':
                        return (baseunit + (timebase << 6) + (ec << 11)) & 0xFFFF
        raise ApiError(f'can\'t encode unit "{unit}"')
    for ec, ev, in EXPONENTS.items():
        if unit == ev:
            return (baseunit + (timebase << 6) + (ec << 11)) & 0xFFFF
    for ec in range(-16, 16):
        if unit == f'10^{ec}':
            return (baseunit + (timebase << 6) + (ec << 11)) & 0xFFFF
    raise ApiError(f'can\'t encode unit "{unit}"')


def decode_unit(code):
    base_code = code & 0x3F
    tb_code = (code >> 6) & 0x1F
    exp_code = (code >> 11) & 0x1F
    if exp_code > 15:
        # make signed
        exp_code -= 32

    if base_code == tb_code == 1:
        base_code = tb_code = 0

    base_unit = BASE_UNITS.get(base_code, 'unknown unit')
    for s in SPECIALS:
        if base_unit == s[1] and exp_code <= s[2]:
            base_unit = s[0]
            exp_code -= s[2]
    time_base = TIME_BASE.get(tb_code, '')
    if time_base and not base_unit:
        return (f'10^{exp_code}' if exp_code else '1') + time_base
    exponent = f'10^{exp_code} ' if exp_code else ''
    if '^' not in base_unit and base_code > 1:
        exponent = EXPONENTS.get(exp_code, exponent)
    return f'{exponent}{base_unit}{time_base}'


def fix_unit(param_unit, main_unit):
    # fixes the unit strings
    if '#' not in param_unit:
        return param_unit
    if not main_unit:
        if '1/(#)' in param_unit:
            return param_unit.replace('1/(#)', '')
        elif '#' in param_unit:
            return param_unit.replace('#', '')
    return param_unit.replace('#', main_unit)
