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

"""Interaction with the indexer."""

import time

from zapf import CommError, SpecError, spec
from zapf.spec import v_2015_02, v_2021_09


class Indexer:
    IO_RETRIES = 32

    def __init__(self, io, log):
        self.io = io
        self.log = log
        self.magicstr = None
        self.indexer_addr = None
        self.indexer_size = None
        self.num_devices = 0
        self.plc_name = None
        self.plc_version = None
        self.plc_author = None
        self.plc_description = None
        self._descr_cache = {}
        self.error = None

    def detect_plc(self, extended=True):
        """Checks if the HW follows the spec and which MAGIC number it has.

        Afterwards, checks the indexer and reads extended meta information
        about the PLC if wanted.
        """
        if self.indexer_addr and self.indexer_size and self.plc_author:
            return

        # check MAGIC
        magic = self.io.detect_magic()
        magicstr = f'{magic:.2f}'.replace('.', '_')
        if magicstr not in spec.SUPPORTED_MAGICS:
            raise SpecError(f'magic {magicstr} is not supported by this client')
        self.magicstr = magicstr

        # read indexer offset from memory address 4
        addr = self.io.read_u16(spec.OFFSET_ADDR)
        if addr < 6 or addr & 1:
            raise SpecError(f'indexer offset {addr} is invalid')
        self.indexer_addr = addr

        # continue in a magic specific way
        getattr(self, f'_check_{magicstr}')(extended)

    def _check_2015_02(self, extended):
        # query indexer size
        size = self.query_word(v_2015_02.INDEXER_DEV, v_2015_02.INFO_SIZE)
        if size < 22 or size > 66 or size & 1:
            raise SpecError(f'indexer size {size} is invalid')
        self.indexer_slot_size = size - 2
        self.indexer_num_slots = 1
        self.indexer_data_size = self.indexer_slot_size * self.indexer_num_slots
        self.indexer_size = 2 + self.indexer_data_size

        # query indexer size and offset again, through the info struct,
        # and ensure consistency
        info = self.query_infostruct(v_2015_02.INDEXER_DEV)
        self.num_devices = 0
        if info[0] + info[1] + info[2] != 0:
            if info[0] != 0 or \
               info[1] != self.indexer_size or \
               info[2] != self.indexer_addr:
                raise SpecError('indexer information from infostruct does not '
                                'match with OFFSET or size')

            # total number of devices can be given in flags
            self.num_devices = info[4] & 0xFF

        # query firmware information
        if extended:
            self.plc_name = self.query_string(
                v_2015_02.INDEXER_DEV, v_2015_02.INFO_NAME
            )
            self.plc_version = self.query_string(
                v_2015_02.INDEXER_DEV, v_2015_02.INFO_VERSION
            )
            author1 = self.query_string(v_2015_02.INDEXER_DEV, v_2015_02.INFO_AUTHOR1)
            author2 = self.query_string(v_2015_02.INDEXER_DEV, v_2015_02.INFO_AUTHOR2)
            self.plc_author = (author1 or 'Anonymous') + '\n' + author2
            self.plc_description = ''

    def _check_2021_09(self, extended):
        # safe starting values (mailbox + minimum slot size)
        self.indexer_slot_size = 32
        self.indexer_num_slots = 1
        self.indexer_data_size = 32
        self.indexer_size = 2 + self.indexer_data_size

        # read beginning of PLC-descripor
        initial_plc_descriptor = self.query_descriptor_uncached(0)

        error = False
        # in case the indexer is stuck at the DebugDescriptor:
        if isinstance(initial_plc_descriptor, v_2021_09.DebugDescriptor):
            self.indexer_slot_size = initial_plc_descriptor.indexer_size
            error = True
        else:
            self.indexer_slot_size = initial_plc_descriptor.slot_size
            self.indexer_num_slots = initial_plc_descriptor.num_slots

        # set correct values
        self.indexer_data_size = self.indexer_slot_size * self.indexer_num_slots
        self.indexer_size = 2 + self.indexer_data_size

        # re-read full sized plc_descriptor
        plc_descriptor = self.query_descriptor(v_2021_09.DEBUG_Descriptor
                                               if error else 0)
        if error:
            self.error = self.plc_description = plc_descriptor.text
            raise CommError('can not initialize, PLC stuck at DebugDescriptor '
                            f'{plc_descriptor.text!r}')

        self.num_devices = plc_descriptor.num_devices

        # query firmware information
        if extended:
            info = plc_descriptor.get_info()
            self.plc_name = info['name']
            self.plc_version = info['version']
            self.plc_author = info['author'] or 'Anonymous Author'
            self.plc_description = info['description']

        self.plc_descriptor = plc_descriptor

    # lowlevel methods to query the indexer

    def query_infostruct(self, devnum):
        # fields in the infostruct: typecode, size, addr, unitcode, unitexp,
        # flags, absmin, absmax, the rest is the name
        result = self.query_data(
            devnum, v_2015_02.INFO_STRUCT, f'HHHBbIII{self.indexer_size - 22}s'
        )
        # convert min/max; these are floats, but we need to potentially word-
        # swap them
        absmin = self.io.float_from_dword(result[6])
        absmax = self.io.float_from_dword(result[7])
        # convert unit to a string
        unit = v_2015_02.convert_unit(*result[3:5])
        # only use the name if it has a trailing null byte, so we can be
        # sure it was fully transferred in the reduced space at the end
        name_parts = result[-1].partition(b'\0')
        name = name_parts[0].decode('latin1') if name_parts[1] else ''
        return result[:3] + (unit, result[5], absmin, absmax, name)

    def query_word(self, devnum, infotype):
        return self.query_data(devnum, infotype, 'H')[0]

    def query_unit(self, devnum, infotype):
        return v_2015_02.convert_unit(*self.query_data(devnum, infotype, 'Bb'))

    def query_string(self, devnum, infotype):
        result = self.query_bytes(devnum, infotype)
        return result.partition(b'\0')[0].decode('latin1')

    def query_bitmap(self, devnum, infotype):
        return [
            grp * 8 + bit
            for grp, byte in enumerate(self.query_bytes(devnum, infotype))
            for bit in range(8)
            if byte & (1 << bit)
        ]

    def query_bytes(self, devnum, infotype):
        return self.query_data(devnum, infotype, f'{self.indexer_size - 2}s')[0]

    # new for 2021_09

    def _descriptor_from_data(self, descriptorid, descriptor_type, data):
        try:
            descr_class = v_2021_09.DESCRIPTOR_TYPES[descriptor_type >> 8]
        except KeyError:
            raise SpecError('got unsupported descriptor_type '
                            f'{descriptor_type:#06x} for descriptor id '
                            f'{descriptorid:#06x}') from None
        return descr_class(descriptorid, descriptor_type, data)

    def query_descriptor(self, desc_id):
        if desc_id in self._descr_cache:
            wanted = self._descr_cache[desc_id]
            if not wanted.is_resolved:
                wanted.resolve(self)
            return wanted

        if desc_id == v_2021_09.DEBUG_Descriptor:
            return self.query_descriptor_uncached(v_2021_09.DEBUG_Descriptor)

        # query descriptors in full blocks of #slots
        datalen = self.indexer_slot_size - 2
        first_desc_id = (desc_id // self.indexer_num_slots) * \
            self.indexer_num_slots
        slot_data = self.query_data(first_desc_id, 0,
                                    self.indexer_num_slots * f'H{datalen}s')
        cur_desc_id = first_desc_id - 1
        while slot_data:
            desc_type, data = slot_data[:2]
            slot_data = slot_data[2:]
            cur_desc_id += 1

            if desc_type == 0:
                continue

            descr = self._descriptor_from_data(cur_desc_id, desc_type, data)
            self._descr_cache[cur_desc_id] = descr

        wanted = self._descr_cache[desc_id]
        wanted.resolve(self)
        return wanted

    def query_descriptor_uncached(self, desc_id):
        if desc_id == v_2021_09.DEBUG_Descriptor:
            datalen = self.indexer_slot_size * self.indexer_num_slots - 2
        else:
            datalen = self.indexer_slot_size - 2
        desc_type, data = self.query_data(desc_id, 0, f'H{datalen}s')
        if desc_type == 0:
            return None
        return self._descriptor_from_data(desc_id, desc_type, data + b'\0')

    # even lower level methods to do an indexer transaction

    def query_data(self, devnum, infotype, fmt):
        request = infotype << 8 | devnum
        self.io.write_u16s(self.indexer_addr, [request])
        for i in range(self.IO_RETRIES):
            reply = self.io.read_fmt(self.indexer_addr, 'H' + fmt)
            if reply[0] == request | 0x8000:
                return reply[1:]
            # indexer stuck at debug descriptor?
            if reply[0] == 0xFFFF and self.magicstr == '2021_09':
                return reply[1:]
            # refresh the query if it got overwritten by a different host
            if reply[0] & 0x7FFF != request:
                self.io.write_u16s(self.indexer_addr, [request])
            time.sleep(0.001 * i * i)
        raise CommError('indexer not responding in time')
