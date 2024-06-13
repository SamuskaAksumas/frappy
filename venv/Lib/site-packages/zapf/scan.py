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

"""Support for scanning the indexer and creating devices."""

from collections import namedtuple

from zapf import SpecError, spec
from zapf.device import TYPECODE_MAP, Device, FMT_LIMITS
from zapf.io import PlcIO
from zapf.spec import FMT_TO_BASETYPE, v_2015_02
from zapf.spec.v_2021_09 import fix_unit

DeviceInfo = namedtuple('DeviceInfo', 'name addr typecode info')


class Scanner:
    """
    :param io_or_proto: A connection URI string, a `.PlcIO` instance or
        a `.Protocol` instance.
    :param log: The `logging.Logger` instance to use for this object and
        all derived objects (e.g. devices).

    **Connection URIs** can have the following forms:

    ``ads://host[:port]/amsnetid:amsport``
        Connection to a Beckhoff PLC using the ADS protocol.  The TCP port is
        48898 by default.  The AMS NetID and AMS port are specific to the PLC.
        Note that an AMS router entry must be set on the PLC in order to
        connect.

        Example: ``ads://192.168.201.2/5.18.77.4.1.1:851``

    ``modbus://host[:port]/slaveno``
        Connection to a host that supports the Modbus/TCP protocol.  The TCP
        port is 502 by default.

        Example: ``modbus://192.168.201.2/0``

    ``tango://dbhost:dbport/tango/device/name``
        Connection to a `Tango <https://tango-controls.org>`_ device which in
        turn connects to the PLC.

        The Tango device interface must conform to the `Profibus
        <https://forge.frm2.tum.de/entangle/defs/entangle-master/profibus/>`_
        Entangle interface specification.

        Example: ``tango://192.168.201.2:10000/box/plc/ads``

    ``sim://filepath``
        "Connection" to a software-simulated PLC.  Zapf starts it in the same
        process when the address is requested.  See the `zapf.simulator`
        package for details.
    """

    IGNORE_BAD_DEVICES = True

    def __init__(self, io_or_proto, log):
        self.log = log
        if isinstance(io_or_proto, PlcIO):
            self.io = io_or_proto
        else:
            self.io = PlcIO(io_or_proto, log)

    def get_plc_data(self):
        """Establish communication with the indexer and return an object
        containing the metadata.

        The following attributes are defined on this object:

        ``magicstr``
            The PLC :ref:`magic number <pils:magic>`, converted to a string,
            that identifies the PILS protocol revision.  Example: `'2015_02'`.
        ``indexer_addr``
            The byte address of the indexer data structure, relative to the
            common data area.
        ``indexer_size``
            The size of the indexer in bytes.
        ``num_devices``
            The number of devices present.  This information is provided
            optionally by PILS, and can be 0.  In this case, the number of
            devices is found by finding the first device number where the
            indexer returns empty information.
        ``plc_name``
            The "PLC name" string as reported by the PLC.
        ``plc_version``
            The "PLC version" string as reported by the PLC.
        ``plc_author``
            The "PLC author" string as reported by the PLC.

        Can raise `.SpecError` if the PLC does not conform to a supported PILS
        version, or returns invalid information.
        """
        self.io.indexer.detect_plc()
        return self.io.indexer

    def scan_devices(self):
        """Scan the PLC and yield information about devices.

        For each device found in the PLC, this will yield a namedtuple with the
        following items:

        * ``number``: device number (1 to N)
        * ``name``: device name
        * ``addr``: device byte address
        * ``typecode``: device :ref:`type code <pils:type-codes>`
        * ``info``: additional information: a dictionary, see below

        Additional information contains at least the following keys:

        * ``lowlevel``: bool, if the device is marked "low level"
        * ``unit``: string, the unit of the main device value
        * ``absmin`` and ``absmax``: float, the limits of the main value
        * ``params``: a list of parameter names if the device has params
        * ``funcs``: a list of special function names if the device has such
        * ``aux``: a specification of bitfields/flags containing the interpretation
          of the up to 24 freely definable bits in the device status.
          Strings that are not defined are empty.

        Upon further extension of PILS, additional keys can appear.
        """

        # ensure we can talk to the indexer
        self.io.indexer.detect_plc()

        # check which method we need to scan
        method = getattr(self, '_scan_' + self.io.indexer.magicstr, None)
        if not method:
            raise RuntimeError(f'Magic {self.io.indexer.magicstr} is '
                               'supported, but no scanner method available')

        yield from method()

    def get_device(self, devinfo):
        """Return a device object for the given devinfo, as yielded by
        `scan_devices`.

        This will return a `.Device` object of the correct subclass for the
        device's typecode, or ``None`` if the typecode is not supported.
        """
        devcls = Device.class_for(devinfo.typecode)
        if devcls is None:
            self.log.warning('type code %#x is not supported, skipping',
                             devinfo.typecode)
            return None
        else:
            return devcls(devinfo.name, devinfo.addr, devinfo.typecode,
                          devinfo.info, self.io, self.log)

    def get_devices(self):
        """Scan the PLC and yield device objects.

        Short form of calling `get_device` for each device returned by
        `scan_devices`.
        """
        for data in self.scan_devices():
            yield self.get_device(data)

    def _scan_2015_02(self):
        indexer = self.io.indexer
        next_addr = indexer.indexer_addr + indexer.indexer_size

        for devnum in range(1, 256):
            info = indexer.query_infostruct(devnum)
            typecode, size, addr, unit, flags, absmin, absmax, name = info

            # gone past last device?
            if typecode == 0:
                break

            # if there is no valid data in the infostruct, query individually
            if size + addr + flags == 0:
                size = indexer.query_word(devnum, spec.v_2015_02.INFO_SIZE)
                if not size:
                    size = 2 * (typecode & 0xFF)
                addr = indexer.query_word(devnum, spec.v_2015_02.INFO_ADDR)
                if not addr:
                    addr = next_addr
                unit = indexer.query_unit(devnum, spec.v_2015_02.INFO_UNIT)
                flags = None
                lowlevel = False
                absmin = -spec.FLOAT32_MAX
                absmax = spec.FLOAT32_MAX
            else:
                lowlevel = (flags & 0x80000000) != 0

            # this might be empty even if a valid infostruct was present,
            # e.g. if the full name didn't fit behind the 20 previous bytes
            if not name:
                name = indexer.query_string(devnum, spec.v_2015_02.INFO_NAME)

            # extract info about parameters and special functions
            devclass = typecode >> 13
            parameters = []
            functions = []
            if devclass in (1, 2):
                if devclass == 1:  # FlatDevices: a list of indices
                    plist = indexer.query_bytes(devnum, spec.v_2015_02.INFO_PARAMS)
                    param_ids = [p for p in plist if p]
                else:  # ParamDevices: a bitmap of indices
                    param_ids = indexer.query_bitmap(devnum,
                                                     spec.v_2015_02.INFO_PARAMS)
                param_ids.sort()  # ... which *should* be unnecessary
                for p in param_ids:
                    if spec.v_2015_02.is_function(p):
                        functions.append(p)
                    else:
                        parameters.append(p)

            # extract AUX string labels
            aux_strings = [''] * 24
            if flags is None:  # old way, stop reading the first empty one
                for idx in range(24):
                    aux_string = indexer.query_string(
                        devnum, spec.v_2015_02.INFO_AUX1 + idx
                    )
                    aux_strings[idx] = aux_string.strip()
                    if not aux_string:
                        break
            else:  # new way, only read the relevant (flagged ones)
                for idx in range(24):
                    if flags & (1 << idx):
                        aux_string = indexer.query_string(
                            devnum, spec.v_2015_02.INFO_AUX1 + idx
                        )
                        aux_strings[idx] = aux_string.strip()

            # XXX: convert to new style! (see _scan_2021_09)
            flags = ['lowlevel'] if lowlevel else []
            aux = []
            for i, s in enumerate(aux_strings):
                if s:
                    # convert to Flag, bitfields dont exist in 2015_02
                    aux.append((i, s))

            try:
                tce = TYPECODE_MAP[typecode]
            except KeyError:
                raise SpecError(f'device {name!r} has unsupported typecode '
                                f'{typecode:#06x}') from None

            params = {}  # derive from parameters
            funcs = {}  # derive from functions
            if tce.num_params:
                if tce.has_pctrl:
                    # convert params and funcs
                    for p in parameters:
                        par = {'idx': p}
                        par.update(v_2015_02.PARAMETER_Specs[p])
                        if 'unit' in par:
                            par['unit'] = fix_unit(par.get('unit', ''), unit)
                        params[par.pop('name')] = par
                    for p in functions:
                        par = {'idx': p}
                        par.update(v_2015_02.FUNC_Specs[p])
                        if par.get('argument', None):
                            arg = {}
                            arg.update(par['argument'])
                            if 'unit' in arg:
                                arg['unit'] = fix_unit(arg.get('unit', ''), unit)
                            par['argument'] = arg
                        funcs[par.pop('name')] = par
                else:
                    # convert flat params
                    for i, p in enumerate(parameters):
                        par = {'idx': i}  # parameter slot id
                        par.update(v_2015_02.PARAMETER_Specs[p])
                        if 'unit' in par:
                            par['unit'] = fix_unit(par.get('unit', ''), unit)
                        params[par.pop('name')] = par

            basetype, width = FMT_TO_BASETYPE.get(tce.value_fmt, (None, 0))
            access = 'ro' if tce.readonly else 'rw'

            # fixup limits, if needed
            limits = FMT_LIMITS.get(tce.value_fmt)
            if limits:
                absmin = max(absmin, limits[0])
                absmax = min(absmax, limits[1])

            info = dict(unit=unit,
                        absmin=absmin, absmax=absmax,
                        params=params, funcs=funcs,
                        aux=aux, tables={}, flags=flags,
                        errid=[(0, 16, 'errid', {})],
                        description='',
                        access=access,
                        basetype=basetype,
                        width=width,
                        enum_r=None,
                        enum_w=None,
                        )
            self.log.info('found device %s at addr %s with type %#x',
                          name, addr, typecode)
            yield DeviceInfo(name, addr, typecode, info)

            next_addr = addr + size

    def _scan_2021_09(self):
        devid = self.io.indexer.plc_descriptor.last_device
        seen_dev_names = set()

        while devid:
            try:
                dev = self.io.indexer.query_descriptor(devid)
            except SpecError as err:
                self.log.error(err)
                # try to re-read content without resolving
                dev = self.io.indexer.query_descriptor_uncached(devid)
                devid = dev.prev
                continue

            if dev.name in seen_dev_names:
                self.log.debug(f'skipping device {dev.name!r}: a device with '
                               'that name already exists.')
                devid = dev.prev
                continue
            seen_dev_names.add(dev.name)

            if dev.typecode not in TYPECODE_MAP:
                self.log.error(f'device {dev.name!r} at address {dev.address:#06x} '
                               f'has unsupported typecode {dev.typecode:#06x}')
            try:
                info = dev.get_info()
            except SpecError as err:
                self.log.error(f'device {dev.name!r}: {err}')
                devid = dev.prev
                if self.IGNORE_BAD_DEVICES:
                    continue
                raise SpecError(f'Bad Device {dev.name!r}: {err}') from err

            self.log.info(f'found device {dev.name!r} at addr '
                          f'{dev.address:#06x} with type {dev.typecode:#06x}')
            yield DeviceInfo(info.pop('name'), info.pop('address'),
                             info.pop('typecode'), info)
            devid = dev.prev
