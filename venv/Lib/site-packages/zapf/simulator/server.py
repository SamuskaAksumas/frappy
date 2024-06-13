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

"""A simple server to access a simulated PLC via Modbus/TCP or ADS."""

import socketserver
from struct import pack, unpack, unpack_from

from zapf.proto.ads import ADS_ADR_LEN, ADS_DEVINFO, ADS_READ, ADS_WRITE, \
    AMS_HEADER_SIZE, DEVINFO, HEADER


class ModbusExc(Exception):
    pass


class ModbusConnectionHandler(socketserver.BaseRequestHandler):

    def handle(self):
        sock = self.request
        while True:
            # read request
            req = sock.recv(8)
            if len(req) != 8:
                return
            tidpid, lgth, unit, func = unpack('>IHBB', req)
            data = sock.recv(lgth - 2)
            try:
                resp = self.handle_req(lgth, func, data)
            except ModbusExc as e:
                msg = pack('>IHBBB', tidpid, lgth, unit, func | 0x80,
                           e.args[0])
            except Exception as e:
                print('during Modbus request: ', e)
                # send as a slave device failure
                msg = pack('>IHBBB', tidpid, lgth, unit, func | 0x80, 4)
            else:
                msg = pack('>IHBB', tidpid, 2 + len(resp), unit, func) + resp
            sock.sendall(msg)

    def handle_req(self, lgth, func, data):
        # decode request
        if len(data) != lgth - 2:      # illegal data value
            raise ModbusExc(3)
        addr, = unpack_from('>H', data)
        addr = addr - 0x3000  # map from Beckhoff standard
        if addr < 0 or addr >= 0x1000:
            # illegal data address
            raise ModbusExc(2)
        baddr = 2*addr + 0x10000  # map to byte address
        with self.server.cond:
            self.server.cond.wait()  # wait for go ahead for one round
            if func in (3, 4):
                # read data
                nreg, = unpack('>H', data[2:])
                if nreg > 125:
                    raise ModbusExc(3)  # illegal data value
                read = self.server.mem.read(baddr, 2*nreg)
                return pack('>B', 2*nreg) + \
                    b''.join(pack('>H', *unpack('H', read[2*i:2*i+2]))
                             for i in range(nreg))
            elif func == 6:
                wdata = pack('H', *unpack('>H', data[2:4]))
                self.server.mem.write(baddr, wdata)
                return data
            elif func == 16:
                nreg, dbytes = unpack_from('>HB', data[2:])
                assert dbytes == 2*nreg
                wdata = b''.join(pack('H', *unpack('>H', data[2*i+5:2*i+7]))
                                 for i in range(nreg))
                self.server.mem.write(baddr, wdata)
                return data[:4]
            raise ModbusExc(1)  # illegal function


class ModbusServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, mem, cond, port):
        self.mem = mem
        self.cond = cond
        socketserver.ThreadingTCPServer.__init__(self, ('localhost', port),
                                                 ModbusConnectionHandler)


class AdsExc(Exception):
    pass


class AdsConnectionHandler(socketserver.BaseRequestHandler):

    def handle(self):
        sock = self.request
        while True:
            # read header
            req = sock.recv(HEADER.size)
            if len(req) != HEADER.size:
                return
            _, length, dest, source, cmd, flags, data_length, _, invoke_id = \
                HEADER.unpack(req)
            data = sock.recv(data_length)
            try:
                if flags != 4:
                    raise AdsExc(0x1C)  # invalid fragment
                if length != 32 + data_length:
                    raise AdsExc(0xE)  # invalid length
                ret_data = self.handle_req(cmd, data)
                err = 0
            except AdsExc as e:
                ret_data = b''
                err = e.args[0]
            except Exception as e:
                print('during ADS request: ', e)
                ret_data = b''
                err = 0x1  # internal error
            response = HEADER.pack(
                0, AMS_HEADER_SIZE + 4 + len(ret_data), source, dest, cmd, 5,
                4 + len(ret_data), err, invoke_id) + pack('<I', err) + ret_data
            sock.sendall(response)

    def handle_req(self, cmd, data):
        if cmd == ADS_DEVINFO:
            return DEVINFO.pack(1, 2, 731, b'Zapf sim')
        elif cmd == ADS_READ:
            if len(data) != 12:
                raise AdsExc(0xE)  # invalid length
            igroup, ioffset, length = ADS_ADR_LEN.unpack(data)
            ioffset += 0x10000
            if igroup != 0x4020:
                raise AdsExc(0x702)  # invalid index group
            with self.server.cond:
                self.server.cond.wait()
                return data[8:12] + self.server.mem.read(ioffset, length)
        elif cmd == ADS_WRITE:
            if len(data) < 12:
                raise AdsExc(0xE)
            igroup, ioffset, length = ADS_ADR_LEN.unpack(data[:12])
            ioffset += 0x10000
            if igroup != 0x4020:
                raise AdsExc(0x702)  # invalid index group
            with self.server.cond:
                self.server.cond.wait()
                self.server.mem.write(ioffset, data[12:])
                return b''
        else:
            raise AdsExc(0xB)  # invalid command


class AdsServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, mem, cond, port):
        self.mem = mem
        self.cond = cond
        socketserver.ThreadingTCPServer.__init__(self, ('localhost', port),
                                                 AdsConnectionHandler)
