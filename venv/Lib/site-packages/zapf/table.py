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

"""Object for handling table data."""

import time

from zapf import ApiError, CommError, SpecError
from zapf.spec import ParamCMDs, v_2021_09
from zapf.spec.v_2021_09 import get_valueinfo


class Table:
    def __init__(self, io, dev, name, info):
        self.io = io
        self.dev = dev
        self.name = name
        self.info = info
        self._rows = info['last_row'] + 1
        self._cols = info['last_column'] + 1
        self._addr = info['idx']
        self._colinfo = {}
        self._last_row = None
        for col, column in enumerate(info['header']):
            self._colinfo[col] = self._colinfo[column['name']] = \
                (col, get_valueinfo(column, self.dev.value_size))

        if info['accesstype'] == v_2021_09.TABLE_ACCESS_PCTL:
            if not dev.pctrl_addr:
                raise SpecError('device has no paramctrl, yet a table '
                                f'{name!r} with that accesstype is declared')
        else:
            if dev.value_size < 4:
                raise SpecError('tables are not allowed on 16-bit devices')
        if info['accesstype'] == v_2021_09.TABLE_ACCESS_MEM_1D:
            io.register_cache_range(self._addr, 4 + self._rows*self.dev.value_size)
        elif info['accesstype'] == v_2021_09.TABLE_ACCESS_MEM_2D:
            io.register_cache_range(self._addr,
                                    self._rows * self._cols * self.dev.value_size)

    def get_size(self):
        """Return the table size as (rows, cols)."""
        return (self._rows, self._cols)

    def list_columns(self):
        """Read the table's columns names (in order)."""
        return [col['name'] for col in self.info['header']]

    def get_column_valueinfo(self, cname):
        return self._colinfo[cname][1]

    def get_cell_raw(self, row, cname):
        """Get the value of a given cell.

        ``cname`` is either the name of a column or a numeric index into the
        header list obtained from `.list_columns`.

        The type of the value depends on the column, it can be int or float.
        enums are not translated.
        """
        if row >= self._rows:
            raise ApiError(f'can\'t access row {row}, it must be < {self._rows}')
        if row < 0:
            raise ApiError(f'can\'t access row {row}, it must be >= 0')
        try:
            col, value_info = self._colinfo[cname]
        except KeyError:
            if isinstance(cname, str):
                raise ApiError(f'column {cname!r} does not exist') from None
            else:
                raise ApiError(f'can\'t access column {cname}, it must be '
                               f'< {self._cols}') from None
        fmt = value_info.fmt
        accesstype = self.info['accesstype']
        cell_number = self._cols * row + col
        if accesstype == v_2021_09.TABLE_ACCESS_PCTL:
            # pylint: disable=protected-access
            pctrl_state, pval = self.dev._get_param(self._addr + cell_number, fmt)
            if pctrl_state is ParamCMDs.DONE:
                return pval
            raise CommError(f'getting table {self.name!r} cell @ row={row} column={cname!r} '
                            f'failed with ParamCMD {ParamCMDs[pctrl_state]!r}')
        elif accesstype == v_2021_09.TABLE_ACCESS_MEM_1D:
            # select right row
            act_row_addr = self._addr
            req_row_addr = self._addr + 2
            addr = self._addr + 4 + self.dev.value_size * col
            tries = 10
            while self.io.read_u16(act_row_addr) != row:
                self.io.write_u16(req_row_addr, row)
                tries -= 1
                if not tries:
                    raise CommError(f'mailbox for table {self.name!r} not responding')
                time.sleep(0.001)
            return self.io.read_fmt(addr, fmt)[0]
        elif accesstype == v_2021_09.TABLE_ACCESS_MEM_2D:
            addr = self._addr + self.dev.value_size * cell_number
            return self.io.read_fmt(addr, fmt)[0]
        else:
            raise SpecError(f'unknown accesstype {accesstype!r}')

    def get_cell(self, row, cname):
        """Get the value of a given cell.

        Like `get_cell_raw`, but enums are translated.
        """
        value = self.get_cell_raw(row, cname)
        _, value_info = self._colinfo[cname]
        if value_info.enum_r:
            value = str(value_info.enum_r.get(value, value))
        return value

    def set_cell_raw(self, row, cname, value):
        """Set the given cell to a new value.

        Returns the read-back value of that cell.  The type of the value
        depends on the column name, it can be int or float, enums are not
        translated.

        If the table has no column with index or name ``cname``,
        `.ApiError` is raised.
        If the cell is readonly, `.ApiError` is raised.
        """
        if row >= self._rows:
            raise ApiError(f'can\'t access row {row}, it must be < {self._rows}')
        if row < 0:
            raise ApiError(f'can\'t access row {row}, it must be >= 0')
        try:
            col, value_info = self._colinfo[cname]
        except KeyError:
            if isinstance(cname, str):
                raise ApiError(f'column {cname!r} does not exist') from None
            else:
                raise ApiError(f'can\'t access column {cname}, it must be < '
                               f'{self._cols}') from None
        if value_info.readonly:
            raise ApiError(f'table {self.name!r} column {cname!r} is read-only')
        fmt = value_info.fmt
        accesstype = self.info['accesstype']
        cell_number = self._cols * row + col
        self._last_row = row
        if accesstype == v_2021_09.TABLE_ACCESS_PCTL:
            # pylint: disable=protected-access
            pctrl_state, pval = self.dev._set_param(self._addr + cell_number, fmt, value)
            if pctrl_state is ParamCMDs.DONE:
                return pval
            raise CommError(f'setting table {self.name!r} cell @ row={row} column={cname!r} '
                            f'failed with ParamCMD {ParamCMDs[pctrl_state]!r}')
        elif accesstype == v_2021_09.TABLE_ACCESS_MEM_1D:
            # select right row
            act_row_addr = self._addr
            req_row_addr = self._addr + 2
            addr = self._addr + 4 + self.dev.value_size * col

            for i in range(10):
                if self.io.read_u16(act_row_addr) == row:
                    break
                self.io.write_u16(req_row_addr, row)
                time.sleep(0.001 * i)
            else:
                raise CommError(f'mailbox for table {self.name!r} not responding')
            self.io.write_fmt(addr, fmt, value)
            return self.io.read_fmt(addr, fmt)[0]
        elif accesstype == v_2021_09.TABLE_ACCESS_MEM_2D:
            addr = self._addr + self.dev.value_size * cell_number
            self.io.write_fmt(addr, fmt, value)
            return self.io.read_fmt(addr, fmt)[0]
        else:
            raise SpecError(f'unknown accesstype {accesstype!r}')

    def set_cell(self, row, cname, value):
        """Set the given cell to a new value.

        Like `set_cell_raw`, but enums are translated and limits are checked.
        """
        try:
            col, value_info = self._colinfo[cname]
        except KeyError:
            if isinstance(cname, str):
                raise ApiError(f'column {cname!r} does not exist') from None
            else:
                raise ApiError(f'can\'t access column {cname}, it must be '
                               f'< {self._cols}') from None
        if isinstance(value, str):
            if value_info.enum_w and value not in value_info.enum_w:
                raise ApiError(f'value {value!r} is not one of the allowed '
                               f'enums (try one of {list(value_info.enum_w)})')
            if not value_info.enum_w:
                raise ApiError(f'string values {value!r} not allowed for cell')
            value = value_info.enum_w[value]
        elif (value < value_info.min_value or
              value > value_info.max_value):
            raise ApiError(f'{value} is not within cell limits '
                           f'{value_info.min_value}..'
                           f'{value_info.max_value}')
        self.set_cell_raw(row, col, value)

    # support >>>with table_obj as t: t.set_cell(...) syntax
    def __enter__(self):
        self._last_row = None
        return self

    def __exit__(self, _type, _value, _traceback):
        if self._last_row is not None:
            row = self._last_row
            # force unbuffer of last write by reading from another row
            self.get_cell_raw(row-1 if row else 1, 0)
            self._last_row = None
        return False
