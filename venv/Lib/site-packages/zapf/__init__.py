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

"""A client library for the  PILS specification for SCADA-PLC interfaces."""


class Error(Exception):
    """Base for Zapf errors."""


class CommError(Error):
    """Raised for communication errors with the PLC.

    For example, loss of socket connection.
    """


class ApiError(Error):
    """Raised for invalid usage of the Zapf API, or the PLC devices.

    For example, trying to write to a read-only device.
    """


class SpecError(Error):
    """Raised for invalid PLCs, or invalid data from the PLC.

    For example, a PLC defining an indexer that is too small, or using an
    unsupported version of PILS.
    """


class DescError(SpecError):
    """Raised when reading and validating descriptors."""
    def __init__(self, desc, reason):
        super().__init__(f'{desc}: {reason}')
