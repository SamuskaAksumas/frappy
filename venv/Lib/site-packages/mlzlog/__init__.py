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
#   Georg Brandl <g.brandl@fz-juelich.de>
#   Alexander Lenz <alexander.lenz@frm2.tum.de>
#
# *****************************************************************************

import os
from os import path
import sys
import time
import linecache
import traceback
from logging import Logger, Formatter, Handler, DEBUG, INFO, WARNING, ERROR, \
    setLoggerClass

from mlzlog.colors import colorize

LOGFMT = '%(asctime)s : %(levelname)-7s : %(name)-15s: %(message)s'
DATEFMT = '%H:%M:%S'
DATESTAMP_FMT = '%Y-%m-%d'
SECONDS_PER_DAY = 60 * 60 * 24

LOGLEVELS = {'debug': DEBUG, 'info': INFO, 'warning': WARNING, 'error': ERROR}
INVLOGLEVELS = {value: key for key, value in LOGLEVELS.items()}


# Is set to the root logger of the MLZLogger hierarchy in initLogging().
log = None


def initLogging(rootname='mlz', rootlevel='info', logdir='/tmp/log', colorize=colorize):
    global log
    setLoggerClass(MLZLogger)
    log = MLZLogger(rootname)
    log.setLevel(LOGLEVELS[rootlevel])

    # console logging for fg process
    log.addHandler(ColoredConsoleHandler(colorize=colorize))

    if logdir is not None:
        # logfile for fg and bg process
        log.addHandler(LogfileHandler(logdir, rootname))


def getLogger(name, subdir=False):
    return log.getChild(name, subdir)


class MLZLogger(Logger):
    maxLogNameLength = 0

    def __init__(self, *args, **kwargs):
        Logger.__init__(self, *args, **kwargs)
        self._storeLoggerNameLength()

    def getChild(self, suffix, ownDir=False):  # pylint: disable=arguments-differ
        child = Logger.getChild(self, suffix)
        child.setLevel(self.getEffectiveLevel())

        for handler in self._collectHandlers():
            if ownDir and isinstance(handler, LogfileHandler):
                handler = handler.getChild(suffix)
            child.addHandler(handler)

        child.propagate = False
        return child

    def getLogfileStreams(self):
        result = []
        for entry in self._collectHandlers():
            if isinstance(entry, LogfileHandler):
                result.append(entry.stream)
        return result

    def _collectHandlers(self):
        result = []

        log = self
        while log is not None:
            result.extend(log.handlers)
            log = log.parent

        return result

    def _storeLoggerNameLength(self):
        """Store max logger name length for formatting."""
        MLZLogger.maxLogNameLength = max(len(self.name),
                                         MLZLogger.maxLogNameLength)


class ConsoleFormatter(Formatter):
    """
    A lightweight formatter for the interactive console, with optional
    colored output.
    """

    def __init__(self, fmt=None, datefmt=None, colorize=None):
        Formatter.__init__(self, fmt, datefmt)
        if colorize:
            self.colorize = colorize
        else:
            self.colorize = lambda c, s: s

    def formatException(self, exc_info):  # pylint: disable=arguments-differ
        return traceback.format_exception_only(*exc_info[0:2])[-1]

    def formatTime(self, record, datefmt=None):
        return time.strftime(datefmt or DATEFMT,
                             self.converter(record.created))

    def format(self, record):
        record.message = record.getMessage()
        levelno = record.levelno
        datefmt = self.colorize('lightgray', '[%(asctime)s] ')
        namefmt = '%(name)-' + str(MLZLogger.maxLogNameLength) + 's: '
        if levelno <= DEBUG:
            fmtstr = self.colorize('darkgray', '%s%%(message)s' % namefmt)
        elif levelno <= INFO:
            fmtstr = '%s%%(message)s' % namefmt
        elif levelno <= WARNING:
            fmtstr = self.colorize('fuchsia', '%s%%(levelname)s: %%(message)s'
                                   % namefmt)
        else:
            # Add exception type to error (if caused by exception)
            msgPrefix = ''
            if record.exc_info:
                msgPrefix = '%s: ' % record.exc_info[0].__name__

            fmtstr = self.colorize('red', '%s%%(levelname)s: %s%%(message)s'
                                   % (namefmt, msgPrefix))
        fmtstr = datefmt + fmtstr
        if not getattr(record, 'nonl', False):
            fmtstr += '\n'
        record.asctime = self.formatTime(record, self.datefmt)
        s = fmtstr % record.__dict__
        # never output more exception info -- the exception message is already
        # part of the log message because of our special logger behavior
        # if record.exc_info:
        #    # *not* caching exception text on the record, since it's
        #    # only a short version
        #    s += self.formatException(record.exc_info)
        return s


def formatExtendedFrame(frame):
    ret = []
    for key, value in frame.f_locals.items():
        try:
            valstr = repr(value)[:256]
        except Exception:
            valstr = '<cannot be displayed>'
        ret.append('        %-20s = %s\n' % (key, valstr))
    ret.append('\n')
    return ret


def formatExtendedTraceback(etype, value, tb):
    ret = ['Traceback (most recent call last):\n']
    while tb is not None:
        frame = tb.tb_frame
        filename = frame.f_code.co_filename
        item = '  File "%s", line %d, in %s\n' % (filename, tb.tb_lineno,
                                                  frame.f_code.co_name)
        linecache.checkcache(filename)
        line = linecache.getline(filename, tb.tb_lineno, frame.f_globals)
        if line:
            item = item + '    %s\n' % line.strip()
        ret.append(item)
        if filename != '<script>':
            ret += formatExtendedFrame(tb.tb_frame)
        tb = tb.tb_next
    ret += traceback.format_exception_only(etype, value)
    return ''.join(ret).rstrip('\n')


class LogfileFormatter(Formatter):
    """
    The standard Formatter does not support milliseconds with an explicit
    datestamp format.  It also doesn't show the full traceback for exceptions.
    """

    extended_traceback = True

    def formatException(self, ei):
        if self.extended_traceback:
            s = formatExtendedTraceback(*ei)
        else:
            s = ''.join(traceback.format_exception(ei[0], ei[1], ei[2],
                                                   sys.maxsize))
            if s.endswith('\n'):
                s = s[:-1]
        return s

    def formatTime(self, record, datefmt=None):
        res = time.strftime(DATEFMT, self.converter(record.created))
        res += ',%03d' % record.msecs
        return res


class StreamHandler(Handler):
    """Reimplemented from logging: remove cruft, remove bare excepts."""

    def __init__(self, stream=None):
        Handler.__init__(self)
        self.stream = stream
        self.disabled = False

    def flush(self):
        self.acquire()
        try:
            if self.stream and hasattr(self.stream, 'flush'):
                self.stream.flush()
        finally:
            self.release()

    def filter(self, record):
        return not self.disabled

    def enable(self, enabled):
        if enabled:
            self.disabled = False
            self.stream.close()
            self.stream = self._open()
        else:
            self.disabled = True

    def emit(self, record):
        try:
            msg = self.format(record)
            try:
                self.stream.write('%s\n' % msg)
            except UnicodeEncodeError:
                self.stream.write('%s\n' % msg.encode('utf-8'))
            self.flush()
        except Exception:
            self.handleError(record)

    def close_stream(self):
        self.acquire()
        try:
            if self.stream:
                self.flush()
                if hasattr(self.stream, 'close'):
                    self.stream.close()
                self.stream = None
        finally:
            self.release()

    def close(self):
        self.close_stream()
        Handler.close(self)


class SimpleLogfileHandler(StreamHandler):
    """
    Logs to log files without rollover.
    """

    def __init__(self, directory, filename):
        if not path.isdir(directory):
            os.makedirs(directory)
        basefn = path.join(directory, filename + '.log')
        self.baseFilename = path.abspath(basefn)
        self.mode = 'a'
        StreamHandler.__init__(self)
        self.setFormatter(LogfileFormatter(LOGFMT, DATEFMT))

    def _open(self):
        return open(self.baseFilename, self.mode)

    def emit(self, record):
        try:
            if self.stream is None:
                self.stream = self._open()
            StreamHandler.emit(self, record)
        except Exception:
            self.handleError(record)


class LogfileHandler(StreamHandler):
    """
    Logs to log files with a date stamp appended, and rollover on midnight.
    """

    def __init__(self, directory, filenameprefix, dayfmt=DATESTAMP_FMT):
        self._directory = path.join(directory, filenameprefix)
        if not path.isdir(self._directory):
            os.makedirs(self._directory)
        if not os.access(self._directory, os.R_OK | os.W_OK):
            raise IOError('Cannot write to logfile path: %r' % self._directory)
        self._currentsymlink = path.join(self._directory, 'current')
        self._filenameprefix = filenameprefix
        self._pathnameprefix = path.join(self._directory, filenameprefix)
        self._dayfmt = dayfmt
        # today's logfile name
        basefn = self._pathnameprefix + '-' + time.strftime(dayfmt) + '.log'
        self.baseFilename = path.abspath(basefn)
        self.mode = 'a'
        StreamHandler.__init__(self)
        # determine time of first midnight from now on
        t = time.localtime()
        self.rollover_at = time.mktime((t[0], t[1], t[2], 0, 0, 0,
                                        t[6], t[7], t[8])) + SECONDS_PER_DAY
        self.setFormatter(LogfileFormatter(LOGFMT, DATEFMT))

    def _open(self):
        # update 'current' symlink upon open
        try:
            os.remove(self._currentsymlink)
        except OSError:
            # if the symlink does not (yet) exist, OSError is raised.
            # should happen at most once per installation....
            pass
        try:
            os.symlink(path.basename(self.baseFilename), self._currentsymlink)
        except (AttributeError, OSError):
            pass
        # finally open the new logfile....
        return open(self.baseFilename, self.mode)

    def getChild(self, name):
        return LogfileHandler(self._directory, name)

    def emit(self, record):
        try:
            t = int(time.time())
            if t >= self.rollover_at:
                self.doRollover()
            if self.stream is None:
                self.stream = self._open()
            StreamHandler.emit(self, record)
        except Exception:
            self.handleError(record)

    def doRollover(self):
        self.stream.close()
        self.baseFilename = self._pathnameprefix + '-' + \
            time.strftime(self._dayfmt) + '.log'
        self.stream = self._open()
        self.rollover_at += SECONDS_PER_DAY


class ColoredConsoleHandler(StreamHandler):
    """
    A handler class that writes colorized records to standard output.
    """

    def __init__(self, colorize=colorize):
        StreamHandler.__init__(self, sys.stdout)
        self.setFormatter(ConsoleFormatter(datefmt=DATEFMT, colorize=colorize))

    def emit(self, record):
        msg = self.format(record)
        try:
            self.stream.write(msg)
        except UnicodeEncodeError:
            self.stream.write(msg.encode('utf-8'))
        self.stream.flush()
