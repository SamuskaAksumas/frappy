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
#   Enrico Faulhaber <enrico.faulhaber@frm2.tum.de>
#   Alexander Lenz <alexander.lenz@frm2.tum.de>
#   Markus Zolliker <markus.zolliker@psi.ch>
#
# *****************************************************************************
"""Define helpers"""

import os
import signal
import sys
from collections import OrderedDict

from frappy.config import load_config
from frappy.errors import ConfigError
from frappy.dynamic import Pinata
from frappy.lib import formatException, generalConfig, get_class, mkthread
from frappy.lib.multievent import MultiEvent
from frappy.params import PREDEFINED_ACCESSIBLES

try:
    from daemon import DaemonContext
    try:
        from daemon import pidlockfile
    except ImportError:
        import daemon.pidfile as pidlockfile
except ImportError:
    DaemonContext = None

try:
    # pylint: disable=unused-import
    import systemd.daemon
except ImportError:
    systemd = None


class Server:
    INTERFACES = {
        'tcp': 'protocol.interface.tcp.TCPServer',
    }
    _restart = True

    def __init__(self, name, parent_logger, cfgfiles=None, interface=None, testonly=False):
        """initialize server

        Arguments:
        - name:  the node name
        - parent_logger: the logger to inherit from
        - cfgfiles: if not given, defaults to name
            may be a comma separated list of cfg files
            items ending with .cfg are taken as paths, else .cfg is appended and
            files are looked up in the config path retrieved from the general config
        - interface: an uri of the from tcp://<port> or a bare port number for tcp
            if not given, the interface is taken from the config file. In case of
            multiple cfg files, the interface is taken from the first cfg file
        - testonly: test mode. tries to build all modules, but the server is not started

        Format of cfg file (for now, both forms are accepted):
        old form:                  new form:

        [node <equipment id>]      [NODE]
        description=<descr>        id=<equipment id>
                                   description=<descr>

        [interface tcp]            [INTERFACE]
        bindport=10769             uri=tcp://10769
        bindto=0.0.0.0

        [module temp]              [temp]
        ramp=12                    ramp=12
        ...
        """
        self._testonly = testonly

        if not cfgfiles:
            cfgfiles = name
        # sanitize name (in case it is a cfgfile)
        name = os.path.splitext(os.path.basename(name))[0]
        self.log = parent_logger.getChild(name, True)

        merged_cfg = load_config(cfgfiles, self.log)
        self.node_cfg = merged_cfg.pop('node')
        self.module_cfg = merged_cfg
        if interface:
            self.node_cfg['equipment_id'] = name
            self.node_cfg['interface'] = str(interface)
        elif not self.node_cfg.get('interface'):
            raise ConfigError('No interface specified in configuration or arguments!')

        self._cfgfiles = cfgfiles
        self._pidfile = os.path.join(generalConfig.piddir, name + '.pid')
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

    def signal_handler(self, _num, _frame):
        if hasattr(self, 'interface') and self.interface:
            self.shutdown()

    def start(self):
        if not DaemonContext:
            raise ConfigError('can not daemonize, as python-daemon is not installed')
        piddir = os.path.dirname(self._pidfile)
        if not os.path.isdir(piddir):
            os.makedirs(piddir)
        pidfile = pidlockfile.TimeoutPIDLockFile(self._pidfile)

        if pidfile.is_locked():
            self.log.error('Pidfile already exists. Exiting')

        with DaemonContext(
                pidfile=pidfile,
                files_preserve=self.log.getLogfileStreams()):
            self.run()

    def unknown_options(self, cls, options):
        return f"{cls.__name__} class don't know how to handle option(s): {', '.join(options)}"

    def restart_hook(self):
        """Actions to be done on restart. May be overridden by a subclass."""

    def run(self):
        global systemd  # pylint: disable=global-statement
        while self._restart:
            self._restart = False
            try:
                # TODO: make systemd notifications configurable
                if systemd:
                    systemd.daemon.notify("STATUS=initializing")
            except Exception:
                systemd = None
            try:
                self._processCfg()
                if self._testonly:
                    return
            except Exception:
                print(formatException(verbose=True))
                raise

            opts = {'uri': self.node_cfg['interface']}
            scheme, _, _ = opts['uri'].rpartition('://')
            scheme = scheme or 'tcp'
            cls = get_class(self.INTERFACES[scheme])
            with cls(scheme, self.log.getChild(scheme), opts, self) as self.interface:
                if opts:
                    raise ConfigError(self.unknown_options(cls, opts))
                self.log.info('startup done, handling transport messages')
                if systemd:
                    systemd.daemon.notify("READY=1\nSTATUS=accepting requests")
                t = mkthread(self.interface.serve_forever)
                # we wait here on the thread finishing, which means we got a
                # signal to shut down or an exception was raised
                # TODO: get the exception (and re-raise?)
                t.join()
                self.interface = None  # fine due to the semantics of 'with'
                # server_close() called by 'with'

            self.log.info(f'stopped listenning, cleaning up'
                          f' {len(self.modules)} modules')
            # if systemd:
            #     if self._restart:
            #         systemd.daemon.notify('RELOADING=1')
            #     else:
            #         systemd.daemon.notify('STOPPING=1')
            for name in self._getSortedModules():
                self.modules[name].shutdownModule()
            if self._restart:
                self.restart_hook()
                self.log.info('restarting')
        self.log.info('shut down')

    def restart(self):
        if not self._restart:
            self._restart = True
            self.interface.shutdown()

    def shutdown(self):
        self._restart = False
        self.interface.shutdown()

    def _processCfg(self):
        """Processes the module configuration.

        All modules specified in the config file and read recursively from
        Pinata class Modules are instantiated, initialized and started by the
        end of this function.
        If there are errors that occur, they will be collected and emitted
        together in the end.
        """
        errors = []
        opts = dict(self.node_cfg)
        cls = get_class(opts.pop('cls'))
        self.dispatcher = cls(opts.pop('name', self._cfgfiles),
                              self.log.getChild('dispatcher'), opts, self)

        if opts:
            self.dispatcher.errors.append(self.unknown_options(cls, opts))
        self.modules = OrderedDict()

        # create and initialize modules
        todos = list(self.module_cfg.items())
        while todos:
            modname, options = todos.pop(0)
            if modname in self.modules:
                # already created by Dispatcher (via Attached)
                continue
            # For Pinata modules: we need to access this in Dispatcher.get_module
            self.module_cfg[modname] = dict(options)
            modobj = self.dispatcher.get_module_instance(modname) # lazy
            if modobj is None:
                self.log.debug('Module %s returned None', modname)
                continue
            self.modules[modname] = modobj
            if isinstance(modobj, Pinata):
                # scan for dynamic devices
                pinata = self.dispatcher.get_module(modname)
                pinata_modules = list(pinata.scanModules())
                for name, _cfg in pinata_modules:
                    if name in self.module_cfg:
                        self.log.error('Module %s, from pinata %s, already'
                                       ' exists in config file!', name, modname)
                self.log.info('Pinata %s found %d modules', modname, len(pinata_modules))
                todos.extend(pinata_modules)

        # initialize all modules by getting them with Dispatcher.get_module,
        # which is done in the get_descriptive data
        # TODO: caching, to not make this extra work
        self.dispatcher.get_descriptive_data('')
        # =========== All modules are initialized ===========

        # all errors from initialization process
        errors = self.dispatcher.errors

        if not self._testonly:
            start_events = MultiEvent(default_timeout=30)
            for modname, modobj in self.modules.items():
                # startModule must return either a timeout value or None (default 30 sec)
                start_events.name = f'module {modname}'
                modobj.startModule(start_events)
                if not modobj.startModuleDone:
                    errors.append(f'{modobj.startModule.__qualname__} was not called, probably missing super call')

        if errors:
            for errtxt in errors:
                for line in errtxt.split('\n'):
                    self.log.error(line)
            # print a list of config errors to stderr
            sys.stderr.write('\n'.join(errors))
            sys.stderr.write('\n')
            sys.exit(1)

        if self._testonly:
            return
        self.log.info('waiting for modules being started')
        start_events.name = None
        if not start_events.wait():
            # some timeout happened
            for name in start_events.waiting_for():
                self.log.warning('timeout when starting %s', name)
        self.log.info('all modules started')
        history_path = os.environ.get('FRAPPY_HISTORY')
        if history_path:
            from frappy_psi.historywriter import FrappyHistoryWriter  # pylint: disable=import-outside-toplevel
            writer = FrappyHistoryWriter(history_path, PREDEFINED_ACCESSIBLES.keys(), self.dispatcher)
            # treat writer as a connection
            self.dispatcher.add_connection(writer)
            writer.init(self.dispatcher.handle_describe(writer, None, None))
        # TODO: if ever somebody wants to implement an other history writer:
        # - a general config file /etc/secp/frappy.conf or <frappy repo>/etc/frappy.conf
        #   might be introduced, which contains the log, pid and cfg directory path and
        #   the class path implementing the history
        # - or we just add here an other if statement:
        #   history_path = os.environ.get('ALTERNATIVE_HISTORY')
        #   if history_path:
        #       from frappy_<xx>.historywriter import ... etc.

    def _getSortedModules(self):
        """Sort modules topologically by inverse dependency.

        Example: if there is an IO device A and module B depends on it, then
        the result will be [B, A].
        Right now, if the dependency graph is not a DAG, we give up and return
        the unvisited nodes to be dismantled at the end.
        Taken from Introduction to Algorithms [CLRS].
        """
        def go(name):
            if name in done:  # visiting a node
                return True
            if name in visited:
                visited.add(name)
                return False  # cycle in dependencies -> fail
            visited.add(name)
            if name in unmarked:
                unmarked.remove(name)
            for module in self.modules[name].attachedModules.values():
                res = go(module.name)
                if not res:
                    return False
            visited.remove(name)
            done.add(name)
            l.append(name)
            return True

        unmarked = set(self.modules.keys())  # unvisited nodes
        visited = set()  # visited in DFS, but not completed
        done = set()
        l = []  # list of sorted modules

        while unmarked:
            if not go(unmarked.pop()):
                self.log.error('cyclical dependency between modules!')
                return l[::-1] + list(visited) + list(unmarked)
        return l[::-1]
