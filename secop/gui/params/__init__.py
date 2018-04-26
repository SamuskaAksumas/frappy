#  -*- coding: utf-8 -*-
# *****************************************************************************
# Copyright (c) 2015-2016 by the authors, see LICENSE
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

try:
    # py2
    unicode(u'')
except NameError:
    unicode = str  # pylint: disable=redefined-builtin

from secop.gui.qt import QWidget, QLabel, QPushButton as QButton, QLineEdit, \
    QMessageBox, QCheckBox, QSizePolicy, Qt, pyqtSignal, pyqtSlot

from secop.gui.util import loadUi
from secop.datatypes import EnumType


class ParameterWidget(QWidget):
    setRequested = pyqtSignal(str, str, object)  # module, parameter, target
    cmdRequested = pyqtSignal(str, str, list)  # module, command, args

    def __init__(self,
                 module,
                 paramcmd,
                 datatype=None,
                 initvalue=None,
                 readonly=True,
                 parent=None):
        super(ParameterWidget, self).__init__(parent)
        self._module = module
        self._paramcmd = paramcmd
        self._datatype = datatype
        self._readonly = readonly

        self._load_ui(initvalue)

    def _load_ui(self, initvalue):
        # load ui file, set initvalue to right widget
        pass

    def updateValue(self, value):
        # async !
        pass


class GenericParameterWidget(ParameterWidget):

    def _load_ui(self, initvalue):
        # using two QLineEdits for current and target value
        loadUi(self, 'parambuttons.ui')

        if self._readonly:
            self.setPushButton.setEnabled(False)
            self.setLineEdit.setEnabled(False)
        else:
            self.setLineEdit.returnPressed.connect(
                self.on_setPushButton_clicked)
        self.updateValue(initvalue)

    @pyqtSlot()
    def on_setPushButton_clicked(self):
        self.setRequested.emit(self._module, self._paramcmd,
                               self.setLineEdit.text())

    def updateValue(self, value):
        if self._datatype:
            value = self._datatype.import_value(value)
        self.currentLineEdit.setText(str(value))


class EnumParameterWidget(GenericParameterWidget):

    def _load_ui(self, initvalue):
        # using two QLineEdits for current and target value
        loadUi(self, 'parambuttons_select.ui')

        # transfer allowed settings from datatype to comboBoxes
        self._map = {}  # maps index to enumstring
        self._revmap = {}  # maps enumstring to index
        index = 0
        for enumval, enumname in sorted(self._datatype.entries.items()):
            self.setComboBox.addItem(enumname, enumval)
            self._map[index] = (enumval, enumname)
            self._revmap[enumname] = index
            self._revmap[enumval] = index
            index += 1
        if self._readonly:
            self.setLabel.setEnabled(False)
            self.setComboBox.setEnabled(False)
            self.setLabel.setHidden(True)
            self.setComboBox.setHidden(True)
        else:
            self.setComboBox.activated.connect(self.on_setPushButton_clicked)

        self.updateValue(initvalue)

    @pyqtSlot()
    def on_setPushButton_clicked(self):
        _enumval, enumname = self._map[self.setComboBox.currentIndex()]
        self.setRequested.emit(self._module, self._paramcmd, enumname)

    def updateValue(self, value):
        try:
            value = int(value)
        except ValueError:
            pass
        if value in self._revmap:
            index = self._revmap[value]
            self.currentLineEdit.setText('(%d): %s' % self._map[index])
        else:
            self.currentLineEdit.setText('undefined Value: %r' % value)


class GenericCmdWidget(ParameterWidget):

    def _load_ui(self, initvalue):
        # using two QLineEdits for current and target value
        loadUi(self, 'cmdbuttons.ui')

        self.cmdLineEdit.setText('')
        self.cmdLineEdit.setEnabled(self.datatype.argtypes is not None)
        self.cmdLineEdit.returnPressed.connect(
            self.on_cmdPushButton_clicked)

    @pyqtSlot()
    def on_cmdPushButton_clicked(self):
        # wait until command complete before retrying
        # since the command is scheduled async: what if an errot happens?
        # XXX: button stays deactivated upon errors in execution of cmd...
        self.cmdPushButton.setEnabled(False)
        self.cmdRequested.emit(
            self._module,
            self._paramcmd,
            self._datatype.from_string(
                self.cmdLineEdit.text()))

    def updateValue(self, value):
        # open dialog and show value, if any.
        # then re-activate the command button
        self.cmdPushButton.setEnabled(True)


def ParameterView(module,
                  paramcmd,
                  datatype=None,
                  initvalue=None,
                  readonly=True,
                  parent=None):
    # depending on datatype returns an initialized widget fit for display and
    # interaction

    if datatype is not None:
        if datatype.IS_COMMAND:
            return GenericCmdWidget(
                module,
                paramcmd,  # name of command
                datatype,
                initvalue,  # not used for comands
                readonly,  # not used for commands
                parent)
        if isinstance(datatype, EnumType):
            return EnumParameterWidget(
                module,
                paramcmd,  # name of parameter
                datatype,
                initvalue,
                readonly,
                parent)

    return GenericParameterWidget(
        module,
        paramcmd,  # name of parameter
        datatype,
        initvalue,
        readonly,
        parent)
