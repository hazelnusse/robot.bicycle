"""
Setup for Robot Bicycle GUI Button Page.

Oliver Lee (oliverzlee@gmail.com)
15 Feb 2013
"""

import time

from PyQt4 import QtCore, QtGui

from rbg_shell import *
from rbg_widgets import *

class ButtonPage(QtGui.QWidget):
    reseted = QtCore.pyqtSignal()
    state_updated = QtCore.pyqtSignal(RbState)
    status_updated = QtCore.pyqtSignal(str)
    
    def __init__(self):
        super(ButtonPage, self).__init__()
        self.state = RbState()
        self.shell = RbgShell(self.state)
        self.consolew = RbgShellWidget(self, self.shell)
        
        self.portw = RbgConnectPort(self, self.shell, '/dev/ttyUSB0')
        self.motorw = RbgDisableMotors(self, self.shell)
        self.resetw = RbgResetState(self, self.shell)
        self.refreshw = RbgRefreshState(self, self.shell)
        self.homew = RbgHomeFork(self, self.shell)
        self.recw = RbgCollectData(self, self.shell)
        self.yrw = RbgControlWidget('Yaw Rate', 'yr', self, self.shell)
        self.speedw = RbgControlWidget('Speed', 'rw', self, self.shell)
        self.disturbw = RbgDisturbance(self, self.shell)
        
        self.shell.connection_failed.connect(lambda s: self.status_updated.emit(s))
        self.shell.bytes_received.connect(self.consolew.update_te)
        self.shell.state_updated.connect(lambda s: self.state_updated.emit(s))
        
        self.sync(self.portw)
        self.sync(self.motorw)
        self.sync(self.resetw)
        self.sync(self.refreshw)
        self.sync(self.homew)
        self.sync(self.recw)
        self.sync(self.yrw)
        self.sync(self.speedw)
        self.sync(self.disturbw)
        
        self.resetw.resetb.clicked.connect(lambda: self.reseted.emit())
        self.recw.recb.clicked.connect(lambda: time.sleep(0.5) or 
                                               self.refreshw.refresh())
        self.portw.connected.connect(self.refreshw.refresh)
        
        self.init_ui()
        
    def init_ui(self):
        hbox00 = QtGui.QHBoxLayout()
        hbox0 = QtGui.QHBoxLayout()
        hbox1 = QtGui.QHBoxLayout()
        
        vbox = QtGui.QVBoxLayout()
        vbox00 = QtGui.QVBoxLayout()
        vbox01 = QtGui.QVBoxLayout()
        vbox10 = QtGui.QVBoxLayout()
        vbox11 = QtGui.QVBoxLayout()
        
        vbox00.addWidget(self.motorw)
        hbox00.addWidget(self.resetw)
        hbox00.addWidget(self.refreshw)
        hbox00.addWidget(self.homew)
        vbox00.addLayout(hbox00)
        vbox00.addWidget(self.portw)
        
        vbox01.addWidget(self.consolew)
        
        hbox0.addLayout(vbox00)
        hbox0.addLayout(vbox01)
        
        vbox10.addWidget(self.recw)
        vbox10.addWidget(self.disturbw)
        
        vbox11.addWidget(self.speedw)
        vbox11.addWidget(self.yrw)
        
        hbox1.addLayout(vbox10)
        hbox1.addLayout(vbox11)
        
        vbox.addLayout(hbox0)
        vbox.addLayout(hbox1)
        
        self.setLayout(vbox)
        
    def sync(self, widget):
        """Sync status updates and reset methods between page widget and other widgets.
        """
        widget.status_updated.connect(lambda s: self.status_updated.emit(s))
        self.reseted.connect(widget.reset)
        self.state_updated.connect(widget.update_state)



