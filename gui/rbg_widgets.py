#!/usr/bin/env python3

"""
Robot Bicycle GUI widgets.

Widgets for common commands in controlling the robot bicycle.


Oliver Lee (oliverzlee@gmail.com)
5 December 2012
"""


import abc
import threading

from rbg_shell import RbState as St

from PyQt4 import QtGui, QtCore


SPINBOX_RESET_VAL = 0.00


class ToggleError(Exception):
    def __init__(self, message):
        Exception.__init__(self, message)


class RbgToggleButton(QtGui.QPushButton):
    status_updated = QtCore.pyqtSignal(str)

    def __init__(self, parent):
        super(RbgToggleButton, self).__init__(parent)
        self.setCheckable(True)
        self.clicked.connect(self.button_clicked)
        self.on_text = ''
        self.off_text = ''
        self.on_action = None
        self.off_action = None
        self.on_status = ''
        self.off_status = ''
        
    def button_clicked(self, checked):
        prev_state = not checked
        try:
            if checked:
                self.click_on()
            else:
                self.click_off()
        except (ToggleError, AttributeError, ValueError) as e:
            self.status_updated.emit(str(e))
            self.setChecked(prev_state) # not toggled
            
    def set_on_state(self, text, action, status):
        self.on_text = text
        self.on_action = action
        self.on_status = status
        
    def set_off_state(self, text, action, status):
        self.off_text = text
        self.off_action = action
        self.off_status = status
        
    def set_on(self):
        self.setChecked(True)
        self.setText(self.on_text)

    def set_off(self):
        self.setChecked(False)
        self.setText(self.off_text)
        
    def reset(self):
        self.set_off()
        
    def click_on(self):
        self.on_action()
        self.set_on()
        self.status_updated.emit(self.on_status)
        
    def click_off(self):
        self.off_action()
        self.set_off()
        self.status_updated.emit(self.off_status)


class RbgPushButton(RbgToggleButton):
    def set_state(self, text, action, status):
        self.set_on_state(text, action, status)
        self.set_off_state(text, action, status)
    
    def set_on(self):
        self.set_off()


class RbgBaseWidget(QtGui.QWidget):
    __metaclass__ = abc.ABCMeta
    status_updated = QtCore.pyqtSignal(str)
    reseted = QtCore.pyqtSignal()
    state_updated = QtCore.pyqtSignal()
    
    def __init__(self, parent, shell):
        """Base class for all widgets.
        """
        super(RbgBaseWidget, self).__init__(parent)
        self.shell = shell
#        self.setSizePolicy(QtGui.QSizePolicy.Preferred, 
#                           QtGui.QSizePolicy.Maximum)
        
    def reset(self):
        self.reseted.emit()
        
    @abc.abstractmethod
    def update_state(self, state):
        return
        
    def sync(self, widget):
        """Synchronize reset and toggled_failed in both widgets."""
#        rbgwidget.status_updated.connect(lambda s: self.status_updated.emit(s))
        widget.status_updated.connect(self.update_status)
        self.reseted.connect(widget.reset)

    def update_status(self, message):
        self.status_updated.emit(message)

class RbgControlWidget(RbgBaseWidget):
    def __init__(self, name, command, parent, shell):
        super(RbgControlWidget, self).__init__(parent, shell)
        self.name = name
        self.command = command

        self.valb = QtGui.QDoubleSpinBox(self)
        self.controlb = RbgToggleButton(self)
        
        self.valb.setDecimals(2)
        self.valb.setRange(-99.99, 99.99)
        self.valb.setSingleStep(0.01)
        self.valb.valueChanged.connect(self.change_control)
        
        self.controlb.set_on_state(
            'Stop {0} Control'.format(self.name), 
            lambda: self.shell.write(self.start_command()), 
            self.start_command())
        self.controlb.set_off_state(
            'Start {0} Control'.format(self.name), 
            lambda: self.shell.write(self.stop_command()), 
            self.stop_command())
#        self.controlb.status_updated(lambda s: self.status_updated.emit(s))
        self.controlb.set_off()
        self.sync(self.controlb)
        self.init_ui()

    def init_ui(self):
        self.setSizePolicy(QtGui.QSizePolicy.Expanding, 
                           QtGui.QSizePolicy.Preferred)
        
        hbox = QtGui.QHBoxLayout()
        hbox.addWidget(self.valb)
        hbox.addWidget(self.controlb)
        
        vbox = QtGui.QVBoxLayout()
        vbox.addStretch(1)
        vbox.addLayout(hbox)
        vbox.addStretch(1)
        
        self.setLayout(vbox)

    def reset(self):
        super(RbgControlWidget, self).reset()
        self.valb.setValue(SPINBOX_RESET_VAL)
        
    def update_state(self, state):
        self.valb.setValue(getattr(state,'{0}_ref'.format(self.command))())
        if(getattr(state, '{0}_en'.format(self.command))()):
            self.controlb.set_on()
        else:
            self.controlb.set_off()

    def start_command(self):
#        return '{0} {1:+06.2f}'.format(self.command,self.valb.value())
        """Uses the same command to toggle control on and off.
        """
        return self.stop_command()
    
    def stop_command(self):
        return '{0}'.format(self.command)
        
    def change_control(self, x):
#        if self.controlb.isChecked():
        sh_command = '{0} {1:+06.2f}'.format(self.command, x)
        self.shell.write(sh_command)
        self.status_updated.emit(sh_command)


class RbgDisableMotors(RbgBaseWidget):
    def __init__(self, parent, shell):
        super(RbgDisableMotors, self).__init__(parent, shell)
#        self.motorb = RbgToggleButton(self)
#        self.motorb.set_on_state(
#            'Enable Motors', 
#            lambda: self.shell.write(self.disable_command()), 
#            self.disable_command())
#        self.motorb.set_off_state(
#            'Disable Motors', 
#            lambda: self.shell.write(self.enable_command()), 
#            self.enable_command())
        self.motorb = RbgPushButton(self)
        self.motorb.set_state('Disable Motors',
                              lambda: self.shell.write('disable'),
                              'disable')
        self.motorb.set_off()
        self.sync(self.motorb)
        self.init_ui()

    def init_ui(self):
        self.motorb.setSizePolicy(QtGui.QSizePolicy.Expanding, 
                                  QtGui.QSizePolicy.Expanding)
        self.setSizePolicy(QtGui.QSizePolicy.Expanding, 
                           QtGui.QSizePolicy.Expanding)
        
        hbox = QtGui.QHBoxLayout()
        hbox.addWidget(self.motorb)
        
        vbox = QtGui.QVBoxLayout()
        vbox.addLayout(hbox)
        
        self.setLayout(vbox)
        
    def update_state(self, state):
        #:TODO
        return

#    def disable_command(self):
#        return 'disable'

#    def enable_command(self):
#        return 'disable'


class RbgResetState(RbgBaseWidget):
    def __init__(self, parent, shell):
        super(RbgResetState, self).__init__(parent, shell)
        self.resetb = RbgPushButton(self)
        self.resetb.set_state('Reset',
                              lambda: self.shell.write('reset'), 
                              'reset')
        self.resetb.set_off()
        self.sync(self.resetb)
        self.init_ui()

    def init_ui(self):
        self.resetb.setSizePolicy(QtGui.QSizePolicy.Expanding, 
                                  QtGui.QSizePolicy.Expanding)
        
        hbox = QtGui.QHBoxLayout()
        hbox.addWidget(self.resetb)
        
        vbox = QtGui.QVBoxLayout()
        vbox.addLayout(hbox)
        
        self.setLayout(vbox)
    
    def update_state(self, state):
        return


class RbgRefreshState(RbgBaseWidget):
    def __init__(self, parent, shell):
        super(RbgRefreshState, self).__init__(parent, shell)
        self.refreshb = RbgPushButton(self)
        self.refreshb.set_state('Refresh State',
                                lambda: self.shell.write('status'), 
                                'status')
        self.refreshb.set_off()
        self.sync(self.refreshb)
        self.init_ui()

    def init_ui(self):
        self.refreshb.setSizePolicy(QtGui.QSizePolicy.Expanding, 
                                    QtGui.QSizePolicy.Expanding)
        hbox = QtGui.QHBoxLayout()
        hbox.addWidget(self.refreshb)
        
        vbox = QtGui.QVBoxLayout()
        vbox.addLayout(hbox)
        
        self.setLayout(vbox)
        
    def update_state(self, state):
        return
        
    def refresh(self):
        self.refreshb.on_action()
        

class RbgHomeFork(RbgBaseWidget):
    def __init__(self, parent, shell):
        super(RbgHomeFork, self).__init__(parent, shell)
        self.pushb = RbgPushButton(self)
        self.pushb.set_state('Home Fork',
                             lambda: self.shell.write('homefork'),
                             'homefork')
        self.pushb.set_off()
        self.sync(self.pushb)
        self.init_ui()
        
    def init_ui(self):
        self.pushb.setSizePolicy(QtGui.QSizePolicy.Expanding, 
                                 QtGui.QSizePolicy.Expanding)
        hbox = QtGui.QHBoxLayout()
        hbox.addWidget(self.pushb)
        
        vbox = QtGui.QVBoxLayout()
        vbox.addLayout(hbox)
        
        self.setLayout(vbox)
    
    def update_state(self, state):
        return

class RbgShellWidget(RbgBaseWidget):
    def __init__(self, parent, shell):
        super(RbgShellWidget, self).__init__(parent, shell)
        self.te = QtGui.QTextEdit(self)
        self.te.setReadOnly(True)
        self.te.document().setMaximumBlockCount(100)
        self.le = QtGui.QLineEdit(self)
        self.le.returnPressed.connect(self.send)
        self.init_ui()

    def init_ui(self):
        vbox = QtGui.QVBoxLayout()
        vbox.addWidget(self.te)
        vbox.addWidget(self.le)
        self.setLayout(vbox)

    def update_te(self, text):
        self.te.moveCursor(QtGui.QTextCursor.End)
        self.te.insertPlainText(text)
        self.te.moveCursor(QtGui.QTextCursor.End)
    
    def send(self):
        self.shell.write(self.le.text())
        self.le.setText('')

class RbgConnectPort(RbgBaseWidget):
    connected = QtCore.pyqtSignal()
    disconnected = QtCore.pyqtSignal()
    
    def __init__(self, parent, shell, port=None):
        super(RbgConnectPort, self).__init__(parent, shell)
        self.port = port
        
        self.selectb = QtGui.QPushButton('...', self)
        self.selectb.clicked.connect(self.select_dialog)
        
        self.portle = QtGui.QLineEdit(self)
        self.portle.setPlaceholderText('Select Port')
        
        self.connectb = RbgToggleButton(self)
        self.connectb.set_on_state('Disconnect', 
                                   self.connect_port, 
                                   'Connected to {0}'.format(self.port))
        self.connectb.set_off_state('Connect',
                                    self.disconnect_port,
                                    'Disconnected from {0}'.format(self.port))
        self.connectb.reset = lambda: None # Do not disconnect on reset
        self.connectb.set_off()
        self.sync(self.connectb)
        self.init_ui(port)
        
    def init_ui(self, port=None):
        self.selectb.setMaximumWidth(30)
        if port is not None:
            self.portle.setText(str(port))

        hbox = QtGui.QHBoxLayout()
        hbox.addWidget(self.selectb)
        hbox.addWidget(self.portle)
        hbox.addWidget(self.connectb)
        
        vbox = QtGui.QVBoxLayout()
        vbox.addStretch(1)
        vbox.addLayout(hbox)
        vbox.addStretch(1)
        
        self.setLayout(vbox)
    
    def update_state(self, state):
        return
        
    def select_dialog(self):
        text = QtGui.QFileDialog.getOpenFileName(self, 'Select Port', '/')
        self.portle.setText(str(text))
            
    def connect_port(self):
        self.port = str(self.portle.text())
        self.shell.connect(self.port)
        self.portle.setReadOnly(True)
#        thread.start_new_thread(self.shell.update_console, ())
        self.t = threading.Thread(target=self.shell.update_console, 
                                  name='update_console')
        self.t.start()
        self.connected.emit()
    
    def disconnect_port(self):
        self.shell.disconnect()
        self.portle.setReadOnly(False)
        self.disconnected.emit()


class RbgCollectData(RbgBaseWidget):
    def __init__(self, parent, shell):
        super(RbgCollectData, self).__init__(parent, shell)

        self.fnamele = QtGui.QLineEdit(self)
        self.fnamele.setPlaceholderText('filename')
        self.fnamele.setText('samples.dat')
        
        self.recb = RbgToggleButton(self)
        self.recb.set_on_state('Stop Recording Data',
                               self.start_recording,
                               self.start_command())
        self.recb.set_off_state('Start Recording Data',
                                self.stop_recording,
                                self.stop_command())
        self.recb.set_off()
        self.sync(self.recb)
        self.init_ui()
        
    def init_ui(self):
        hbox = QtGui.QHBoxLayout()
        hbox.addWidget(self.fnamele)
        hbox.addWidget(self.recb)
        
        vbox = QtGui.QVBoxLayout()
        vbox.addStretch(1)
        vbox.addLayout(hbox)
        vbox.addStretch(1)
        
        self.setLayout(vbox)
        
    def update_state(self, state):
        self.fnamele.setText(state.fn())
        if state.col_en():
            self.recb.set_on()
        else:
            self.recb.set_off()
        
    def start_command(self):
        return 'collect {0}'.format(self.fnamele.text())
#        return 'collect'
    
    def start_recording(self):
        text = self.fnamele.text()
        if text:
            self.shell.write(self.start_command())
            self.fnamele.setReadOnly(True)
        else:
            raise ToggleError("Invalid filename")
#        self.shell.write(self.start_command())
        self.fnamele.setReadOnly(True)
    
    def stop_command(self):
        return 'collect'
    
    def stop_recording(self):
        self.shell.write(self.stop_command())
        self.fnamele.setReadOnly(False)


class RbgDisturbance(RbgBaseWidget):
    def __init__(self, parent, shell):
        super(RbgDisturbance, self).__init__(parent, shell)
        
        self.Ab = QtGui.QDoubleSpinBox(self)
        self.Ab.setPrefix('A: ')
        self.Ab.setDecimals(2)
        self.Ab.setRange(00.00, 99.99)
        self.Ab.setSingleStep(0.01)
        self.Ab.valueChanged.connect(self.change_disturb)
        
        self.omegab = QtGui.QDoubleSpinBox(self)
        self.omegab.setPrefix('omega: ')
        self.omegab.setDecimals(2)
        self.omegab.setRange(00.00, 99.99)
        self.omegab.setSingleStep(0.01)
        self.omegab.valueChanged.connect(self.change_disturb)

        self.disturbb = RbgToggleButton(self)
        self.disturbb.set_on_state('Stop Disturbance', 
                                   self.start_disturb, 
                                   self.start_command())
        self.disturbb.set_off_state('Start Disturbance', 
                                    self.stop_disturb, 
                                    self.stop_command())
        self.disturbb.set_off()
        self.sync(self.disturbb)
        self.init_ui()
        
    def init_ui(self):
        hbox = QtGui.QHBoxLayout()
        hbox.addWidget(self.Ab)
        hbox.addWidget(self.omegab)
        hbox.addWidget(self.disturbb)
        
        vbox = QtGui.QVBoxLayout()
        vbox.addStretch(1)
        vbox.addLayout(hbox)
        vbox.addStretch(1)
        
        self.setLayout(vbox)

    def reset(self):
        super(RbgDisturbance, self).reset()
        self.Ab.setValue(SPINBOX_RESET_VAL)
        self.omegab.setValue(SPINBOX_RESET_VAL)
        
    def update_state(self):
        #TODO:
        return

    def start_command(self):
        return 'initiate_disturbance {0} {1}'.format(self.Ab.value(), 
                                                     self.omegab.value())
        
    def start_disturb(self):
        if self.Ab.value() != 0.0 and self.omegab.value() != 0.0:
            self.shell.write(self.start_command())
        else:
            raise ToggleError("Set A and omega to nonzero values")
     
    def stop_command(self):
        return 'terminate_disturbance'
     
    def stop_disturb(self):
        self.shell.write(self.stop_command())
        
    def change_disturb(self, d):
        if self.disturbb.isChecked():
            sh_command = 'initiate_disturbance {0} {1}'.format(
                self.Ab.value(), self.omegab.value())
            self.shell.write(sh_command)
            self.status_updated.emit(sh_command)



