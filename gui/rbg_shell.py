#!/usr/bin/env python3

"""
Shell class for Robot Bicycle GUI.


Oliver Lee (oliverzlee@gmail.com)
5 December 2012
"""

import serial
import time

from PyQt4 import QtGui, QtCore


ST_KEYWORD = 'status' # string keyword denoting system state
ST_DELIM = ','


class RbState(object):
    """Methods match with struct definition in Sample.h although steer methods
    have been renamed yr (yawrate).
    """
    def __init__(self):
        super(RbState, self).__init__()
        self.init = False
        self.s = ''
        
    def set(self, s):
        self.init = True
        self.s = s
        self.flags, self._rw_ref, self._yr_ref, self._fn = s.split(ST_DELIM)[:]
        
    def __str__(self):
        return self.s
    
    def rw_en(self):
        return bool(int(self.flags) & 0x0001)
        
    def yr_en(self):
        return bool(int(self.flags) & 0x0002)
        
    def rw_fa(self):
        return bool(int(self.flags) & 0x0004)
        
    def yr_fa(self):
        return bool(int(self.flags) & 0x0008)
        
    def rw_di(self):
        return bool(int(self.flags) & 0x0010)
        
    def yr_di(self):
        return bool(int(self.flags) & 0x0020)
        
    def fw_di(self):
        return bool(int(self.flags) & 0x0040)
        
    def rwc_di(self):
        return bool(int(self.flags) & 0x0080)
        
    def yrc_di(self):
        return bool(int(self.flags) & 0x0100)
        
    def fs_wt(self):
        return bool(int(self.flags) & 0x0200)
        
    def col_en(self):
        return bool(int(self.flags) & 0x8000)

    def rw_ref(self):
        return float(self._rw_ref)
        
    def yr_ref(self):
        return float(self._yr_ref)
        
    def fn(self):
        return self._fn

class RbgShell(QtCore.QObject):
    bytes_received = QtCore.pyqtSignal(str)
    connection_failed = QtCore.pyqtSignal(str)
    state_updated = QtCore.pyqtSignal(RbState)

    def __init__(self, state):
        super(RbgShell, self).__init__()
        self.ser = serial.Serial()
        self.ser.baudrate = 115200
        self.ser.port = None
        self.ser.bytesize = 8
        self.ser.parity = 'N'
        self.ser.stopbits = 1
        self.state = state

    def connect(self, port):
        try:
            self.ser.port = port
            if not self.ser.isOpen():
                self.ser.open()
        except serial.serialutil.SerialException as e:
            self.connection_failed.emit(str(e))

    def disconnect(self):
        self.ser.close()

    def update_console(self):
        recbuf = ''
        while(self.ser.isOpen()):
            bytes_rec = self.ser.inWaiting()
            if bytes_rec:
                rec = self.ser.read(bytes_rec)
                try:
                    recstr = rec.decode('UTF-8')
                    self.bytes_received.emit(recstr)
                    recbuf += recstr
                except UnicodeDecodeError as e:
                    try:
                        recstr = rec.decode('cp1252')
                        self.bytes_received.emit(recstr)
                        recbuf += recstr
                    except Exception as e:
                        raise(e)
                if not self.receiving_state(recbuf):
#                    self.bytes_received.emit(recbuf)
                    recbuf = ''
            time.sleep(0.5)
#            rec = self.ser.read()
#            try:
#                self.bytes_received.emit(rec.decode('UTF-8'))
#            except UnicodeDecodeError as e:
#                print(e, rec)


    def receiving_state(self, s):
        def receiving_state_keyword(s):
            for i in range(len(ST_KEYWORD)):
                sub = ST_KEYWORD[0:i+1]
                if s.endswith(sub):
                    return True
            return False
        
        idx = s.rfind(ST_KEYWORD)
        if idx < 0 and not receiving_state_keyword(s):
            # not receiving state, send text to console
            return False
        
        # receiving state
        nls = '\n'
        nl0 = s.find(nls, idx)
        nl1 = s.find(nls, nl0 + 1)
    
        if nl0 >= 0 and nl1 >= 0: # state is contained in text
            if nl0 >= nl1: # error with received text
                raise Exception('StateDecodeError')
                return False
            statestr = s[nl0+1:nl1]
            # state has been received and updated so send text to console
            self.state.set(statestr)
            self.state_updated.emit(self.state)
            return False
        
        # still waiting for rest of state to be received
        return True


    def write(self, command):
        success = False
        try:
            self.ser.write(bytes('{0}{1}'.format(command, '\r\n'),
                                 'UTF-8')) #TODO 
#            time.sleep(0.2)
#            rb = self.ser.inWaiting()
#            print(rb)
#            if rb:
#                ret = self.ser.read(rb)
#                self.bytes_received.emit(ret.decode('UTF-8'))
#            ret = self.ser.readline()
#            self.bytes_received.emit(ret.decode('UTF-8'))
#            res = ser.read()
#            if res == 0:
#                success = True
#            print('{0}: {1}'.format(self.ser.port, command))

#            print('{0}: {1}'.format(self.port, command))
#            success = True
        except ValueError as e:
#            print('Error: {0}'.format(str(e)))
#            self.connection_failed.emit(str(e))
            raise e
        return success

    def close(self):
        ser.close()
