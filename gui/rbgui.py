#!/usr/bin/env python3

"""
Robot Bicycle GUI.

Barebones GUI allows user to run common shell commands with buttons.


Oliver Lee (oliverzlee@gmail.com)
5 December 2012
"""

import sys

from PyQt4 import QtGui, QtCore

from rbg_shell import *
from rbg_widgets import *
from rbg_button_page import ButtonPage
from rbg_plot_page import PlotPage


class MainWindow(QtGui.QMainWindow):
    reseted = QtCore.pyqtSignal()
    state_updated = QtCore.pyqtSignal(RbState)
    
    def __init__(self):
        super(MainWindow, self).__init__()
        tabs = QtGui.QTabWidget()
        self.statusBar()

        button_page = ButtonPage()
        button_page.status_updated.connect(self.update_status)
        self.shell = button_page.shell
        tabs.addTab(button_page, "Data Collection and Control")

        plot_page = PlotPage()
        tabs.addTab(plot_page, "Data Plots")
        
        self.cw = tabs
        self.init_ui()
        
    def init_ui(self):
        self.setCentralWidget(self.cw)

        self.resize(1000, 600)
        qr = self.frameGeometry() #geometry of the main window
        #center point of monitor
        cp = QtGui.QDesktopWidget().availableGeometry().center() 
        qr.moveCenter(cp)
        self.move(qr.topLeft())
        
        self.setWindowTitle('BEEP BEEP BLOOP BLARP')
        self.show()

    def update_status(self, text):
        self.statusBar().showMessage(text)
        
        
def main():
    app = QtGui.QApplication(sys.argv)
    ex = MainWindow()
    app.aboutToQuit.connect(ex.shell.disconnect)
    sys.exit(app.exec_())
    
    
if __name__ == '__main__':
    main()
