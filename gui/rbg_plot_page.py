"""
Setup for Robot Bicycle GUI Plot Page.

Oliver Lee (oliverzlee@gmail.com)
18 Feb 2013
"""

import os
import sys
sys.path.append(os.path.join(os.getcwd(), "..", "common"))

import math
import sip
from PyQt4 import QtCore, QtGui
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg \
         as FigureCanvas
from matplotlib.backends.backend_qt4agg import NavigationToolbar2QTAgg \
         as NavigationToolbar
import sampleplotter as sp


class PlotPage(QtGui.QWidget):
    def __init__(self, parent=None):
        super(PlotPage, self).__init__()
        self.filewidget = PlotFileWidget(self.create_plots)
        self.vbox = QtGui.QVBoxLayout()
        self.vbox.addWidget(self.filewidget)
        self.setLayout(self.vbox)

    def create_plots(self):
        try:
            self.plotter
            text = self.filewidget.text()
            self.plotter.load_file(text)
        except AttributeError:
            self.layout_setup()
        self.draw_plots()

    def layout_setup(self):
        text = self.filewidget.text()
        if text:
            self.plotter = sp.Samples(text)
            self.figs = []
            fig1 = FigureWidget(self.plotter.rearwheel.figure)
            fig2 = FigureWidget(self.plotter.accelerometer.figure)
            fig3 = FigureWidget(self.plotter.gyroscope.figure)
            fig4 = FigureWidget(self.plotter.temperature.figure)
            fig5 = FigureWidget(self.plotter.time.figure)
            fig6 = FigureWidget(self.plotter.steer.figure)
            #fig7 = FigureWidget(self.plotter.state.figure)
            self.figs.append(fig1)
            self.figs.append(fig2)
            self.figs.append(fig3)
            self.figs.append(fig4)
            self.figs.append(fig5)
            self.figs.append(fig6)
            #self.figs.append(fig7)

            #self.clear_layout()
            hbox1 = QtGui.QHBoxLayout()
            hbox1.addWidget(fig1)
            hbox1.addWidget(fig2)
            hbox2 = QtGui.QHBoxLayout()
            hbox2.addWidget(fig3)
            hbox2.addWidget(fig4)
            hbox3 = QtGui.QHBoxLayout()
            hbox3.addWidget(fig5)
            hbox3.addWidget(fig6)
            #hbox3.addWidget(fig7)
            self.vbox.addLayout(hbox1)
            self.vbox.addLayout(hbox2)
            self.vbox.addLayout(hbox3)

    def draw_plots(self):
        self.plotter.draw_plots()
        for f in self.figs:
            f.canvas.draw()

    def clear_layout(self):
        if self.layout() is not None:
            layout = self.layout()
            for i in reversed(range(layout.count())):
                try:
                    layout.itemAt(i).widget().setParent(None)
                except AttributeError:
                    pass
            sip.delete(layout)

class PlotWidget(QtGui.QWidget):
    def __init__(self, samples, plottype):
        super(PlotWidget, self).__init__()

        self.dpi = 100
        #self.fig = Figure((5.0, 4.0), dpi=self.dpi)
        f, ax = getattr(samples, plottype)()
        self.fig = f
        self.canvas = FigureCanvas(self.fig)
        self.canvas.setParent(self)

        self.axes = self.fig.add_subplot(111)
        self.mpl_toolbar = NavigationToolbar(self.canvas, self)

        vbox = QtGui.QVBoxLayout()
        vbox.addWidget(self.canvas)
#vbox.addWidget(self.mpl_toolbar)
        self.setLayout(vbox)


class FigureWidget(QtGui.QWidget):
    def __init__(self, figure):
        super(FigureWidget, self).__init__()
        self.fig = figure
        self.canvas = FigureCanvas(self.fig)
        self.canvas.setParent(self)
        #self.axes = self.fig.add_subplot(111) # ?? NEEDED ??
        self.mpl_toolbar = NavigationToolbar(self.canvas, self)
        vbox = QtGui.QVBoxLayout()
        vbox.addWidget(self.canvas)
        self.setLayout(vbox)


class PlotFileWidget(QtGui.QWidget):
    def __init__(self, plot_action, parent=None):
        super(PlotFileWidget, self).__init__()
        self.load_button = QtGui.QPushButton("Load File", self)
        self.plot_button = QtGui.QPushButton("Plot", self)
        self.filename_edit = QtGui.QLineEdit(self)
        self.filename_edit.setPlaceholderText('datafile')
        self.load_button.clicked.connect(self.set_load_action)
        self.plot_button.clicked.connect(plot_action)

        hbox = QtGui.QHBoxLayout()
        hbox.addWidget(self.load_button)
        hbox.addWidget(self.filename_edit)
        hbox.addWidget(self.plot_button)
        self.setLayout(hbox)

    def set_load_action(self):
        text = QtGui.QFileDialog.getOpenFileName(self, 'Select datafile')
        if text:
            self.filename_edit.setText(str(text))
            self.plot_button.click()

    def text(self):
        return str(self.filename_edit.text())
