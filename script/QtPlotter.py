from pyqtgraph.Qt import QtWidgets, QtCore
import pyqtgraph as pg
from PyQt5.QtWidgets import QApplication
import collections
import numpy as np
from random import randrange
import sys

class QtPlotter():

    def __init__(self, sampleinterval=0.1, timewindow=10., size=(800,600), dict_data=[]):
        # Data stuff
        self._interval = int(sampleinterval*1000)
        self._bufsize = int(timewindow/sampleinterval)
        self.x = np.linspace(-timewindow, 0.0, self._bufsize)
        # PyQtGraph stuff
        self.app = QApplication(sys.argv)
        self.pg_layout = pg.GraphicsLayoutWidget()
        self.curves = []
        self.buffers = [] 
        self.plts = []

        self.groups = len(dict_data)
        self.channels = sum(len(group) for group in dict_data)
        print("channels: ", self.channels)

        for group in range(self.groups):
            self.plts.append(self.pg_layout.addPlot())
            self.plts[-1].resize(*size)
            self.plts[-1].showGrid(x=True, y=True)
            self.plts[-1].setLabel('bottom', 'time', 's')
            self.plts[-1].addLegend()
            for subchannel in range( len(dict_data[group]) ):
                self.buffers.append(collections.deque([0.0]*self._bufsize, self._bufsize))
                self.curves.append( self.plts[-1].plot(self.x, self.buffers[-1], pen=(randrange(255), randrange(255), randrange(255)), name=dict_data[group][subchannel]) )    

        # QTimer
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.updateplot)
        self.timer.start(self._interval)



    def updateData(self, data):
        for i in range(self.channels):
            self.buffers[i].append(data[i])
        return True

    def updateplot(self):
        for i in range(self.channels):
            self.curves[i].setData(self.x, self.buffers[i])
        self.app.processEvents()

    def run(self):
        self.pg_layout.show()
        self.app.exec_()
            

if __name__ == '__main__':
    m = QtPlotter(sampleinterval=0.05, timewindow=20.,dict_data=[['d1', 'd2'], ['c1', 'c2'] ])
    m.run()
