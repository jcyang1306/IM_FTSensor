from pyqtgraph.Qt import QtCore
import pyqtgraph as pg
from PyQt5.QtWidgets import QApplication
import scipy.signal

from collections import deque
import numpy as np
from random import randrange
import sys

class QtPlotter():

    def __init__(self, sampleinterval=0.1, timewindow=10., size=(800,600), dict_data=[], filteron = True):
        # Data stuff
        self._interval = int(sampleinterval*1000)
        self._bufsize = int(timewindow/sampleinterval)
        self.x = np.linspace(-timewindow, 0.0, self._bufsize)
        # PyQtGraph stuff
        self.app = QApplication(sys.argv)
        self.pg_layout = pg.GraphicsLayoutWidget()
        self.curves = []
        self.plts = []
        self.buffers = [] 

        self.filteron = filteron
        self.xs_bufs = []
        self.ys_bufs = []

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
                self.buffers.append(deque([0.0]*self._bufsize, self._bufsize))
                self.curves.append( self.plts[-1].plot(self.x, self.buffers[-1], pen=(randrange(255), randrange(255), randrange(255)), name=dict_data[group][subchannel]) )    

        # Live LPF stuff
        if filteron:
            # b, a = scipy.signal.iirfilter(4, Wn=[0.5, 2.5],rs=60, btype="bandpass", ftype="cheby2")
            b, a = scipy.signal.butter(2, 0.05, 'lowpass')   
            self.filt_b = b
            self.filt_a = a
            for i in range(self.channels):
                self.xs_bufs.append( deque([0] * len(b), maxlen=len(b)) )
                self.ys_bufs.append( deque([0] * (len(a) - 1), maxlen=len(a)-1) )


        # QTimer
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.updateplot)
        self.timer.start(self._interval)

    # input x[] -> output y[]
    # a0*y[n] = b0*x[n-1] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
    def processData(self, data):
        ys_vec = []
        for i in range(self.channels):
            print("data", data[i])
            self.xs_bufs[i].append(data[i])
            ys = np.dot(self.filt_b, self.xs_bufs[i]) - np.dot(self.filt_a[1:], self.ys_bufs[i]) 
            ys /= self.filt_a[0]
            self.ys_bufs[i].append(ys)
            ys_vec.append(ys)


            print(self.filt_b)
            print(self.filt_a)

            print(self.xs_bufs[i])
            print(self.ys_bufs[i], "\n")
            # print(i,"-> ", ys)
        return ys_vec


    def updateData(self, data):
        # TODO: get data from sensor here
        if self.filteron:
            f_data = self.processData(data)
        else:
            f_data = data
        for i in range(self.channels):
            self.buffers[i].append(f_data[i])
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
