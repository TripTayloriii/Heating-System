import serial
import pyqtgraph
from pyqtgraph.Qt import QtWidgets
import struct
import numpy    

#Setting up serial
serialReader = serial.Serial(port = COM4, baudrate = 9600, timeout = 1)
startingByte = b'\xAA'
packageSize = 12 #in bytes

def readPacket():
    while(True):
        serialReader.read_until(startingByte) #find starting byte
        break
    package = serialReader.read(packageSize)
    if(len(package) != packageSize):
        return None #package incompelete
    return package

def decodePacket(bytePackage):
    (setpoint, measurement, PIDoutput) = struct.unpack("<fff", bytePackage)
    return (setpoint, measurement, PIDoutput) #return tuple

#setup Plotter
plotController = QtWidgets.QApplication([])
plotWindow = pyqtgraph.GraphicsLayoutWidget(show = True, title = "PID Heating System")
plot = plotWindow.addPlot()
plot.setLabel('left', 'Value')    # Y-axis label
plot.setLabel('bottom', 'Time')   # X-axis label
curveSetpoint = plot.plot(pen='r', name="Setpoint")
curveMeasurement = plot.plot(pen='g', name="Measured")
curvePID = plot.plot(pen='y', name="PID Output")

#initialize empty data
numPoints = 500 #number of points plotted at a time
setpointData = numpy.zeros(500)
measurementData = numpy.zeros(500)
PIDData = numpy.zeros(500)

def updateData(setpoint, measurment, PIDoutput):
    

#main loop

