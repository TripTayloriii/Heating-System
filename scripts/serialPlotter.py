import serial
import pyqtgraph
from pyqtgraph.Qt import QtWidgets
from pyqtgraph.Qt import QtCore
import struct
import numpy    

#Setting up serial
refreshRate = 225 #based off MAX6675
serialReader = serial.Serial(port = "COM6", baudrate = 9600, timeout = 1)
startingByte = b'\xAA'
packageSize = 12 #in bytes

def readPacket():
    while(True):
        if serialReader.read(1) == startingByte: #find starting byte
            break
    package = serialReader.read(packageSize)
    if(len(package) != packageSize):
        return None #package incompelete
    return package

def decodePacket(bytePackage):
    try:
        (setpoint, measurement, PIDoutput) = struct.unpack("<fff", bytePackage)
        return (setpoint, measurement, PIDoutput) #return tuple
    except:
        return None #decode failed

#setup Plotter
plotController = QtWidgets.QApplication([])
plotWindow = pyqtgraph.GraphicsLayoutWidget(show = True, title = "PID Heating System")
plot = plotWindow.addPlot()
plot.setLabel('left', 'Value')    # Y-axis label
plot.setLabel('bottom', 'Time')   # X-axis label
curveSetpoint = plot.plot(pen='r', name="Setpoint")
curveMeasurement = plot.plot(pen='g', name="Measured")
curvePID = plot.plot(pen='y', name="PID Output")
plot.enableAutoRange(axis='y')
plot.addLegend()

#initialize empty data
numPoints = 500 #number of points plotted at a time
setpointData = numpy.zeros(numPoints)
measurementData = numpy.zeros(numPoints)
PIDData = numpy.zeros(numPoints)

def update(setpoint, measurement, PIDoutput):
    setpointData[:] = numpy.roll(setpointData,-1) #shift old data left 1
    measurementData[:] = numpy.roll(measurementData,-1)
    PIDData[:] = numpy.roll(PIDData,-1)

    setpointData[-1] = setpoint # add new values to data
    measurementData[-1] = measurement
    PIDData[-1] = PIDoutput

    curveSetpoint.setData(setpointData) #redraw graphs
    curveMeasurement.setData(measurementData)
    curvePID.setData(PIDData)

def readAndUpdate():
    if(serialReader.is_open):
        packet = readPacket()
        if packet == None:
            return #missing packet
    else:
        return #nothing in serial
    setpoint, measurement, PIDoutput = decodePacket(packet)
    update(setpoint, measurement, PIDoutput)

#main loop using timer
timer = QtCore.QTimer()
timer.timeout.connect(readAndUpdate)
timer.start(refreshRate)

#begin timer loop
QtWidgets.QApplication.instance().exec_()