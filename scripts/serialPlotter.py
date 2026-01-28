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
packageSize = 20 #in bytes

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
        (setpoint, rampedSetpoint, measurement, totalPowerOutput, PIDcorrection) = struct.unpack("<fffff", bytePackage)
        return (setpoint, rampedSetpoint, measurement, totalPowerOutput, PIDcorrection) #return tuple
    except:
        return None #decode failed

#setup Plotter
plotController = QtWidgets.QApplication([])
plotWindow = pyqtgraph.GraphicsLayoutWidget(show = True, title = "PID Heating System")
plot = plotWindow.addPlot()
plot.setLabel('left', 'Value')    # Y-axis label
plot.setLabel('bottom', 'Time')   # X-axis label
curveSetpoint = plot.plot(pen='r', name="Setpoint")
curveRampedSP = plot.plot(pen='m', name="Ramped Setpoint")
curveMeasurement = plot.plot(pen='g', name="Measured")
curveOutput = plot.plot(pen='b', name="Total Output")
curvePID = plot.plot(pen='y', name="PID Correction")
# plot.enableAutoRange(axis='y')
plot.setYRange(0, 100)
plot.addLegend(offset=(10,10))

#initialize empty data
numPoints = 500 #number of points plotted at a time
setpointData = numpy.zeros(numPoints)
rampedSPData = numpy.zeros(numPoints)
measurementData = numpy.zeros(numPoints)
outputData = numpy.zeros(numPoints)
PIDData = numpy.zeros(numPoints)

def update(setpoint, rampedSetpoint, measurement, totalPowerOutput, PIDcorrection):
    setpointData[:] = numpy.roll(setpointData,-1) #shift old data left 1
    rampedSPData[:] = numpy.roll(rampedSPData,-1)
    measurementData[:] = numpy.roll(measurementData,-1)
    outputData[:] = numpy.roll(outputData,-1)
    PIDData[:] = numpy.roll(PIDData,-1)

    setpointData[-1] = setpoint # add new values to data
    rampedSPData[-1] = rampedSetpoint
    measurementData[-1] = measurement
    outputData[-1] = totalPowerOutput
    PIDData[-1] = PIDcorrection


    curveSetpoint.setData(setpointData) #redraw graphs
    curveRampedSP.setData(rampedSPData)
    curveMeasurement.setData(measurementData)
    curveOutput.setData(outputData)
    curvePID.setData(PIDData)

def readAndUpdate():
    if(serialReader.is_open):
        packet = readPacket()
        if packet == None:
            return #missing packet
    else:
        return #nothing in serial
    setpoint, rampedSetpoint, measurement, totalPowerOutput, PIDcorrection = decodePacket(packet)
    update(setpoint, rampedSetpoint, measurement, totalPowerOutput, PIDcorrection)

#main loop using timer
timer = QtCore.QTimer()
timer.timeout.connect(readAndUpdate)
timer.start(refreshRate)

#begin timer loop
QtWidgets.QApplication.instance().exec_()