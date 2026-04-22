import serial
import threading
import time
import pyqtgraph
from pyqtgraph.Qt import QtWidgets
from pyqtgraph.Qt import QtCore
import struct
import numpy   
import csv 
from pathlib import Path

#Setting up serial
refreshRate = 100 #based off MAX6675
serialReader = serial.Serial(port = "COM6", baudrate = 9600, timeout = 1)
startingByte = b'\xAA'
packageSize = 28 #in bytes
time.sleep(2) #wait for arduino to reset
print("Arduino ready")

startingTime = time.time()
downloads = Path.home() / "Downloads"
defaultFilename = downloads / f"PIDdata_{int(startingTime)}.csv"
def saveAsCSV(filename = defaultFilename):
    with open(filename, mode = 'w', newline = '') as file:
        writer = csv.writer(file)

        #Headers
        writer.writerow(["Time(s)", "Setpoint (C)", "Temperature (C)", 
                         "TotalOutput (%)", "PIDOutput (%)"])
        
        #Data
        writer.writerows(loggedData)
        print("Data saved to downloads")
        
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
        (setpoint, measurement, totalPowerOutput, PIDcorrection, Kp, Ki, Kd) = struct.unpack("<fffffff", bytePackage)
        return (setpoint, measurement, totalPowerOutput, PIDcorrection, Kp, Ki, Kd) #return tuple
    except:
        return None #decode failed
    
def sendCommand(cmd, value): #string, float
    # P -> setpoint
    # KP -> P gain
    # KI -> I gain
    # KD -> D gain
    command = f"{cmd}{value}\n" 
    print(f"Command {cmd}{value} sent\n")
    serialReader.write(command.encode())

def inputThread(): #thread allows for Arduino commands without interrupts
    while True:
        try:
            userInput = input("Enter command (SP/KP/KI/KD) + value: \n\n").strip()
            if not userInput: #blank input
                continue
            if userInput.upper() == "SAVE":
                saveAsCSV()
                continue
            cmd = userInput[:2].upper()
            value = float(userInput[2:])
            sendCommand(cmd, value)
        except:
            print("Invalid input")

#Begin input thread
threading.Thread(target = inputThread, daemon = True).start()

#setup Plotter
plotController = QtWidgets.QApplication([])
plotWindow = pyqtgraph.GraphicsLayoutWidget(show = True, title = "PID Heating System")
plot = plotWindow.addPlot()
plot.setLabel('left', 'Value')    # Y-axis label
plot.setLabel('bottom', 'Time')   # X-axis label
curveSetpoint = plot.plot(pen='r', name="Setpoint")
curveMeasurement = plot.plot(pen='g', name="Measured")
curveOutput = plot.plot(pen='b', name="Total Output")
curvePID = plot.plot(pen='y', name="PID Correction")
# plot.enableAutoRange(axis='y')
plot.setYRange(0, 100)
plot.addLegend(offset=(10,10))

#initialize empty data
numPoints = 500 #number of points plotted at a time
loggedData = []
setpointData = numpy.zeros(numPoints)
measurementData = numpy.zeros(numPoints)
outputData = numpy.zeros(numPoints)
PIDData = numpy.zeros(numPoints)

def update(setpoint, measurement, totalPowerOutput, PIDcorrection):
    setpointData[:] = numpy.roll(setpointData,-1) #shift old data left 1
    measurementData[:] = numpy.roll(measurementData,-1)
    outputData[:] = numpy.roll(outputData,-1)
    PIDData[:] = numpy.roll(PIDData,-1)

    setpointData[-1] = setpoint # add new values to data
    measurementData[-1] = measurement
    outputData[-1] = totalPowerOutput
    PIDData[-1] = PIDcorrection


    curveSetpoint.setData(setpointData) #redraw graphs
    curveMeasurement.setData(measurementData)
    curveOutput.setData(outputData)
    curvePID.setData(PIDData)

counter = 0
def readAndUpdate():
    global counter
    if(serialReader.is_open):
        packet = readPacket()
        if packet == None:
            return #missing packet
    else:
        return #nothing in serial
    decoded = decodePacket(packet)
    if(decoded == None):
        return #decode failed
    setpoint, measurement, totalPowerOutput, PIDcorrection, Kp, Ki, Kd = decoded

    #Adding new data to logged data
    loggedData.append([time.time() - startingTime, setpoint, measurement, totalPowerOutput, PIDcorrection])

    #printing diagnostics
    if counter % 10 == 0: #print every 10th sample
        print(f"Temp: {measurement:.2f} °C | Kp: {Kp:.2f} | Ki: {Ki:.2f} | Kd: {Kd:.2f}")
    counter += 1
    update(setpoint, measurement, totalPowerOutput, PIDcorrection)

#main loop using timer
timer = QtCore.QTimer()
timer.timeout.connect(readAndUpdate)
timer.start(refreshRate)

#begin timer loop
QtWidgets.QApplication.instance().exec_()