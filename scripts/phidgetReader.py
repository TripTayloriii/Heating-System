# User's Manual: https://www.phidgets.com/?prodid=725&pcid=87#Tab_API
from Phidget22.Phidget import *
from Phidget22.Devices.TemperatureSensor import *
import time



def onTempChange(self, temperature):
    print("Temperature", temperature)


tempReader = TemperatureSensor()
tempReader.setOnTemperatureChangeHandler(onTempChange)
tempReader.openWaitForAttachment(Phidget.DEFAULT_TIMEOUT)
tempReader.setDataRate(tempReader.getMaxDataRate())
tempReader.open()
while True:
    time.sleep(0.01)