import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

# Getting displacement data from Paraview ---------------------------------------------------------------------------

# Naming convention: ##_TOP, ##_BOTTOM --> ex. 01_TOP, 01_BOTTOM
testNumber          = "00"
initialLength       = 5.4     #in mm
DICTimeOffset       = 0     #in s
MM_TO_PIXEL         = 106   #pix/mm
TIMESTEP_TO_TIME    = 0.025*25   #s/timestep



downloads = Path.home() / "Downloads"
gaugeTopFile    = downloads / (testNumber + "_TOP.csv")
gaugeBottomFile = downloads / (testNumber + "_BOTTOM.csv")

gaugeTop        = pd.read_csv(gaugeTopFile)
gaugeBottom     = pd.read_csv(gaugeBottomFile)

print(gaugeTop.columns)
# Y displacements
avgDispTop      = gaugeTop.groupby("TimeStep", as_index=False)["DISPLACEMENT:1"].mean()
avgDispBottom   = gaugeBottom.groupby("TimeStep", as_index=False)["DISPLACEMENT:1"].mean()

DICdata = avgDispTop.merge(avgDispBottom, on = "TimeStep", suffixes = ("_TOP", "_BOTTOM"))

DICdata["deltaLength"] = (
    DICdata["DISPLACEMENT:1_TOP"]
    - DICdata["DISPLACEMENT:1_BOTTOM"]
)

# convert to mm and s
initialLength *= MM_TO_PIXEL
DICdata["strain"] = (DICdata["deltaLength"] / initialLength)
DICdata["Time"] = DICdata["TimeStep"] * TIMESTEP_TO_TIME
print(DICdata)

#Getting stress data from testworks --------------------------------------------------------------------
testWorksFileName = "PlasticDIC"
testWorksFileName =  downloads / (testWorksFileName + ".csv")

testWorksData = pd.read_csv(testWorksFileName)

print(testWorksData)

#Combine DIC and testworks data -----------------------------------------------------------------------
# align time using load and strain 
loadTW = testWorksData["Stress (Mpa)"]
initialLoad = loadTW.iloc[0]
loadThreshold = initialLoad + 0.02 * (loadTW.max() - initialLoad) #triggers on 2% increase
timeTW = testWorksData["Time"]

strainDIC = DICdata["strain"]
initialStrain = strainDIC.iloc[0]
strainThreshold = initialStrain + 0.02 * (strainDIC.max() - initialStrain) 
timeDIC = DICdata["Time"]

startLoadIndex = np.where(loadTW > loadThreshold)[0][0]
startStrainIndex = np.where(strainDIC > strainThreshold)[0][0]

DICTimeOffset = timeTW.iloc[startLoadIndex] - timeDIC.iloc[startStrainIndex]
# DICdata["Time"] += DICTimeOffset*1.75
DICdata["Time"] -= DICdata["Time"].iloc[0]
testWorksData["Time"] -= testWorksData["Time"].iloc[0]

result = testWorksData.copy()
start = DICdata["Time"].min()
end = DICdata["Time"].max()
result = result[(result["Time"] >= start) & (result["Time"] <= end)].copy()

result["Strain"]  = np.interp(result["Time"], DICdata["Time"], DICdata["strain"])
result["Stress (Mpa)"] -= result["Stress (Mpa)"].iloc[0]
print(result)

#Plotting ----------------------------------------------------------------------------------------------
time = result["Time"]
strain = result["Strain"]
stress = result["Stress (Mpa)"]


print("Lengths:")
print("DIC strain:", len(result["Strain"]))
print("TW strain:", len(result["Strain (mm/mm)"]))
print("Stress:", len(stress))

print("\nLast values:")
print(result[[
    "Time",
    "Strain",
    "Strain (mm/mm)",
    "Stress (Mpa)"
]].tail(10))

#plot
x = strain
y = stress
plt.figure()

# plt.plot(time, strain, color = "r")
# plt.plot(time, result["Strain (mm/mm)"], color = "g")
# plt.xlim(0, 1.5 * np.max(time))          
# plt.ylim(-1, 2 * np.max(strain)) 

plt.plot(x, y, color = "r")
# plt.plot(result["Strain (mm/mm)"], y, color = "g")
plt.xlim(-0.001, 1.1 * np.max(x))          
plt.ylim(-1, 1.1 * np.max(y))  
plt.xlabel(x.name)
plt.ylabel(y.name)
plt.title("Data for test " + testNumber)
plt.grid()


plt.show()




