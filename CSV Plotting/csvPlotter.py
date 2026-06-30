import pandas as pd
import matplotlib.pyplot as plt
import os
import numpy as np

#load CSV file
filename = "PID5V-P12-I2-D2.csv"  #filename

#file must me in same folder as script
script_dir = os.path.dirname(os.path.abspath(__file__))
file_path = os.path.join(script_dir, filename)
df = pd.read_csv(file_path)

#print columns
print(df.columns)

time = df.iloc[:, 0]
setpoint = df.iloc[:, 1]
measurement = df.iloc[:, 2]
totalOutput = df.iloc[:, 3]
PIDOutput = df.iloc[:, 4]


#trim data to window
startTime = 0
endTime = time.iloc[-1]
startTime = 65
# endTime = 300
window = (time >= startTime) & (time <= endTime)
time = time[window].reset_index(drop=True)
time = time - startTime
setpoint = setpoint[window].reset_index(drop=True)
measurement = measurement[window].reset_index(drop=True)


#plot
plt.figure()
plt.plot(time, measurement)
plt.plot(time, setpoint)
plt.xlabel(df.columns[0])
plt.ylabel(df.columns[2])
plt.title(filename[:filename.find(".csv")])
plt.grid()
plt.xlim(0, endTime - startTime)          
plt.ylim(-1, 300)   
#annotations
# plt.axvline(x=187.5, linestyle='--', color = 'r')
# plt.text(189.5, 60, "Measuring Sample", rotation=90, fontsize = 8)     

# plt.axvline(x=308, linestyle='--', color = 'r')
# plt.text(310, 60, "Measuring CFC", rotation=90, fontsize = 8)   

# plt.axvline(x=404, linestyle='--', color = 'r')
# plt.text(406, 60, "Measuring Copper Lug", rotation=90, fontsize = 8)   

# plt.axvline(x=352, linestyle='--', color = 'r')
# plt.text(354, 45, "Measuring Wire near CFC", rotation=90, fontsize = 8)  


# Analysis 
def getScore(time, setpoint, measurement):
    print("\n\n\n--------------------------------------\n")
    #checking edge cases
    if len(time) == 0:
        return float('inf')

    #criteria 1 - iae
    error = setpoint - measurement
    dt = np.diff(time, prepend=time.iloc[0])
    iae = np.sum(np.abs(error) * dt)
    iae = iae/100
    print("IAE:                   ", iae)

    #criteria 2 - overshoot
    target = setpoint.iloc[-1]
    overshoot = max(0, np.max(measurement) - target)
    print("Overshoot:             ", overshoot, "C")

    #criteria 3 - settling time
    settlingTime = time.iloc[-1]
    settlingIndex = len(time) - 1
    for i in range(len(error)):
        if np.all(np.abs(error[i:]) < 5.0):
            settlingTime = time.iloc[i] - time.iloc[0]
            settlingIndex = i
            print("Settling Time:         ", settlingTime, "s")
            break
    if settlingTime == time.iloc[-1]:
        print("Settling Time:          Not settled for", settlingTime, "s")

    #criteria 4 - oscillation amplitude
    setpointReached = measurement.iloc[settlingIndex:]
    oscillationAmp = (np.max(setpointReached) - np.min(setpointReached)) / 2
    print("Oscillation Amplitude: ", oscillationAmp, "C")

    #final score
    totalScore = 0.7*iae + 0.2*overshoot + 0.1*settlingTime + 0.1*oscillationAmp
    return totalScore

print("Score:                 ", getScore(time, setpoint, measurement))

plt.show()



