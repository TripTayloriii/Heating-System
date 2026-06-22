import pandas as pd
import matplotlib.pyplot as plt
import os
import numpy as np

#load CSV file
filename = "1.43V-2.024A-2.935W.csv"  #filename

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


#choose x and y axis
x = time  # first column
y = measurement  # second column


#plot
plt.figure()
plt.plot(x, y)
plt.xlabel(df.columns[0])
plt.ylabel(df.columns[2])
plt.title(filename[:filename.find(".csv")])
plt.grid()
plt.xlim(0, len(x))          
plt.ylim(-1, 100)   
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
    #criteria 1
    error = setpoint - measurement
    dt = np.diff(time, prepend=time.iloc[0])
    iae = np.sum(np.abs(error) * dt)
    iae = iae/100
    print("IAE: ", iae)

    #criteria 2
    target = setpoint.iloc[-1]
    overshoot = max(0, np.max(measurement) - target)
    print("Overshoot", overshoot)

    #criteria 3
    settlingTime = 100
    for i in range(len(error)):
        if np.all(np.abs(error[i:]) < 5.0):
            settlingTime = time.iloc[i]
            break
    print("Settling Time: ", settlingTime)

    totalScore = 0.7*iae + 0.2*overshoot + 0.1*settlingTime
    return totalScore

print("Score: ", getScore(time, setpoint, measurement))

plt.show()



