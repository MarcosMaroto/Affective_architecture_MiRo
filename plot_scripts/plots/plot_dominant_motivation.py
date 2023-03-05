import sys
import matplotlib.pyplot as plt
import numpy as np
import csv
from datetime import datetime
import os


file_date = sys.argv[1]

data_save = "/home/marcosm/ROS/catkin_workspace/src/motivational_dms/data/log/" + file_date + "/graphs/motivations/"
data_path =  "/home/marcosm/ROS/catkin_workspace/src/motivational_dms/data/log/" + file_date + "/motivation/motivation-dominant"

if not os.path.exists(data_save):
    os.makedirs(data_save)

motivations = {"play": 0, "rest": 1, "interaction": 2, "no_motivation": 3}
motivations_time = {"play": 0, "rest": 0, "interaction": 0, "no_motivation": 0}
total_time = 0
color = ["b", "g", "r", "k"]

with open(data_path+".txt",'r') as csvfile:
	data = csv.reader(csvfile, delimiter=';')
	x = list()
	y = list()
	z = list()

	for idx, row in enumerate(data):
		if idx == 0:
			start_time = datetime.strptime(row[0], '%d/%m/%Y  %H:%M:%S.%f')
		diff_time = datetime.strptime(row[0], '%d/%m/%Y  %H:%M:%S.%f')  - start_time
		x.append(float(diff_time.total_seconds()))
		y.append(int(motivations[row[1]]))
		z.append(datetime.strptime(row[0], '%d/%m/%Y  %H:%M:%S.%f'))

		mot_time = (z[idx]- z[idx-1]).total_seconds()
		motivations_time[row[1]] += mot_time

total_time = (z[-1] - z[0]).total_seconds()

for key, value in motivations_time.items():
	motivations_time[key] = 100 * value/total_time
	motivations_time[key.replace("_", " ").title()] = motivations_time.pop(key)

plt.figure(figsize=(12,8))
fig, ax = plt.subplots() 
y_pos = np.arange(len(motivations_time.values()))
plt.bar(y_pos, motivations_time.values(), align='center', color="cornflowerblue")
plt.xticks(y_pos, motivations_time.keys(), weight="bold", size=14)
for i, v in enumerate(motivations_time.values()):
    ax.text(i, v+2, str(round(v,1))+"%", color='cornflowerblue', weight='bold', ha="center")
plt.ylim([0,100])
plt.ylabel('Percentage (%)', weight="bold")
plt.title("Dominant motivation percentage", weight="bold", y=1.03)
plt.grid(True)
plt.savefig(data_save+"dominant_motivation_bars.png")
plt.cla()

plt.figure(figsize=(12,8))
plt.plot(x,y, color="r", marker=".", markersize=5, linestyle="None")
plt.axis([0, max(x), -0.2, 3.2])
plt.yticks(motivations.values(), motivations_time.keys(), size=9, weight="bold")
plt.xlabel('Time', weight="bold")
plt.ylabel('Value', weight="bold")
plt.title("Dominant motivation", weight="bold", size=18,  y=1.03)
plt.grid(True)
plt.savefig(data_save+"dominant_motivation.png")
plt.cla()