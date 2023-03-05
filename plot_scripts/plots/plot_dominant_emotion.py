import sys
import matplotlib.pyplot as plt
import numpy as np
import csv
from datetime import datetime
import os

file_date = sys.argv[1]

data_save = "/home/marcosm/ROS/catkin_workspace/src/motivational_dms/data/log/" + file_date + "/graphs/emotions/"
data_path =  "/home/marcosm/ROS/catkin_workspace/src/motivational_dms/data/log/" + file_date + "/emotions/emotion-dominant_emotion"

if not os.path.exists(data_save):
    os.makedirs(data_save)

emotions = {"calmness": 0, "joy": 1, "sadness": 2, "surprise": 3}
emotions_time = {"calmness": 0, "joy": 0, "sadness": 0, "surprise": 0}
total_time = 0
color = ["grey", "darkorange", "blue", "cyan"]

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
		y.append(int(emotions[row[1]]))
		z.append(datetime.strptime(row[0], '%d/%m/%Y  %H:%M:%S.%f'))

		emo_time = (z[idx]- z[idx-1]).total_seconds()
		emotions_time[row[1]] += emo_time

total_time = (z[-1] - z[0]).total_seconds()

for key, value in emotions_time.items():
	emotions_time[key] = 100 * value/total_time
	emotions_time[key.replace("_", " ").title()] = emotions_time.pop(key)

plt.figure(figsize=(12,8))
fig, ax = plt.subplots() 
y_pos = np.arange(len(emotions_time.values()))
plt.bar(y_pos, emotions_time.values(), align='center', color="lime")
plt.xticks(y_pos, emotions_time.keys(), weight="bold", size=14)
plt.ylim([0,100])
for i, v in enumerate(emotions_time.values()):
    ax.text(i, v+2, str(round(v,1))+"%", color='lime', weight='bold', ha="center")
plt.ylabel('Percentage (%)', weight="bold")
plt.title("Dominant emotion percentage", weight="bold", y=1.03, size=18)
plt.grid(True)
plt.savefig(data_save+"dominant_emotion_bars.png")
plt.cla()

plt.figure(figsize=(12,8))
plt.plot(x,y, color="r", marker=".", markersize=5, linestyle="None")
plt.axis([0, max(x), -0.2, 3.2])
plt.yticks(emotions.values(), emotions_time.keys(), size=12, weight="bold")
plt.xlabel('Time', weight="bold")
plt.ylabel('Value', weight="bold")
plt.title("Dominant emotion", weight="bold", y=1.03, size=18)
plt.grid(True)
plt.savefig(data_save+"dominant_emotion.png")
plt.cla()