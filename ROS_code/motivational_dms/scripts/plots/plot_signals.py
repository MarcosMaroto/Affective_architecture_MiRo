import sys
import matplotlib.pyplot as plt
import numpy as np
import csv
from datetime import datetime
import os


file_date = sys.argv[1]

data_save = "/home/marcosm/ROS/catkin_workspace/src/motivational_dms/data/log/" + file_date + "/graphs/emotions/"
data_path =  "/home/marcosm/ROS/catkin_workspace/src/motivational_dms/data/log/" + file_date + "/emotions/emotion-"

if not os.path.exists(data_save):
    os.makedirs(data_save)

with open(data_path+"emotion_signal.txt",'r') as csvfile:
	data = csv.reader(csvfile, delimiter=';')
	x = list()
	y = list()
	z = list()

	for idx, row in enumerate(data):
		if idx == 0:
			start_time = datetime.strptime(row[0], '%d/%m/%Y  %H:%M:%S.%f')
		diff_time = datetime.strptime(row[0], '%d/%m/%Y  %H:%M:%S.%f')  - start_time
		x.append(float(diff_time.total_seconds()))
		y.append(float(row[1]))
		#z.append(np.mean(y))

# Calculation of statistical values
mean = round(np.mean(y), 2)
std = round(np.std(y), 2)
max_value = round(max(y), 2)
min_value = round(min(y), 2)
variance = round(np.var(y), 2)

fig, ax = plt.subplots()
plt.figure(figsize=(12,8))
plt.plot(x,y, color="blue", linewidth=2)
#plt.plot(x,z, color="k", linewidth=2)
plt.ylim([0,100])
plt.xlim([0,x[-1]])
plt.xlabel("Time(s)", weight="bold")
plt.ylabel('Value', weight="bold")
plt.title("Emotional signal", weight="bold", y=1.03, size=18)
plt.grid(True)
plt.legend(["Emotional Sig", "Mean"])
plt.text(-150, 95, 'Mean: '+str(mean), color='k', fontsize=11)
plt.text(-150, 90, 'Std: '+str(std), color='k', fontsize=11)
plt.text(-150, 85, 'Max: '+str(max_value), color='k', fontsize=11)
plt.text(-150, 80, 'Min: '+str(min_value), color='k', fontsize=11)
plt.text(-150, 75, 'Var: '+str(variance), color='k', fontsize=11)
plt.savefig(data_save+"emotional_signal.png")
plt.cla()

with open(data_path+"external_emotion_signal.txt",'r') as csvfile:
	data = csv.reader(csvfile, delimiter=';')
	x = list()
	y = list()
	z = list()

	for idx, row in enumerate(data):
		if idx == 0:
			start_time = datetime.strptime(row[0], '%d/%m/%Y  %H:%M:%S.%f')
		diff_time = datetime.strptime(row[0], '%d/%m/%Y  %H:%M:%S.%f')  - start_time
		x.append(float(diff_time.total_seconds()))
		y.append(float(row[1]))
		#z.append(np.mean(y))

# Calculation of statistical values
mean = round(np.mean(y), 2)
std = round(np.std(y), 2)
max_value = round(max(y), 2)
min_value = round(min(y), 2)
variance = round(np.var(y), 2)

fig, ax = plt.subplots()
plt.figure(figsize=(12,8))
plt.plot(x,y, color="blue", linewidth=2)
#plt.plot(x,z, color="k", linewidth=2)
plt.ylim([0,100])
plt.xlim([0,x[-1]])
plt.xlabel("Time(s)", weight="bold")
plt.ylabel('Value', weight="bold")
plt.title("External emotional signal", weight="bold", y=1.03, size=18)
plt.grid(True)
plt.legend(["Ext. Emotional Sig", "Mean"])
plt.text(-150, 95, 'Mean: '+str(mean), color='k', fontsize=11)
plt.text(-150, 90, 'Std: '+str(std), color='k', fontsize=11)
plt.text(-150, 85, 'Max: '+str(max_value), color='k', fontsize=11)
plt.text(-150, 80, 'Min: '+str(min_value), color='k', fontsize=11)
plt.text(-150, 75, 'Var: '+str(variance), color='k', fontsize=11)
plt.savefig(data_save+"external_emotion_signal.png")
plt.cla()