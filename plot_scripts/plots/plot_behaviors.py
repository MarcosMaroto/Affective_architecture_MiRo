import sys
import matplotlib.pyplot as plt
import numpy as np
import csv
from datetime import datetime
import os

file_date = sys.argv[1]

data_save = "/home/marcosm/ROS/catkin_workspace/src/motivational_dms/data/log/" + file_date + "/graphs/decision_making/"
data_path =  "/home/marcosm/ROS/catkin_workspace/src/motivational_dms/data/log/" + file_date + "/decision_making/behavior-active_behavior"

if not os.path.exists(data_save):
    os.makedirs(data_save)

behaviors = {"wander": 0, "find_user": 1, "approach_user": 2, "sleeping": 3, "request_interaction": 4, "play_alone": 5, "play_game": 6}

with open(data_path+".txt",'r') as csvfile:
	data = csv.reader(csvfile, delimiter=';')
	x = list()
	y = list()

	for idx, row in enumerate(data):
		if idx == 0:
			start_time = datetime.strptime(row[0], '%d/%m/%Y  %H:%M:%S.%f')
		diff_time = datetime.strptime(row[0], '%d/%m/%Y  %H:%M:%S.%f')  - start_time
		x.append(float(diff_time.total_seconds()))	
		if "express_" in row[3]:
			row[3] = "wander"
		y.append(int(behaviors[row[3]]))

	plt.figure(figsize=(12,8))
	plt.plot(x,y, color="r", marker=".", markersize=5, linestyle="None")
	plt.axis([0, max(x), -0.2, 6.2])
	plt.yticks(behaviors.values(), behaviors.keys(), size=12, weight="bold")
	plt.xlabel('Time(s)', weight="bold")
	plt.ylabel('Value', weight="bold")
	plt.title("Active behavior", weight="bold", y=1.03, size=18)
	plt.grid(True)
	plt.savefig(data_save+"behaviors.png")
	plt.cla()