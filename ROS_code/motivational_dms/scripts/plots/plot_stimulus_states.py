import sys
import matplotlib.pyplot as plt
import csv
from datetime import datetime
import os


file_date = sys.argv[1]

data_save = "/home/marcosm/ROS/catkin_workspace/src/motivational_dms/data/log/" + file_date + "/graphs/stimulus/"
data_path =  "/home/marcosm/ROS/catkin_workspace/src/motivational_dms/data/log/" + file_date + "/decision_making/behavior-external_stimulus"

if not os.path.exists(data_save):
    os.makedirs(data_save)

stimulus = ["lights", "user_near", "balls", "user_present", "user_far"]
colors = ['lime', "cyan", "magenta", "cyan", "red"]


fig, axs = plt.subplots(5, 1,figsize=(12,10))
for p, variable in enumerate(stimulus):

	x = list()
	y = list()

	with open(data_path+".txt",'r') as csvfile:
		data = csv.reader(csvfile, delimiter=';')

		for idx, row in enumerate(data):
			if idx > 0:
				if idx == 1:
					start_time = datetime.strptime(row[0], '%d/%m/%Y  %H:%M:%S.%f')
				diff_time = datetime.strptime(row[0], '%d/%m/%Y  %H:%M:%S.%f')  - start_time
				x.append(float(diff_time.total_seconds()))
				if row[p+1] == "True":
					y.append(int(1))
				else:
					y.append(int(0))

	aux_var = variable.replace("_", " ").title()
	axs[p].plot(x, y, color=colors[p], marker=".", markersize=5, linestyle="-", linewidth=0.5)
	axs[p].set_xlim([0, x[-1]])
	axs[p].set_ylim([-0.2, 1.2])
	axs[p].set_yticks([0,1])
	axs[p].set_yticklabels(["False","True"])
	axs[p].set_xlabel('Time(s)', weight="bold")
	axs[p].set_ylabel(variable, weight="bold",size=10)
	axs[p].grid(True)

plt.title("External stimulus state", weight="bold", size=18, y=6.03)
plt.savefig(data_save+"external_stimulus_states.png")
plt.cla()