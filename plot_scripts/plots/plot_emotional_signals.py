import sys
import matplotlib.pyplot as plt
import csv
from datetime import datetime
import os
import numpy as np

file_date = sys.argv[1]

data_save = "/home/marcosm/ROS/catkin_workspace/src/motivational_dms/data/log/" + file_date + "/graphs/emotions/"
data_path =  "/home/marcosm/ROS/catkin_workspace/src/motivational_dms/data/log/" + file_date + "/emotions/emotion-signal_values"

if not os.path.exists(data_save):
    os.makedirs(data_save)

signals = ["Answers", "Wellbeing", "Touch Stimulus", "User presence"]
colors = ["cyan", "magenta", "darkorange","lime"]

fig, axs = plt.subplots(4, 1,figsize=(12,8))
for p, variable in enumerate(signals):
	with open(data_path+".txt",'r') as csvfile:
		data = csv.reader(csvfile, delimiter=';')

		x = list()
		y = list()
		z = list()

		for idx, row in enumerate(data):
			if idx > 0:
				if idx == 1:
					start_time = datetime.strptime(row[0], '%d/%m/%Y  %H:%M:%S.%f')
				diff_time = datetime.strptime(row[0], '%d/%m/%Y  %H:%M:%S.%f')  - start_time
				x.append(float(diff_time.total_seconds()))
				y.append(float(row[p+1]))
				#z.append(np.mean(y))

	# Calculation of statistical values
	mean = round(np.mean(y), 2)
	std = round(np.std(y), 2)
	max_value = round(max(y), 2)
	min_value = round(min(y), 2)
	variance = round(np.var(y), 2)

	aux_var = variable.replace("_", " ").title()
	axs[p].plot(x, y, color=colors[p], linestyle="-", linewidth=2)
	#axs[p].plot(x, z, color="k", linestyle="-", linewidth=2)
	axs[p].set_xlim([0, x[-1]])
	axs[p].set_ylim([0, 100.2])
	axs[p].set_xlabel('Time(s)', weight="bold")
	axs[p].set_ylabel(variable, weight="bold",size=10)
	axs[p].text(-150, 95, 'Mean: '+str(mean), color='k', fontsize=11)
	axs[p].text(-150, 85, 'Std: '+str(std), color='k', fontsize=11)
	axs[p].text(-150, 75, 'Max: '+str(max_value), color='k', fontsize=11)
	axs[p].text(-150, 65, 'Min: '+str(min_value), color='k', fontsize=11)
	axs[p].text(-150, 55, 'Var: '+str(variance), color='k', fontsize=11)
	axs[p].grid(True)

plt.savefig(data_save+"emotional_signals.png")
plt.cla()