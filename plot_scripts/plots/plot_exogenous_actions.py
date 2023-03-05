import sys
import matplotlib.pyplot as plt
import csv
from datetime import datetime
import os


file_date = sys.argv[1]

data_save = "/home/marcosm/ROS/catkin_workspace/src/motivational_dms/data/log/" + file_date + "/graphs/stimulus/"
data_path =  "/home/marcosm/ROS/catkin_workspace/src/motivational_dms/data/log/" + file_date + "/motivation/motivation-exogenous_actions"

if not os.path.exists(data_save):
    os.makedirs(data_save)

stimulus = ["touch"]
color = ["lime"]
values = dict()

for value in stimulus:

	values[value] = list()

with open(data_path+".txt",'r') as csvfile:
	data = csv.reader(csvfile, delimiter=';')

	x = list()

	for idx, row in enumerate(data):
		if idx > 0:
			if idx == 1:
				start_time = datetime.strptime(row[0], '%d/%m/%Y  %H:%M:%S.%f')
			diff_time = datetime.strptime(row[0], '%d/%m/%Y  %H:%M:%S.%f')  - start_time
			x.append(float(diff_time.total_seconds()))
			values[row[1]].append(float(row[3]))

for idx, (key, value) in enumerate(values.items()):
	aux_var = key.replace("_", " ").title()
	plt.figure(figsize=(12,8))
	plt.plot(x, value, color=color[idx], label=aux_var,linewidth=2)
	plt.axis([0, x[-1], 0, max(value)+5])
	plt.xlabel('Time(s)', weight="bold")
	plt.ylabel('Value', weight="bold")
	plt.title(aux_var, weight="bold", size=18, y=1.03)
	plt.grid(True)
	#plt.legend()
	plt.savefig(data_save+key+".png")
	plt.cla()