import sys
import matplotlib.pyplot as plt
import csv
from datetime import datetime
import os


file_date = sys.argv[1]

data_save = "/home/marcosm/ROS/catkin_workspace/src/motivational_dms/data/log/" + file_date + "/graphs/emotions/adaptation/"
data_path =  "/home/marcosm/ROS/catkin_workspace/src/motivational_dms/data/log/" + file_date + "/emotions/emotion-"

if not os.path.exists(data_save):
    os.makedirs(data_save)

signals = ["internal_param", "external_param", "touch_stimulus_param", "user_answers_param", "wellbeing_param", "user_presence_param"]
color = "c"

for p, variable in enumerate(signals):
	plt.figure(figsize=(12,8))
	with open(data_path+variable+".txt",'r') as csvfile:
		data = csv.reader(csvfile, delimiter=';')

		x = list()
		y = list()

		for idx, row in enumerate(data):
			if idx > 0:
				if idx == 1:
					start_time = datetime.strptime(row[0], '%d/%m/%Y  %H:%M:%S.%f')
				diff_time = datetime.strptime(row[0], '%d/%m/%Y  %H:%M:%S.%f')  - start_time
				x.append(float(diff_time.total_seconds()))
				y.append(float(row[1]))

	aux_var = variable.replace("_", " ").title()
	plt.plot(x, y, color=color, linestyle="-", linewidth=2)
	plt.xlim([0, x[-1]])
	plt.ylim([0, 1.02])
	plt.xlabel('Time(s)', weight="bold")
	plt.ylabel('Value', weight="bold",size=10)
	plt.title(aux_var, weight="bold", y=1.03, size=18)
	plt.grid(True)
	plt.savefig(data_save+variable+".png")
	plt.cla()