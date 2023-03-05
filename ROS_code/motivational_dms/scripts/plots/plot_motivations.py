import sys
import matplotlib.pyplot as plt
import csv
from datetime import datetime
import os


file_date = sys.argv[1]

data_save = "/home/marcosm/ROS/catkin_workspace/src/motivational_dms/data/log/" + file_date + "/graphs/motivations/"
data_path =  "/home/marcosm/ROS/catkin_workspace/src/motivational_dms/data/log/" + file_date + "/motivation/motivation-"

if not os.path.exists(data_save):
    os.makedirs(data_save)

motivations = ["play", "rest", "interaction", "no_motivation"]
color = ["b", "g", "r", "k"]

for p, variable in enumerate(motivations):

	with open(data_path+variable+".txt",'r') as csvfile:
		data = csv.reader(csvfile, delimiter=';')
		x = list()
		y = list()

		for idx, row in enumerate(data):
			if idx == 0:
				start_time = datetime.strptime(row[0], '%d/%m/%Y  %H:%M:%S.%f')
			diff_time = datetime.strptime(row[0], '%d/%m/%Y  %H:%M:%S.%f')  - start_time
			x.append(float(diff_time.total_seconds()))
			y.append(float(row[1])) 

		plt.figure(figsize=(12,8))
		plt.plot(x,y, color=color[p], label=variable,linewidth=2)
		plt.axis([0, max(x)+10, 0, 180])
		plt.xlabel('Time', weight="bold")
		plt.ylabel('Value', weight="bold")
		plt.title(variable.title(), weight="bold", size=18, y=1.03)
		plt.grid(True)
		#plt.legend()
		plt.savefig(data_save+variable+".png")
		plt.cla()