import sys
import matplotlib.pyplot as plt
from datetime import datetime
import os
import stat
import numpy as np
import csv

data_path =  "/home/marcosm/Dropbox/MIRO/Graphs/Emotions/"
target_path = "/home/marcosm/Results/REAL/ADAPTED/"
file = "/emotion-emotion_signal.txt"
colors = ['lime', 'darkorange', 'indigo']
#colors = ['magenta', 'deepskyblue', 'darkorange']
#colors = ["lime", 'magenta', 'deepskyblue', 'darkorange', 'red', 'dimgrey', 'indigo']
#labels = ["nD", "MnD", "NR", "MD", "FD"]
#labels = ["MnD", "NR", "MD"]
#labels = ["FEng", "TEng", "GEng", "NU", "DwG", "DwT", "FDis"]
labels = ["FEng", "NU", "FDis"]

title = "ES_Adaptation_Real"
folders_n = 2
max_x = 0

directories = [x[0] for x in os.walk(target_path)]
files_dirs = list()

for dire in directories:
	if(os.path.isfile(dire+file)):
		files_dirs.append(dire+file)

if not os.path.exists(data_path):
    os.makedirs(data_path)

def running_mean(l, N):
    sum = 0
    result = list( 0 for x in l)
 
    for i in range( 0, N ):
        sum = sum + l[i]
        result[i] = sum / (i+1)
 
    for i in range( N, len(l) ):
        sum = sum - l[i-N] + l[i]
        result[i] = sum / N
 
    return result

plt.figure(figsize=(12,8), dpi=1000)
plt.ylim([0,100.1])
plt.xlabel("Time (s)", weight='bold', fontsize=24)
plt.ylabel("Value", weight='bold', fontsize=24)
plt.xticks(fontsize=24)
plt.yticks(fontsize=24)
plt.grid(True)

for n, dire in enumerate(sorted(files_dirs)):
	print "OUT DIR:", dire
	#if (n+1)%folders_n == 0:
	#if n > 5 and n < 9:
	print "ACCESSING DIR:", dire
	with open(dire,'r') as csvfile:
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

		if(x[-1]) > max_x:
			max_x = x[-1]

		for p, value in enumerate(y):
			if p < len(y)-1:
				low_limit = value - 1
				up_limit = value + 1
				if (low_limit < y[p+1] < up_limit):
					del y[p+1], x[p+1]

		averaged_data = running_mean(y, 200)

	plt.plot(x, averaged_data, linewidth=2.5, label=labels[n], color=colors[n])

#plt.plot(x, averaged_data, linewidth=1.5)
#plt.plot(x, y, linewidth=1.5)
#plt.plot(x, anchor(y, 1), linewidth=1.5)

plt.xlim([0, 1500])
plt.tight_layout()
#plt.legend(loc=4, fontsize=15, ncol=7)
plt.savefig(data_path+title+".png", bbox_inches="tight")
plt.cla()
os.chmod(data_path+title+".png", stat.S_IRWXO)
