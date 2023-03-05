import sys
import matplotlib.pyplot as plt
from datetime import datetime
import os, stat
import numpy as np

data_path =  "/home/marcosm/Dropbox/MIRO/Graphs/Emotions/"

if not os.path.exists(data_path):
    os.makedirs(data_path)

title = "FullyEngaged"

summation = [0,0,0,0,0]

#signals = ["nD", "MnD", "NR", "MD", "FD"]
signals = ["MnD", "NR", "MD"]
values = [[0, 18.4, 11.6, 13.3, 11.0], [91.8, 2.1, 0.9, 0.8, 4.8], [8.2, 79.4, 87.5, 85.8, 84.0], [0, 0, 0, 0.1, 0.2]]
colors = ["lightgrey", "coral", "royalblue", "deepskyblue"]

ind = [x for x, _ in enumerate(signals)]

plt.figure(figsize=(8,8))
for idx, value in enumerate(summation):
	if values[0][idx] > 10:
		plt.text(idx-0.38, (values[0][idx]/2)+summation[idx]-1, str(values[0][idx])+"%", weight='bold', fontsize=18)
	elif values[0][idx] > 3:
		plt.text(idx-0.3, (values[0][idx]/2)+summation[idx]-1, str(values[0][idx])+"%", weight='bold', fontsize=18)
	summation[idx] += values[0][idx]
plt.bar(ind, summation, color=colors[0], width=0.8, align='center', label='Calmness', linewidth=1.5)
plt.bar(ind, values[1], bottom=summation, width=0.8, color=colors[1], align='center', label='Joy', linewidth=1.5)
for idx, value in enumerate(summation):
	if values[1][idx] > 10:
		plt.text(idx-0.38, (values[1][idx]/2)+summation[idx]-1, str(values[1][idx])+"%", weight='bold', fontsize=18)
	elif values[1][idx] > 3:
		plt.text(idx-0.3, (values[1][idx]/2)+summation[idx]-1, str(values[1][idx])+"%", weight='bold', fontsize=18)	
	summation[idx] += values[1][idx]
plt.bar(ind, values[2], bottom=summation, color=colors[2], align='center', label='Sadness', linewidth=1.5)
for idx, value in enumerate(summation):
	if values[2][idx] > 10:
		plt.text(idx-0.38, (values[2][idx]/2)+summation[idx]-1, str(values[2][idx])+"%", weight='bold', fontsize=18)
	elif values[2][idx] > 3:
		plt.text(idx-0.3, (values[2][idx]/2)+summation[idx]-1, str(values[2][idx])+"%", weight='bold', fontsize=18)
	summation[idx] += values[2][idx]
plt.bar(ind, values[3], bottom=summation, color=colors[3], align='center', label='Surprise', linewidth=1.5)
for idx, value in enumerate(summation):
	if values[3][idx] > 10:
		plt.text(idx-0.38, (values[3][idx]/2)+summation[idx]-1, str(values[3][idx])+"%", weight='bold', fontsize=18)
	elif values[3][idx] > 3:
		plt.text(idx-0.3, (values[3][idx]/2)+summation[idx]-1, str(values[3][idx])+"%", weight='bold', fontsize=18)
plt.xticks(ind, signals, fontsize=24,  weight="bold")
plt.ylim([0, 100.1])
plt.yticks(fontsize=24)
plt.ylabel('Percentage (%)', weight="bold", fontsize=24)
plt.grid(True)
#plt.legend(bbox_to_anchor=(1.6, 1.2), ncol=4, fontsize=30)
plt.tight_layout()
plt.savefig(data_path+title+".png", bbox_inches="tight")
#plt.savefig(data_path+"legend"+".png", bbox_inches="tight")
plt.cla()
os.chmod(data_path+title+".png", stat.S_IRWXO)

