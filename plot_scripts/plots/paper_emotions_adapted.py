import sys
import matplotlib.pyplot as plt
from datetime import datetime
import os, stat
import numpy as np

data_path =  "/home/marcosm/Dropbox/MIRO/Graphs/Emotions/"

if not os.path.exists(data_path):
    os.makedirs(data_path)

title = "Adaptation"

summation = [0,0,0,0,0,0,0]

signals = ["FEng", "TEng", "GEng", "NU", "DwG", "DwT", "FDis"]
#signals = ["FEng", "NU", "FDis"]
values = [[0, 0, 0, 0, 8.1, 7.5, 8.8], [98.9, 98.7, 99.1, 99.2, 12.7, 90.0, 12.6], [0, 0, 0, 0, 79.2, 1.1, 78.6], [1.1, 1.3, 0.5, 0.8, 0.6, 1.5, 0.8]]
emotions = ["Calmness", "Joy", "Sadness", "Surprise"]
colors = ["lightgrey","coral", "royalblue", "deepskyblue"]

ind = [x for x, _ in enumerate(signals)]

plt.figure(figsize=(8,8))
for idx, value in enumerate(summation):
	if values[0][idx] > 10:
		plt.text(idx-0.38, (values[0][idx]/2)+summation[idx]-1, str(values[0][idx])+"%", weight='bold', fontsize=14)
	elif values[0][idx] > 3:
		plt.text(idx-0.28, (values[0][idx]/2)+summation[idx]-1, str(values[0][idx])+"%", weight='bold', fontsize=14)
	summation[idx] += values[0][idx]
plt.bar(ind, summation, color=colors[0], width=0.8, align='center', label='Calmness')
plt.bar(ind, values[1], bottom=summation, width=0.8, color=colors[1], align='center', label='Joy')
for idx, value in enumerate(summation):
	if values[1][idx] > 10:
		plt.text(idx-0.38, (values[1][idx]/2)+summation[idx]-1, str(values[1][idx])+"%", weight='bold', fontsize=14)
	elif values[1][idx] > 3:
		plt.text(idx-0.28, (values[1][idx]/2)+summation[idx]-1, str(values[1][idx])+"%", weight='bold', fontsize=14)	
	summation[idx] += values[1][idx]
plt.bar(ind, values[2], bottom=summation, color=colors[2], align='center', label='Sadness', width=0.8)
for idx, value in enumerate(summation):
	if values[2][idx] > 10:
		plt.text(idx-0.38, (values[2][idx]/2)+summation[idx]-1, str(values[2][idx])+"%", weight='bold', fontsize=14)
	elif values[2][idx] > 3:
		plt.text(idx-0.28, (values[2][idx]/2)+summation[idx]-1, str(values[2][idx])+"%", weight='bold', fontsize=14)
	summation[idx] += values[2][idx]
plt.bar(ind, values[3], bottom=summation, color=colors[3], align='center', label='Surprise', width=0.8)
for idx, value in enumerate(summation):
	if values[3][idx] > 10:
		plt.text(idx-0.38, (values[3][idx]/2)+summation[idx]-1, str(values[3][idx])+"%", weight='bold', fontsize=14)
	elif values[3][idx] > 3:
		plt.text(idx-0.28, (values[3][idx]/2)+summation[idx]-1, str(values[3][idx])+"%", weight='bold', fontsize=14)
plt.xticks(ind, signals, fontsize=18,  weight="bold")
plt.ylim([0, 100.1])
plt.ylabel('Percentage (%)', weight="bold", fontsize=18)
plt.grid(True)
plt.yticks(fontsize=20)
#plt.legend(bbox_to_anchor=(0.975, 1.1), ncol=4, fontsize=16)
plt.tight_layout()
plt.savefig(data_path+title+".png", bbox_inches="tight")
plt.cla()
os.chmod(data_path+title+".png", stat.S_IRWXO)

