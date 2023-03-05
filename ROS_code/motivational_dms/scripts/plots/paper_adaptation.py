import sys
import matplotlib.pyplot as plt
from datetime import datetime
import os
import stat
import numpy as np

data_path =  "/home/marcosm/Dropbox/MIRO/Graphs/Adaptation/"

if not os.path.exists(data_path):
    os.makedirs(data_path)

title = "AdaptationExternal"

signals = ["FEng", "TEng", "GEng", "NU", "DwG", "DwT", "FDis"]
#signals = ["FE", "N", "FD"]
values = [[88.25, 88.52, 91.96, 79.42, 39.22, 21.47, 21.41], [88.17, 88.42, 81.98, 79.57, 21.99, 65.03, 20.03], [88.25, 88.44, 81.71, 80.78, 21.65, 74.5, 22.18], [88.08, 88.56, 81.14, 80.71, 24.39, 47.86, 22.04]]
errors = [[3, 2.89, 2.89, 5.3, 11.93, 14.15, 11.85], [2.92, 2.92, 3.28, 5.44, 11.56, 17.35, 11.8], [2.95, 2.93, 3.11, 5.31, 11.45, 10.33, 12.03], [2.94, 3.01, 3.84, 5.27, 12.16, 12.11, 12.21]]
external_values = [[97.96, 97.85, 90.63, 56.28, 13.85, 25.39, 13.68], [98.2, 97.97, 93.98, 56.78, 14.36, 31.78, 13.07], [98.37, 98.05, 95.43, 58.48, 13.92, 32.05, 43.71], [98.95, 98.08, 95.55, 61.81, 14.37, 29.38, 13.68]]
external_errors = [[7.63, 8.42, 14.8, 44.36, 29.25, 39.24, 29.07], [6.54, 8.09, 11.5, 44.91, 29.81, 43.6, 29.41], [6.39, 7.85, 9.83, 45.12, 29.24, 43.71, 29.15], [5.65, 7.74, 9.19, 42.53, 29.93, 42.22, 29.17]]
emotional_values = [[98.06, 97.95, 91.23, 83.85, 24.84, 45.82, 24.63], [98.32, 98.22, 95.01, 87.42, 30.08, 71.15, 28.95], [98.65, 98.55, 96.31, 88.2, 30.06, 79.77, 30.04], [99.05, 99.01, 96.43, 89.25, 33.15, 60.02, 30.78]]
emotional_errors = [[6.65, 7.25, 11.21, 13.63, 22.44, 25.88, 22.13], [5.03, 5.56, 8.14, 11.84, 24.64, 21.37, 24.74], [4.34, 4.79, 6.93, 11.37, 24.25, 14.48, 24.27], [3.4, 3.62, 6.72, 11.23, 24.19, 24.29, 24.34]]

up_error = list()
low_error = list()

for idx, value in enumerate(external_values):
	up_aux = list()
	low_aux = list()
	for p, data in enumerate(value):
		if (data + external_errors[idx][p]) >= 100:
			up_aux.append(100-data)
		else:
			up_aux.append(external_errors[idx][p])
		if (data - external_errors[idx][p] <= 0):
			low_aux.append(-(0-data))
		else:
			low_aux.append(external_errors[idx][p])
	up_error.append(up_aux)
	low_error.append(low_aux)

alphas = ["0.01", "0.05", "0.1", "0.5"]

ind = [0.5, 1, 1.5, 2, 2.5, 3, 3.5]

colors = ["lime", "coral", "royalblue", "deepskyblue"]

plt.figure(figsize=(6,8))
for idx, value in enumerate(external_values):
	plt.subplot(411+idx)
	plt.errorbar(ind, value, yerr=[low_error[idx], up_error[idx]], mew=2, ms=8, fmt='o', color='limegreen', ecolor='g', capthick=3, elinewidth=3, capsize=8, zorder=5, linestyle='dotted', linewidth=2)
	plt.grid(True, linewidth=1.5)
	plt.xlim([0, 4])
	plt.xticks(ind, signals, fontsize=14,  weight="bold")
	plt.yticks(fontsize=16)
	plt.ylim([0, 100])
	plt.ylabel(r"af="+alphas[idx], weight="bold", fontsize=14)

plt.tight_layout()
plt.savefig(data_path+title+".png", bbox_inches="tight", dpi=400)
plt.cla()
os.chmod(data_path+title+".png", stat.S_IRWXO)
