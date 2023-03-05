####################################
# File that plots the adaptive rates
####################################

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from datetime import datetime
from math import pi
import sys
import os


# SIMULATED PLOTS
fig = plt.figure(figsize=(24, 10), tight_layout=True)

path = "/home/marcosm/Results/plots/csv files/adaptive/Sim/"
save_path = "/home/marcosm/Results/plots/rates/adaptive/"

user_profiles = ["FEng", "TEng", "GEng", "NU", "DwT", "DwG", "FDis"]

colors = {"FEng": "blue", "TEng": "orange", "GEng": "magenta", "NU": "lime", "DwT": "slategrey", "DwG": "aqua", "FDis": "red"}

patches = list()

for i, up in enumerate(user_profiles):
	dates = list()
	values = list()
	data = pd.read_csv(path+up+".txt", index_col=0)
	for idate, l in enumerate(data.index):
		date, value = l.split(";")
		date = date.split("  ")[1]
		date =  datetime.strptime(date, '%H:%M:%S.%f')
		if idate == 0:
			initial_time = date.hour*3600+date.minute*60+date.second+date.microsecond/1000000.0
		date = date.hour*3600+date.minute*60+date.second+date.microsecond/1000000.0 - initial_time
		date = date*100/60.0
		dates.append(date)
		values.append(float(value))
	
	# Define subplot
	ax = fig.add_subplot(2, 4, i+1)
	ax.set_ylim([-0.03, 1.03])
	ax.set_yticks([0, 0.2, 0.4, 0.6, 0.8, 1.0])
	if i == 0 or i == 4:
		ax.set_ylabel("Internal Param", fontsize=25, weight="bold")
		ax.set_yticklabels([0, 0.2, 0.4, 0.6, 0.8, 1.0], fontsize=20)
	else:
		ax.set_yticklabels([])
	ax.set_xlim([0, 600])
	ax.set_xticks(np.arange(0, 601, 100))
	ax.set_xticklabels(np.arange(0, 601, 100), fontsize=20)
	ax.set_xlabel("Time (m)", fontsize=25, weight="bold")
	ax.grid()
	# Plot
	ax.plot(dates, values, color=colors[up], lw=4)
	patches.append(Rectangle((0, 0), 2, 2, fc=colors[up], ec="k"))

ax = fig.add_subplot(2,(len(user_profiles)+1)/2.0,i+2, label=up)
ax.set_yticklabels([])
ax.set_xticklabels([])
ax.axis('off')
ax.legend(patches, user_profiles, fancybox=True, loc='center', prop={'size': 25}, ncol=2)
#fig.legend(patches, user_profiles, bbox_to_anchor=(0.4, 1.05), fancybox=True, loc='center', prop={'size': 40}, ncol=3, handletextpad=0.9, markerscale=1.5)
plt.savefig(save_path+"adaptive_sim.pdf", bbox_inches="tight")

# REAL PLOTS
# Create figure
fig = plt.figure(figsize=(40, 12), tight_layout=True)

# Path to the csv file
path = "/home/marcosm/Results/plots/csv files/adaptive/Real/"
save_path = "/home/marcosm/Results/plots/rates/adaptive/"

user_profiles = ["FEng", "NU", "FDis"]

colors = ["blue", "lime", "red"]

patches = list()

# Create figure
fig = plt.figure(figsize=(40, 12), tight_layout=True)

for i, up in enumerate(user_profiles):
	dates = list()
	values = list()
	data = pd.read_csv(path+up+".txt", index_col=0)
	for idate, l in enumerate(data.index):
		date, value = l.split(";")
		date = date.split("  ")[1]
		date =  datetime.strptime(date, '%H:%M:%S.%f')
		if idate == 0:
			initial_time = date.hour*3600+date.minute*60+date.second+date.microsecond/1000000.0
		date = date.hour*3600+date.minute*60+date.second+date.microsecond/1000000.0 - initial_time
		date = date/60.0
		dates.append(date)
		values.append(float(value))
	# Define subplot
	ax = fig.add_subplot(1, 4, i+1)
	ax.set_xlim(0, 20)
	ax.set_ylim(-0.01, 1.01)
	ax.grid()
	if i == 0:
		ax.set_yticks([0, 0.2, 0.4, 0.6, 0.8, 1.0])
		ax.set_yticklabels([0, 0.2, 0.4, 0.6, 0.8, 1.0], fontsize=30)
		ax.set_ylabel("Internal Param", fontsize=40, labelpad=15, weight="bold")
	elif i in [1, 2]:
		ax.set_yticks([0, 0.2, 0.4, 0.6, 0.8, 1.0])
		ax.set_yticklabels([])
	# elif i == 2:
	# 	ax.yaxis.tick_right()
	# 	ax.yaxis.set_label_position("right")
	# 	ax.set_yticks([0, 0.2, 0.4, 0.6, 0.8, 1.0])
	# 	ax.set_yticklabels([0, 0.2, 0.4, 0.6, 0.8, 1.0], fontsize=25)
	# 	ax.set_ylabel("External Param", fontsize=40, rotation=-90, labelpad=60)
	# 	ax.invert_yaxis()
	ax.set_xticks([0, 4, 8, 12, 16, 20])
	ax.set_xticklabels([0, 4, 8, 12, 16, 20], fontsize=30)
	ax.set_xlabel("Time(m)", fontsize=30, labelpad=15, weight="bold")
	# Plot
	ax.plot(dates, values, color=colors[i], lw=5)
	patches.append(Rectangle((0, 0), 2, 2, fc=colors[i], ec="k"))

fig.legend(patches, user_profiles, bbox_to_anchor=(0.4, 1.05), fancybox=True, loc='center', prop={'size': 40}, ncol=3, handletextpad=0.9, markerscale=1.5, columnspacing=4)
plt.savefig(save_path+"adaptive.pdf", bbox_inches="tight")