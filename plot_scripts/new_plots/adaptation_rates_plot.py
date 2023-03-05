####################################
# File which plots the adaptation rates mean and std
# in order to compare them
####################################


# Import libs
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from datetime import datetime
from math import pi
import sys
import os

# Path to the csv file
path = "/home/marcosm/Results/plots/csv files/"
save_path = "/home/marcosm/Results/plots/rates/"
# Create dir if it does not exist
if not os.path.exists(save_path):
	os.makedirs(save_path)

# Read datafiles
es_datafile = pd.read_csv(path+"val_rates.csv", index_col=0)
is_datafile = pd.read_csv(path+"is_rates.csv", index_col=0)
ex_datafile = pd.read_csv(path+"ex_rates.csv", index_col=0)

signals = ["Valence", "Physiological signal", "Quality of interaction"]
dataframes = [es_datafile, is_datafile, ex_datafile]
rates_labels = ["0.01", "0.05", "0.1", "0.5"]
user_profiles = ["FEng", "TEng", "GEng", "NU", "DwT", "DwG", "FDis"]
profiles_labels = {"FEng": "Fully Engaged", "TEng": "Touch Engaged", "GEng": "Game Engaged", "NU": "Neutral User", "DwT": "Disengaged with Touch", "DwG": "Disengaged with Game", "FDis": "Fully Disengaged"}
colors = ["b", "r", "g"]

# Create figure
fig = plt.figure(figsize=(40, 12), tight_layout=True)

c = 0# Color counter
# Loop all rates labels
for rl, ri in enumerate(rates_labels):

	# Define subplot
	ax = fig.add_subplot(1, 4, rl+1)
	# Define axis ticks
	ax.set_ylim([0, 100])
	ax.set_title("Rate " + rates_labels[rl], fontsize=40, weight="normal", pad=18)
	if rl == 0:
		ax.set_yticklabels(np.arange(0, 101, 20), fontsize=30)
		ax.set_ylabel("Valence", fontsize=50, weight="normal")
	else:
		ax.set_yticklabels([])
	if True:
		ax.set_xticks([a+1 for a in range(len(user_profiles)+1)])
		ax.set_xticklabels(user_profiles, fontsize=30, weight="normal")
		ax.set_xlabel("User profiles", fontsize=40, weight="normal", labelpad=10)
	else:
		ax.set_xticklabels([])
	
	ax.grid()
	# Loop all signals
	for si, s in enumerate(signals):
		# Loop all user profiles
		means = list()
		stds = list()
		up_error = list()
		low_error = list()
		for u, up in enumerate(user_profiles):
			value = dataframes[si][ri].loc[up]
			mean, std = value.split("\n")
			# Get values
			mean_value = float(mean.split(":")[1])
			means.append(mean_value)
			error = error_u = error_d = float(std.split(":")[1])
			stds.append(error)
			# Normalize error
			if mean_value + error > 100:
				error_u = 100 - mean_value
			if mean_value - error < 0:
				error_d = mean_value
			# Get limits
			up_error.append(error_u)
			low_error.append(error_d)

		# Plot error bars
		means = np.array(means)
		#ax.fill_between([x for x in range(1,8)], means+up_error, means-up_error, alpha=0.2, color=colors[si])
		ax.errorbar(np.arange(1,8), means, yerr=[low_error, up_error], color='b', ms=15, elinewidth=5, fmt='o', capsize=15, capthick=8)
		c += 1
		if c > 2:
			c = 0
		break

#fig.legend(["Valence"], bbox_to_anchor=(0.5, 1.04), fancybox=True, loc='center', prop={'size': 50}, ncol=1, handletextpad=0.9, markerscale=1.5)
plt.savefig(save_path+"rates_full3.pdf", bbox_inches="tight")