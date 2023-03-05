####################################
# FILE WHICH PLOTS THE EMOTIONAL SIGNAL
# FROM THE COMFORT FIXED VALUES CSV FILE
####################################


# Import libs
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from datetime import datetime
import sys
import os

# Running mean
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

# Path to the csv file
path = "/home/marcosm/Results/plots/csv files/"
# Files to read
files = ["val_simulation_fixed.csv", "val_real_fixed.csv"]

# Style params
top_letters = ["a) ", "b) ", "c) ", "d) ", "e) ", "f) ", "g) ", "h) "]
fontsizes = [11, 18]
title_fontsizes = [18, 25]

# Path were data is saved
save_path = "/home/marcosm/Results/plots/valence/"
saved_files_user = ["val_fixed_user.pdf", "val_fixed_user_real.pdf"]
saved_files_robot = ["val_fixed_robot.pdf", "val_fixed_robot_real.pdf"]

# Create dir if it does not exist
if not os.path.exists(save_path):
	os.makedirs(save_path)

# Loop for files in list
for fi, f in enumerate(files):
	# Save csv as pandas
	data = pd.read_csv(path+f, index_col=False)

	# Get columns names
	columns = data.columns
	# Get profiles
	user_profiles = ["FEng", "TEng", "GEng", "NU", "DwT", "DwG", "FDis"]
	robot_profiles = ["nD", "MnD", "NR", "MD", "FD"]
	aux_up = list()
	aux_rp = list()
	for c in columns:
		name = c.split("_")
		if len(name) == 2:
			if not name[0] in aux_up:
				aux_up.append(name[0])
			if not name[1] in aux_rp:
				aux_rp.append(name[1])

	user_profiles = [v for v in user_profiles if v in aux_up]
	robot_profiles = [v for v in robot_profiles if v in aux_rp]

	# Define figure properties
	fig = plt.figure(figsize=(24, 10), tight_layout=True)
	# Colors used for each emotion
	colors = {"nD": "blue", "MnD": "red", "NR": "magenta", "MD": "lime", "FD": "slategrey"}
	# Set labels
	labels = {"FEng": "Engaged in\nall modalities (FEng)", "TEng": "Mostly Engaged\nin all modalities\nspecially with Touch (TEng)", "GEng": "Mostly Engaged\nin all modalities\nspecially with Game (GEng)", "NU": "Mostly Engaged\nin all modalities (NU)", "DwT": "Disengaged in\nall modalities\nexcepting Touch (DwT)", "DwG": "Disengaged in\nall modalities\nexcepting Game (DwG)", "FDis": "Disengaged in\nall modalities (FDis)"}

	# Define x axis as user profile
	for i, up in enumerate(user_profiles):
		# Define subplot
		if len(user_profiles) > 4:
			rows = 2
			columns = (len(user_profiles)+1)/2.0
			ax = fig.add_subplot(rows, columns, i+1, label=up)
		else:
			rows = 1
			columns = len(user_profiles)
			ax = fig.add_subplot(rows, columns, i+1, label=up)

		# X axis
		if "real" in f:
			ax.set_xlim([0, 20])
			ax.set_xticks(np.arange(0, 21, 5))
		else:
			ax.set_xlim([0, 600])
			ax.set_xticks(np.arange(0, 601, 100))

		ax.tick_params(axis="x", labelsize=title_fontsizes[fi])

		# Y axis
		ax.set_ylim([0, 100])
		if i == 0 or i == 4:
			ax.set_ylabel("Valence", fontsize=title_fontsizes[fi]+2, weight="bold")
			ax.set_yticklabels(np.arange(0, 101, 20), fontsize=fontsizes[fi])
		else:
			ax.set_yticklabels([])

		# Define axis ticks
		ax.set_xlabel("Time (m)", fontsize=title_fontsizes[fi]+2, weight="bold")
		ax.set_title(top_letters[i]+labels[up], fontsize=title_fontsizes[fi]+5, weight="bold", pad=10)
		ax.grid()

		# Get values of each emotion
		for v, rp in enumerate(robot_profiles):
			# Get cell values of dataframe
			time_values = list(data[up+"_"+rp+"_time"].values)
			values = list(data[up+"_"+rp].values)
			# Convert axis
			init_t = datetime.strptime(time_values[0], '%d/%m/%Y  %H:%M:%S.%f')
			prev_value = None
			aux_values = list()
			aux_time_values = list()
			for idx, t in enumerate(time_values):
				if type(t) is str and prev_value != values[idx]:
					if "real" in f:
						aux_time_values.append((datetime.strptime(t, '%d/%m/%Y  %H:%M:%S.%f') - init_t).total_seconds()/60.0)
					else:
						aux_time_values.append((datetime.strptime(t, '%d/%m/%Y  %H:%M:%S.%f') - init_t).total_seconds()*100/60.0)
					aux_values.append(float(values[idx]))
					prev_value = values[idx]

			ax.plot(aux_time_values, running_mean(aux_values, 1000), color=colors[rp], lw=3)

	# Set legend in last axes
	patches = [Rectangle((0, 0), 0.5, 2, fc=colors[rp], ec="k") for rp in robot_profiles]
	if len(user_profiles) > 4:
		ax = fig.add_subplot(2,(len(user_profiles)+1)/2.0,i+2, label=up)
		ax.set_yticklabels([])
		ax.set_xticklabels([])
		ax.axis('off')
		ax.legend(patches, robot_profiles, fancybox=True, loc='center', prop={'size': 25}, ncol=2)
		# Save figure
		plt.savefig(save_path+saved_files_user[fi])
	else:
		lgd = fig.legend(patches, robot_profiles, bbox_to_anchor=(0.27,0.94,0.5,0.2), fancybox=True, loc='center', prop={'size': 30}, ncol=4, mode="expand")
		# Save figure
		plt.savefig(save_path+saved_files_user[fi], bbox_extra_artists=(lgd,), bbox_inches='tight')

###########################################################
# Plot same information but inverting axis
###########################################################

	plt.close('all')

	colors = {"FEng": "blue", "TEng": "orange", "GEng": "magenta", "NU": "lime", "DwT": "slategrey", "DwG": "aqua", "FDis": "red"}
	# Set labels
	labels = {"nD": "Non-Dependent\non the interaction (nD)", "MnD": "Mostly non-Dependent\non the interaction (MnD)", "NR": "Equally Dependent\non internal and\nexternal factors (NR)", "MD": "Mostly Dependent\non the interaction (MD)", "FD": "Fully Dependent\non the interaction (FD)"}

	# Define figure properties
	fig = plt.figure(figsize=(24, 10), tight_layout=True)

	# Define x axis as user profile
	for i, rp in enumerate(robot_profiles):
		# Define subplot
		if len(robot_profiles) > 4:
			rows = 2
			columns = (len(robot_profiles)+1)/2.0
			ax = fig.add_subplot(rows, columns, i+1, label=up)
		else:
			rows = 1
			columns = len(robot_profiles)
			ax = fig.add_subplot(rows, columns, i+1, label=up)

		# X axis
		if "real" in f:
			ax.set_xlim([0, 20])
			ax.set_xticks(np.arange(0, 21, 5))
		else:
			ax.set_xlim([0, 600])
			ax.set_xticks(np.arange(0, 601, 100))

		ax.tick_params(axis="x", labelsize=title_fontsizes[fi])

		# Y axis
		ax.set_ylim([0, 100])
		if i == 0 or i == 3:
			ax.set_ylabel("Valence", fontsize=title_fontsizes[fi]+2, weight="bold")
			ax.set_yticklabels(np.arange(0, 101, 20), fontsize=fontsizes[fi])
		else:
			ax.set_yticklabels([])

		# Define axis ticks
		ax.set_xlabel("Time (m)", fontsize=title_fontsizes[fi]+2, weight="bold")
		ax.set_title(top_letters[i]+labels[rp], fontsize=title_fontsizes[fi]+5, weight="bold", pad=10)
		ax.grid()

		# Get values of each emotion
		for v, up in enumerate(user_profiles):
			# Get cell values of dataframe
			time_values = list(data[up+"_"+rp+"_time"].values)
			values = list(data[up+"_"+rp].values)
			# Convert axis
			init_t = datetime.strptime(time_values[0], '%d/%m/%Y  %H:%M:%S.%f')
			prev_value = None
			aux_values = list()
			aux_time_values = list()
			for idx, t in enumerate(time_values):
				if type(t) is str and prev_value != values[idx]:
					if "real" in f:
						aux_time_values.append((datetime.strptime(t, '%d/%m/%Y  %H:%M:%S.%f') - init_t).total_seconds()/60.0)
					else:
						aux_time_values.append((datetime.strptime(t, '%d/%m/%Y  %H:%M:%S.%f') - init_t).total_seconds()*100/60.0)
					aux_values.append(float(values[idx]))
					prev_value = values[idx]

			# Plot signal
			ax.plot(aux_time_values, running_mean(aux_values, 1000), color=colors[up], lw=3)

	# Set legend in last axes
	patches = [Rectangle((0, 0), 2, 0.5, fc=colors[up], ec="k") for up in user_profiles]
	if len(robot_profiles) > 4:
		ax = fig.add_subplot(2,(len(robot_profiles)+1)/2.0,i+2, label=up)
		ax.set_yticklabels([])
		ax.set_xticklabels([])
		ax.axis('off')
		ax.legend(patches, user_profiles, fancybox=True, loc='center', prop={'size': 25}, ncol=2)
		# Save figure
		plt.savefig(save_path+saved_files_robot[fi])
	else:
		lgd = fig.legend(patches, user_profiles, bbox_to_anchor=(0.27,0.94,0.5,0.2), fancybox=True, loc='center', prop={'size': 30}, ncol=4, mode="expand")
		# Save figure
		plt.savefig(save_path+saved_files_robot[fi], bbox_extra_artists=(lgd,), bbox_inches='tight')