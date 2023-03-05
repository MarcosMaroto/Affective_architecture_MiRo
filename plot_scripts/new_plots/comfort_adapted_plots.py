####################################
# FILE WHICH PLOTS THE COMFORT SIGNAL
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
files = ["comfort_simulation_adapted.csv", "comfort_real_adapted.csv"]

# Style params
top_letters = ["a) ", "b) ", "c) ", "d) ", "e) ", "f) ", "g) ", "h) "]
fontsizes = [11, 18]

# Path were data is saved
save_path = "/home/marcosm/Results/plots/comfort/"
saved_filcomfort_user = ["comfort_adapted_user.pdf", "comfort_adapted_user_real.pdf"]

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
	aux_up = list()
	for c in columns:
		name = c.split("_")
		if len(name) == 1:
			if not name[0] in aux_up:
				aux_up.append(name[0])

	user_profiles = [v for v in user_profiles if v in aux_up]

	# Define figure properties
	fig = plt.figure(figsize=(18, 10), tight_layout=True)
	# Colors used for each emotion
	colors = {"FEng": "blue", "TEng": "orange", "GEng": "magenta", "NU": "lime", "DwT": "yellow", "DwG": "aqua", "FDis": "red"}
	
	# Set labels
	labels = {"FEng": "Engaged in\nall modalities", "TEng": "Mostly Engaged\nin all modalities\nspecially with Touch", "GEng": "Mostly Engaged\nin all modalities\nspecially with Game", "NU": "Mostly Engaged\nin all modalities", "DwT": "Disengaged in\nall modalities\nexcepting Touch", "DwG": "Disengaged in\nall modalities\nexcepting Game", "FDis": "Disengaged in\nall modalities"}
	# Define subplot
	ax = fig.add_subplot(111)
	# X axis
	if "real" in f:
		ax.set_xlim([0, 20])
		ax.set_xticks(np.arange(0, 21, 5))
	else:
		ax.set_xlim([0, 600])
		ax.set_xticks(np.arange(0, 601, 100))

	ax.tick_params(axis="x", labelsize=25)

	# Y axis
	ax.set_ylim([0, 100])
	ax.set_ylabel("Physiological stability", fontsize=30, weight="bold")
	ax.set_yticklabels(np.arange(0, 101, 20), fontsize=25)

	# Define axis ticks
	ax.set_xlabel("Time (m)", fontsize=30, weight="bold")
	ax.grid()

	# Get values of each emotion
	for v, up in enumerate(user_profiles):
		# Get cell values of dataframe
		time_values = list(data[up+"_time"].values)
		values = list(data[up].values)
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

		ax.plot(aux_time_values, running_mean(aux_values, 1000), color=colors[up], lw=4)

	# Set legend in last axes
	patches = [Rectangle((0, 0), 2, 2, fc=colors[up], ec="k") for up in user_profiles]
	ax.legend(patches, user_profiles, bbox_to_anchor=(0,1.01,1,0.2), fancybox=True, loc='lower left', prop={'size': 25}, ncol=7, mode="expand")

	# Save figure
	plt.savefig(save_path+saved_filcomfort_user[fi])