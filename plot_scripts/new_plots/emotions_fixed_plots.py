####################################
# FILE WHICH PLOTS THE EMOTIONAL PERCENTAGES
# FOR THE FILE EMOTIONAL TABLES FIXED EMOTIONS
####################################


# Import libs
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import sys
import os

# Path to the csv file
path = "/home/marcosm/Results/plots/csv files/"
# Files to read
files = ["emotions_tables.csv", "emotions_tables_real.csv"]

# Style params
top_letters = ["a) ", "b) ", "c) ", "d) ", "e) ", "f) ", "g) ", "h) "]
fontsizes = [13, 22]

# Path were data is saved
save_path = "/home/marcosm/Results/plots/emotions/"
saved_files_user = ["emotions_fixed_user.pdf", "emotions_fixed_user_real.pdf"]
saved_files_robot = ["emotions_fixed_robot.pdf", "emotions_fixed_robot_real.pdf"]

# Create dir if it does not exist
if not os.path.exists(save_path):
	os.makedirs(save_path)

# Loop for files in list
for fi, f in enumerate(files):
	# Save csv as pandas
	data = pd.read_csv(path+f, index_col=0)

	# Get user profiles
	robot_profiles = list(data.columns)
	# Get robot profiles
	user_profiles = list(data.index)

	# Define figure properties
	fig = plt.figure(figsize=(20, 10), tight_layout=True)
	# Colors used for each emotion
	colors = {"Neutral": "lightgrey", "Happiness": "coral", "Sadness": "royalblue", "Surprise": "deepskyblue"}
	emotions = ["Happiness", "Neutral", "Sadness", "Surprise"]
	emotions.reverse()
	# Set labels
	if len(user_profiles) == 7:
		labels = ["Engaged in\nall modalities (FEng)", "Mostly Engaged\nin all modalities\nspecially with Touch (TEng)", "Mostly Engaged\nin all modalities\nspecially with Game (GEng)", "Mostly Engaged\nin all modalities (NU)", "Disengaged in\nall modalities\nexcepting Touch (DwT)", "Disengaged in\nall modalities\nexcepting Game (DwG)", "Disengaged in\nall modalities (FDis)"]
	elif len(user_profiles) == 3:
		labels = ["Engaged in\nall modalities (FEng)", "Mostly Engaged\nin all modalities (NU)", "Disengaged\nin all modalities (FDis)"]

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
		ax.set_xticks([a for a in range(len(robot_profiles))])
		ax.set_xticklabels(robot_profiles, fontsize=fontsizes[fi]+5)
		ax.set_xlabel("Robot profiles", fontsize=fontsizes[fi]+10, weight="normal", labelpad=8)

		# Y axis
		ax.set_ylim([0, 100])
		if i == 0 or i == 4:
			ax.set_ylabel("Percentage of time", fontsize=fontsizes[fi]+3, weight="bold")
			ax.set_yticklabels(np.arange(0, 101, 20), fontsize=fontsizes[fi])
		else:
			ax.set_yticklabels([])

		ax.set_title(top_letters[i]+labels[i], fontsize=fontsizes[fi]+5, weight="bold", pad=10)
		ax.grid(axis="y")

		# Get values of each emotion
		for v, rp in enumerate(robot_profiles):
			# Get cell values of dataframe
			value = data[rp].loc[up]
			# Split by newline
			split = value.split("\n")
			# Get numerical values
			summation = 0
			aux_sum = 0
			for s in split:
				e, ev = s.split(":")
				if e == "Joy":
					e = "Happiness"
				# Normalize values
				aux_sum += float(ev)
				if aux_sum > 100:
					ev = float(ev) - (aux_sum-100)
				else:
					ev = float(ev)
				# Plot on axis
				ax.bar(v, ev, bottom=summation, width=0.8, align='center', label=rp, color=colors[e], ec="k")
				# Write text if pct is significant
				if ev > 5:
					ax.annotate(str(np.format_float_positional(ev, 3, trim='-'))+"%", (v, summation+(ev/2.0)), ha="center", va="center", fontsize=fontsizes[fi], weight="bold")
				# Save value
				summation += ev

	# Set legend in last axes
	patches = [Rectangle((0, 0), 2, 2, fc=colors[c], ec="k") for c in emotions]
	if len(user_profiles) > 4:
		ax = fig.add_subplot(2,(len(user_profiles)+1)/2.0,i+2, label=up)
		ax.set_yticklabels([])
		ax.set_xticklabels([])
		ax.axis('off')
		ax.legend(patches, emotions, fancybox=True, loc='center', prop={'size': 25})
		# Save figure
		plt.savefig(save_path+saved_files_user[fi])
	else:
		lgd = fig.legend(patches, emotions, bbox_to_anchor=(0.18,0.92,0.7,0.2), fancybox=True, loc='center', prop={'size': 25}, ncol=4, mode="expand")
		# Save figure
		plt.savefig(save_path+saved_files_user[fi], bbox_extra_artists=(lgd,), bbox_inches='tight')

###########################################################
# Plot same information but inverting axis
###########################################################

	plt.close('all')

	# Define figure properties
	fig = plt.figure(figsize=(20, 10), tight_layout=True)

	# Set labels
	if len(robot_profiles) == 5:
		labels = ["Non-Dependent\non the interaction (nD)", "Mostly non-Dependent\non the interaction (MnD)", "Equally Dependent\non internal and\nexternal factors (NR)", "Mostly Dependent\non the interaction (MD)", "Fully Dependent\non the interaction (FD)"]
	elif len(robot_profiles) == 3:
		labels = ["Mostly non-Dependent\non the interaction (nD)", "Equally Dependent\non internal and\nexternal factors (NR)", "Mostly Dependent\non the interaction (MD)"]

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
		ax.set_xticks([a for a in range(len(user_profiles))])
		ax.set_xticklabels(user_profiles, fontsize=fontsizes[fi]+5)
		ax.set_xlabel("User profiles", fontsize=fontsizes[fi]+10, weight="normal", labelpad=8)

		# Y axis
		ax.set_ylim([0, 100])
		if i == 0 or i == 3:
			ax.set_ylabel("Percentage of time", fontsize=fontsizes[fi]+3, weight="bold")
			ax.set_yticklabels(np.arange(0, 101, 20), fontsize=fontsizes[fi])
		else:
			ax.set_yticklabels([])

		ax.set_title(top_letters[i]+labels[i], fontsize=fontsizes[fi]+5, weight="bold", pad=10)
		ax.grid(axis="y")

		# Get values of each emotion
		for v, up in enumerate(user_profiles):
			# Get cell values of dataframe
			value = data[rp].loc[up]
			# Split by newline
			split = value.split("\n")
			# Get numerical values
			summation = 0
			aux_sum = 0
			for s in split:
				e, ev = s.split(":")
				if e == "Joy":
					e = "Happiness"
				# Normalize values
				aux_sum += float(ev)
				if aux_sum > 100:
					ev = float(ev) - (aux_sum-100)
				else:
					ev = float(ev)
				# Plot on axis
				ax.bar(v, ev, bottom=summation, width=0.8, align='center', label=up, color=colors[e], ec="k")
				# Write text if pct is significant
				if ev > 5:
					ax.annotate(str(np.format_float_positional(ev, 3, trim='-'))+"%", (v, summation+(ev/2.0)), ha="center", va="center", fontsize=fontsizes[fi], weight="bold")
				# Save value
				summation += ev

	# Set legend in last axes
	patches = [Rectangle((0, 0), 2, 2, fc=colors[c], ec="k") for c in emotions]
	if len(robot_profiles) > 4:
		ax = fig.add_subplot(2, (len(robot_profiles)+1)/2.0,i+2, label=up)
		ax.set_yticklabels([])
		ax.set_xticklabels([])
		ax.axis('off')
		ax.legend(patches, emotions, fancybox=True, loc='center', prop={'size': 25})
		# Save figure
		plt.savefig(save_path+saved_files_robot[fi])
	else:
		lgd = fig.legend(patches, emotions, bbox_to_anchor=(0.18,0.92,0.7,0.2), fancybox=True, loc='center', prop={'size': 25}, ncol=4, mode="expand")
		# Save figure
		plt.savefig(save_path+saved_files_robot[fi], bbox_extra_artists=(lgd,), bbox_inches='tight')