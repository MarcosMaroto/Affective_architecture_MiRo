####################################
# FILE WHICH PLOTS THE MOTIVATIONAL PERCENTAGES
# FOR THE FILE MOTIVATIONAL TABLES ADAPTED MOTIVATIONS
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
files = ["motivations_tables_adapted.csv", "motivations_tables_adapted_real.csv"]

# Path were data is saved
save_path = "/home/marcosm/Results/plots/motivations/"
saved_files = ["motivations_adapted.pdf", "motivations_adapted_real.pdf"]

# Style params
top_letters = ["a) ", "b) ", "c) ", "d) ", "e) ", "f) ", "g) ", "h) "]
fontsizes = [16, 24]

for fi, f in enumerate(files):
	# Save csv as pandas
	data = pd.read_csv(path+f, index_col=0)
	# Get user profiles
	robot_profiles = list(data.columns)
	# Get robot profiles
	user_profiles = list(data.index)

	# Define figure properties
	fig = plt.figure(figsize=(9, 9), tight_layout=True)
	# Colors used for each mot
	colors = {"None": "lightgrey", "Socialise": "red", "Rest": "dodgerblue", "Play": "lime"}
	mot = ["Play", "Rest", "Socialise", "None"]
	# Set labels
	if len(user_profiles) == 7:
		labels = ["Engaged in\nall modalities (FEng)", "Mostly Engaged\nin all modalities\nspecially with Touch (TEng)", "Mostly Engaged\nin all modalities\nspecially with Game (GEng)", "Mostly Engaged\nin all modalities (NU)", "Disengaged in\nall modalities\nexcepting Touch (DwT)", "Disengaged in\nall modalities\nexcepting Game (DwG)", "Disengaged in\nall modalities (FDis)"]
	elif len(user_profiles) == 3:
		labels = ["Engaged in\nall modalities (FEng)", "Mostly Engaged\nin all modalities (NU)", "Disengaged\nin all modalities (FDis)"]

	# Define subplot
	ax = fig.add_subplot(1,1,1)
	# Define axis ticks
	ax.set_xticks([a for a in range(len(user_profiles))])
	ax.set_xticklabels(user_profiles, fontsize=24)
	ax.set_yticklabels(np.arange(0, 101, 20), fontsize=18)
	ax.set_ylim([0, 100])
	ax.set_ylabel("Percentage of time", fontsize=20, weight="bold")
	ax.set_xlabel("User profiles", fontsize=fontsizes[fi]+10, weight="normal", labelpad=8)
	ax.grid(axis="y")

	# Define x axis as user profile
	for i, up in enumerate(user_profiles):
		# Get cell values of dataframe
		value = data["0.001"].loc[up]
		# Split by newline
		split = value.split("\n")
		split.reverse()
		# Get numerical values
		summation = 0
		aux_sum = 0
		for s in split:
			e, ev = s.split(":")
			# Normalize values
			aux_sum += float(ev)
			if aux_sum > 100:
				ev = float(ev) - (aux_sum-100)
			else:
				ev = float(ev)
			# Plot on axis
			ax.bar(i, ev, bottom=summation, width=0.8, align='center', color=colors[e], ec="k", label=up)
			# Write text if pct is significant
			if ev > 5:
				ax.annotate(str(np.format_float_positional(ev, 3, trim='-'))+"%", (i, summation+(ev/2.0)), ha="center", va="center", fontsize=fontsizes[fi], weight="bold")
			# Save value
			summation += ev

	# Set legend in last axes
	patches = [Rectangle((0, 0), 2, 2, fc=colors[c], ec="k") for c in mot]
	ax.legend(patches, list(mot), bbox_to_anchor=(0,1.02,1,0.2), fancybox=True, loc='lower left', prop={'size': 20}, ncol=4, mode="expand")

	# Save figure
	plt.savefig(save_path+saved_files[fi])
	plt.close("all")