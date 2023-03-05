####################################
# FILE WHICH LOADS THE COMFORT SIGNAL FOR EACH USER-ROBOT PROFILE COMBINATION
# AFTER, CREATES THE ASSOCIATED CSV FILE FOR EACH SET OF FOLDERS
# BY LAST, CREATES DE NECESSARY PLOTS
####################################

# Import libs
import numpy as np
import pandas as pd
import random
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import sys
import os


# GENERATE SIMULATION FIXED DATA

# Walk through the folders and read txt files associated to Comfort signals
save_folder = "/home/marcosm/Results/plots/csv files/"
filename = "emotion-signal_values.txt"

robot_profiles = ["nD", "MnD", "NR", "MD", "FD"]
user_profiles = ["FEng", "TEng", "GEng", "NU", "DwT", "DwG", "FDis"]

# Define folder properties
target_path = "/home/marcosm/Results/"
folders = ["Simulation/No adaptation", "Simulation/Adaptation", "Real/No adaptation", "Real/Adaptation"]
save_extensions = ["_simulation_fixed.csv", "_simulation_adapted.csv", "_real_fixed.csv", "_real_adapted.csv"]
# Define files properties
filenames = ["emotions/emotion-signal_values.txt", "emotions/emotion-emotion_signal.txt"]
signals = ["Value2", "Value"]
saved_files = ["comfort", "es"]

# Method used to define a csv with data from files
def define_csv(filename, signal, folder_name, save_filename):
	# Get list of directories in target path
	path = target_path+folder_name
	directories = os.listdir(path)

	# Loop all dirs
	aux_dataframe = pd.DataFrame()
	for d in directories:
		# Check if dir name matches user_profile
		if d in user_profiles:
			# Get directories inside subfolder
			subdirectories = os.listdir(path+"/"+d+"/")
			if any([True for x in subdirectories if x in robot_profiles]):
				# Loop all subdirecotires
				for s in subdirectories:
					# If subdir is in robot_profiles
					if s in robot_profiles:
						# Get a random dir
						pdir = random.choice(os.listdir(path+"/"+d+"/"+s+"/"))

						# Check if pdir cointais file
						possible_files = [path+"/"+d+"/"+s+"/"+pdir+"/"+filename, path+"/"+d+"/"+s+"/"+filename]
						for f in possible_files:
							if os.path.isfile(f):
								# Read file as csv
								data = pd.read_csv(f, delimiter=";", names=["Timestamp", "Value","Value2","Value3","Value4"], header=None)
								data = data.iloc[1:]
								# Define an auxiliary dataset
								aux_dataframe[d+"_"+s+"_time"] = data["Timestamp"].copy()
								aux_dataframe[d+"_"+s] = data[signal].copy()
								break
			else:
				# Check if pdir cointais file
				possible_files = [path+"/"+d+"/"+filename]
				for f in possible_files:
					if os.path.isfile(f):
						# Read file as csv
						data = pd.read_csv(f, delimiter=";", names=["Timestamp", "Value","Value2","Value3","Value4"], header=None)
						data = data.iloc[1:]
						# Define an auxiliary dataset
						aux_dataframe[d+"_time"] = data["Timestamp"].copy()
						aux_dataframe[d] = data[signal].copy()
						break

	# Save dataframe
	aux_dataframe.to_csv(save_folder+save_filename, index=False)

# Generate data
for i, s in enumerate(signals):
	# Generate all types of data
	for u, e in enumerate(save_extensions):
		define_csv(filenames[i], s, folders[u], saved_files[i]+e)