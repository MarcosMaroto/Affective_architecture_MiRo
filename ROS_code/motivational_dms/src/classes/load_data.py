#!/usr/bin/python
# +*+ coding: utf+8 +*+

__author__ = "Marcos Maroto Gómez"
__copyright__ = "Social Robots Group. Robotics Lab. University Carlos III of Madrid"
__credits__ = ["Marcos Maroto Gómez"]
__license__ = "LEUC3M v1.0"
__version__ = "0.0.0"
__maintainer__ = "Marcos Maroto Gómez"
__email__ = "marmarot@ing.uc3m.es"
__status__ = "Development"

"""
	@brief: Class load category data from xml files
"""

# Importing libs
import xml.etree.ElementTree as ET
import rospy
import rospkg
import os
import random
import yaml

from geometry_msgs.msg import Pose2D, Twist

pkg_name = 'motivational_dms'
rospack = rospkg.RosPack()

#-----------------------------------------------------------------------------------#
# Class: XmlImport
#-----------------------------------------------------------------------------------#

class DataImport():

	def __init__(self):

		# Path to xml files
		self._data_path = rospack.get_path(pkg_name) + "/data/config/"
		
	#-----------------------------------------------------------------------------------#
	# Class methods
	#-----------------------------------------------------------------------------------#

	def load_emotions(self):
		
		emotions = []

		root = yaml.load(open(self._data_path + "emotions.yaml"))

		# Load info from xml files
		for value in root.values():
			emotions = value

		return emotions

	def load_signal_params(self):
		
		params = []

		root = yaml.load(open(self._data_path + "signal_params.yaml"))

		# Load info from xml files
		for value in root.values():
			params = value

		return params

	def load_behavior_units(self):
		
		behavior_units = list()
		root = yaml.load(open(self._data_path + "behavior_units.yaml"))

		# Load info from xml files
		for value in root.values():
			behavior_units = value

		return behavior_units

	def load_behavior_info(self):
		
		behaviors = list()
		root = yaml.load(open(self._data_path + "behaviors_info.yaml"))

		# Load info from xml files
		for value in root.values():
			behaviors = value

		return behaviors

	def load_homeostatic_variables(self):
		
		homeostatic_variables = None
		root = yaml.load(open(self._data_path + "homeostatic_variables.yaml"))

		for value in root.values():
			homeostatic_variables = value 

		return homeostatic_variables

	def load_motivations(self):

		motivations = None

		root = yaml.load(open(self._data_path + "motivations.yaml"))

		for value in root.values():
			motivations = value

		return motivations

	def load_ext_stimulus(self):

		ext_stimuli = None

		root = yaml.load(open(self._data_path + "external_stimuli.yaml"))

		for value in root.values():
			ext_stimuli = value

		return ext_stimuli

	def load_exogenous_actions(self):

		exogenous = None

		root = yaml.load(open(self._data_path + "exogenous_actions.yaml"))

		for value in root.values():
			exogenous = value

		return exogenous


	def load_profiles(self):

		profile = None

		root = yaml.load(open(self._data_path + "user_profiles.yaml"))

		for value in root.values():
			profile = value

		return profile

	def load_robot_profiles(self):

		profile = None

		root = yaml.load(open(self._data_path + "robot_profiles.yaml"))

		for value in root.values():
			profile = value

		return profile

	def find_profile(self, id, profiles):

		profile = dict()

		for item in profiles:
			if id == item["id"]:
				profile = item
				return profile

		return profile

