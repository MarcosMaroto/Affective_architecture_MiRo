#!/usr/bin/python
# -*- coding: utf-8 -*-

__author__ = "Marcos Maroto"
__copyright__ = "Social Robots Group. R.L. University Carlos III of Madrid"
__credits__ = ["Marcos Maroto"]
__license__ = "LEUC3M v1.0"
__version__ = "0.0.0"
__maintainer__ = "Marcos Maroto"
__email__ = "marmarot@ing.uc3m.es"
__status__ = "Development"

pkg_name = "motivational_dms"
node_name = "emotion_manager"

import rospy
import random

from datetime import datetime as dt
from std_msgs.msg import Float32, String, Empty
from miro_msgs.msg import platform_control
from classes.load_data import DataImport
from classes.logger import Logger
from motivational_dms.msg import ExternalStimulus, BehaviorResult,ExogenousAction

# Simulation flag
SIMULATION = rospy.get_param("decision_making/simulate")

# Real time step value used when simulation is not running
REAL_TIME_STEP = 1

# Time step used in case of simulation
if SIMULATION:
	TIME_STEP = 0.05
	CORRECTION_TIME = TIME_STEP/REAL_TIME_STEP
else:
	TIME_STEP = REAL_TIME_STEP
	CORRECTION_TIME = 1

dominant_threshold = 20

FILENAME = dt.now().strftime("%Y%m%d-%H%M") + "/motivation"

class MotivationManager():
	""" --- Motivation CLASS---
	    --- Motivational state and drive evolution of Miro's internal state ---
	"""

	def __init__(self):
		"""
		Init of the node
		"""

		# Class variables
		self.__active_behavior = {"name": None, "status": None, "result": None, "effects": [], "related_homeostatic_variables": None}
		self.__satisfaction_time = dict()
		self.__active_exogenous_actions = []
		self.__stop_signal = False

		# Motivations variables
		self.__dominant_motivation = {"motivation_name": None}
		self.__last_dominant_motivation = {"motivation_name": None}

		# Load data from files
		self.__data = DataImport()
		self.__homeostatic_variables = self.__data.load_homeostatic_variables()
		self.__motivations = self.__data.load_motivations()
		self.__external_stimulus = self.__data.load_ext_stimulus()
		self.__behaviors = self.__data.load_behavior_info()
		self.__exogenous_actions = self.__data.load_exogenous_actions()

		# logging
		self.__mot_log = {}
		self.__homeostatic_log = {}
		self.__exogenous_actions_log = {}
		for mot in self.__motivations:
			self.__mot_log[mot["motivation_name"]] = Logger(mot["motivation_name"], FILENAME, 'motivation')
		for var in self.__homeostatic_variables:
			self.__homeostatic_log[var["variable_name"]] = Logger(var["variable_name"], FILENAME, 'homeostatic_variable')
		self.__dom_mot_log = Logger('dominant', FILENAME, 'motivation')
		self.__external_stimulus_log = Logger("external_stimulus_values", FILENAME, 'motivation')
		self.__external_stimulus_log.write_labels("Balls;User_presence;Lights;User_near;User_far;")
		self.__exogenous_actions_log = Logger("exogenous_actions", FILENAME, 'motivation')
		self.__exogenous_actions_log.write_labels("Action;Agent/Object;Value")

		# Create pubs and subs
		self.create_msg_srv()

		rospy.sleep(1)

		# Creating homeostatic timer
		self.__homeostatic_timer = rospy.Timer(rospy.Duration(TIME_STEP), self.__homeostatic_timeout)

	def create_msg_srv(self):
		"""
		Creation of publishers and subscribers
		"""

		# Publishers
		self.__dom_mot_pub = rospy.Publisher("motivational_dms/dominant_motivation", String, queue_size=1, latch=True)
		self.__wellbeing_pub = rospy.Publisher("motivational_dms/wellbeing", Float32, queue_size=1, latch=True)

		# Subscribers
		self.__external_stimulus_sub = rospy.Subscriber("perception_manager/external_stimulus", ExternalStimulus, self.__ext_stimulus_cb)
		self.__behavior_result_sub = rospy.Subscriber("motivational_dms/behavior_result", BehaviorResult, self.__result_cb)
		self.__exogenous_sub = rospy.Subscriber("perception_manager/exogenous_action", ExogenousAction, self.__exogenous_cb)
		self.__stop_signal_sub = rospy.Subscriber("motivational_dms/stop_simulation", Empty, self.__stop_cb)	

	def execute(self):
		
		self.__dominant_motivation = self.get_dominant()

		if self.__dominant_motivation["motivation_name"] != self.__last_dominant_motivation["motivation_name"]:
			self.__dom_mot_pub.publish(self.__dominant_motivation["motivation_name"])
			self.__last_dominant_motivation = self.__dominant_motivation.copy()

		self.save_log()
		self.calculate_wellbeing()

		if not SIMULATION:
			self.print_debug()

	def run(self):

		while not rospy.is_shutdown():

			if self.__stop_signal:
				break

			self.execute()

			rospy.sleep(TIME_STEP)

	def stop(self):

		print "STOPPING THE NODE"

######################################################
################### CLASS FUNCTIONS ##################
######################################################

	def get_dominant(self):

		dominant_motivation = None

		# Apply effect of drives over motivation
		for motivation in self.__motivations:
			for variable in self.__homeostatic_variables:
				if variable["variable_name"] in motivation["related_homeostatic_variables"]:
					motivation["value"] = abs(variable["value"] - variable["ideal_value"])

		# Apply effect of external stimulus over motivations
		for motivation in self.__motivations:
			average_value = 0
			n_estimulus = 0
			drive_value = motivation["value"]
			for external_stimulus in self.__external_stimulus:
				if external_stimulus["active"]:
					if motivation["motivation_name"] in external_stimulus["affected_motivations"]:
						motivation["value"] += external_stimulus["alpha"] * (motivation["value"]/100) * external_stimulus["value"]

		for motivation in self.__motivations:
			if motivation["motivation_name"] != "no_motivation":
				if motivation["motivation_name"] == self.__dominant_motivation["motivation_name"]:
					motivation["value"] += dominant_threshold
				elif self.__last_dominant_motivation["motivation_name"] == motivation["motivation_name"]:
					motivation["value"] -= dominant_threshold

		# Find dominant motivation
		max_value = 0
		for motivation in self.__motivations:
			if motivation["value"] >= motivation["activation_value"]:
				if motivation["value"] > max_value:
					dominant_motivation = motivation.copy()
					max_value = dominant_motivation["value"]

		return dominant_motivation

	def find_behavior(self, name):
		
		effects = dict()

		for behavior in self.__behaviors:
			if name == behavior["behavior"]:
				effects = behavior["effects"]
				break

		return effects

	def apply_effect(self, active_behavior):
		
		for var in self.__homeostatic_variables:
			for effect in var["effects"]:
				if var["variable_name"] == effect["homeostatic_variable"]:
					var["value"] += effect["value"]
					var["value"] = max(var["value"], var["min_value"])
					if var["value"] == var["ideal_value"]:
						var["satisfied"] = True
						self.__satisfaction_time[var["variable_name"]] = rospy.Time.now()

	def calculate_wellbeing(self):
		
		ideal_value = 0
		deficit_value = 0
		signal_range = [0, 100]

		for variable in self.__homeostatic_variables:
			ideal_value += variable["ideal_value"]
			deficit_value += abs(variable["ideal_value"] - variable["value"])

		wellbeing = ideal_value - deficit_value

		# Adapt signal to signal_range
		wellbeing = wellbeing * signal_range[1] / ideal_value

		self.__wellbeing_pub.publish(wellbeing)

	def print_debug(self):
		
		print "···········MOTIVATIONS············"
		for mot in self.__motivations:
			print("Motivation: %s. Value: %.3f" % (mot["motivation_name"], mot["value"]))
		print "\n"

		print "···········HOMEST. VARIABLES············"
		for var in self.__homeostatic_variables:
			print("Variable: %s. Value: %.3f" % (var["variable_name"], var["value"]))
		print "\n"

		print "···········EXT. STIMULUS············"
		for stimulus in self.__external_stimulus:
			print("Stimulus: %s. Value: %.3f. Status: %s" % (stimulus["stimulus"], stimulus["value"], str(stimulus["active"])))
		print "\n"

	def save_log(self):

		# logging
		for mot in self.__motivations:
			self.__mot_log[mot["motivation_name"]].write_data(mot["value"])
		for var in self.__homeostatic_variables:
			self.__homeostatic_log[var["variable_name"]].write_data(var["value"])
		self.__dom_mot_log.write_data(self.__dominant_motivation["motivation_name"])

######################################################
############# SUBSCRIBERS FUNCTIONS ##################
######################################################

	def __homeostatic_timeout(self, event):
		
		for variable in self.__homeostatic_variables:
			if variable["satisfied"]:
				if rospy.Time.now() > (self.__satisfaction_time[variable["variable_name"]] + rospy.Duration(variable["satisfaction_time"]*CORRECTION_TIME)):
					del self.__satisfaction_time[variable["variable_name"]]
					variable["satisfied"] = False

			if not variable["satisfied"]:
				init_value = variable["value"]
				for effect in self.__active_behavior["effects"]:
					if effect["homeostatic_variable"] == variable["variable_name"] and effect["type"] == "continuous":
						variable["value"] += effect["value"]
				
				if init_value == variable["value"]:
					variable["value"] += variable["variation_value"]		
				variable["value"] = min(max(variable["value"], variable["min_value"]), variable["max_value"])


			if variable["value"] == variable["ideal_value"] and not variable["satisfied"]:
				variable["satisfied"] = True
				self.__satisfaction_time[variable["variable_name"]] = rospy.Time.now()

	def __ext_stimulus_cb(self, msg):

		for stimulus in self.__external_stimulus:
			if stimulus["stimulus"] == msg.stimulus:
				stimulus["active"] = msg.status
				stimulus["value"] = msg.value
				break

		aux_str = ""
		for stimulus in self.__external_stimulus:
			aux_str += str(stimulus["value"]) + ";"
		self.__external_stimulus_log.write_data(aux_str)

	def __result_cb(self, msg):

		if msg.behavior_name:
			self.__active_behavior["name"] = msg.behavior_name
			self.__active_behavior["result"] = msg.result
			self.__active_behavior["status"] = msg.status

			self.__active_behavior["effects"] = self.find_behavior(msg.behavior_name)
			if self.__active_behavior["status"] == "finished":
				for effect in self.__active_behavior["effects"]:
					if effect["type"] == "punctual":
						self.apply_effect(self.__active_behavior)

	def __exogenous_cb(self, msg):

		for action in self.__exogenous_actions:
			if action["action"] == msg.action_name:
				if action["effect"]["type"] == "punctual":
					for variable in self.__homeostatic_variables:
						if variable["variable_name"] in action["related_homeostatic_variables"]:
							if action["effect"]["value"] == "variable":
								variable["value"] += msg.value
							else:
								variable["value"] += action["effect"]["value"]
				elif action["effect"]["type"] == "continuous":
					for idx, act in enumerate(self.__active_exogenous_actions):
						if act["action"] == msg.action_name:
							if not msg.status:
								del self.__active_exogenous_actions[idx]
								break
					self.__active_exogenous_actions.append(action)
		aux_str = str(msg.action_name) + ";" + str(msg.agent_object) + ";" + str(msg.value)
		self.__exogenous_actions_log.write_data(aux_str)

	def __stop_cb(self, msg):
		
		self.__stop_signal = True
		
if __name__ == '__main__':
    try:
        # start the node
        rospy.init_node(node_name)

        # create and spin the node
        node = MotivationManager()

        rospy.on_shutdown(node.stop)

        # spin the node
        node.run()
    except rospy.ROSInterruptException:
        pass