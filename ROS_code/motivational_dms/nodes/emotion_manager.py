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
from motivational_dms.msg import Emotion, BehaviorResult, ExternalStimulus

# Robot emotional profile
ROBOT_PROFILE = "neutral"

# Simulation flag
SIMULATION = rospy.get_param("decision_making/simulate")

# Adaptation flag
ADAPTATION = rospy.get_param(node_name+"/adaptation")
ADAPTATION_FACTOR = 0.005

# Real time step value used when simulation is not running
REAL_TIME_STEP = 0.2

# Time step used in case of simulation
if SIMULATION:
	TIME_STEP = 0.01
	CORRECTION_TIME = TIME_STEP/REAL_TIME_STEP
else:
	TIME_STEP = REAL_TIME_STEP
	CORRECTION_TIME = 1

FILENAME = dt.now().strftime("%Y%m%d-%H%M") + "/emotions"

# Range of increasment
inc_max = 10
inc_min = 0

class EmotionManager():
	""" --- EMOTION CLASS---
	    --- MIRO touch sensor raw data interpreter ---
	"""

	def __init__(self):
		"""
		Init of the node
		"""

		# Class variables 
		self.__signal_values = {"wellbeing": 100, "user_presence": 0, "user_answers": 50, "touch_stimulus": 0}
		self.__signal_params = {"wellbeing": 1, "user_presence": 0.33, "user_answers": 0.33, "touch_stimulus": 0.33}
		self.__external_signals_params = {"user_presence": 0, "user_answers": 50, "touch_stimulus": 0}
		self.__previous_answer = 0
		self.__answers_values = {BehaviorResult().NO_ANSWER: -30, BehaviorResult().CORRECT: 20, BehaviorResult().INCORRECT: -10}
		self.__forget_emotion_timer = None
		self.__stop_signal = False

		# Emotion-related variables
		self.__emotions = list()
		self.__dominant_emotion = dict()
		self.__last_dominant_emotion = {"emotion": None}
		self.__emotion_signal = 0
		self.__prev_emotion_signal = 0
		self.__external_emotion_signal = 0
		self.__prev_external_emotion_signal = 0
		self.__decrese_answer_signal = False
		self.__forget_time = 120
		self.__forget_emotion_timer = rospy.Timer(rospy.Duration(self.__forget_time*CORRECTION_TIME), self.__forget_emotion_cb, oneshot=True)

		# Load emotions
		self.__data = DataImport()
		self.__emotions = self.__data.load_emotions()
		self.__signal_types = self.__data.load_signal_params()
		self.__robot_profiles = self.__data.load_robot_profiles()
		self.__robot_profile = self.__data.find_profile(ROBOT_PROFILE, self.__robot_profiles)

		# Assigning weights to params based on profile
		self.__internal_alpha = self.__robot_profile["internal"]
		self.__external_alpha = 1 - self.__internal_alpha
		for key in self.__signal_params.keys():
			self.__signal_params[key] = self.__robot_profile[key]
			
		# Logging
		self.__emotions_log = {}
		self.__signal_params_log = {}
		for emotion in self.__emotions:
			self.__emotions_log[emotion["emotion"]] = Logger(emotion["emotion"], FILENAME, 'emotion')
		for key in self.__signal_params.keys():
			self.__signal_params_log[key] = Logger(key+"_param", FILENAME, 'emotion')
		self.__signal_params_log["internal"] = Logger("internal_param", FILENAME, 'emotion')
		self.__signal_params_log["external"] = Logger("external_param", FILENAME, 'emotion')
		self.__dom_emotion_log = Logger('dominant_emotion', FILENAME, 'emotion')
		self.__emotion_signal_log = Logger('emotion_signal', FILENAME, 'emotion')
		self.__external_emotion_signal_log = Logger('external_emotion_signal', FILENAME, 'emotion')
		self.__signal_values_log = Logger('signal_values', FILENAME, 'emotion')
		self.__signal_values_log.write_labels("Answers;Wellbeing;Touch Stimulus;User presence")
		
		# Logging profile used in simulation
		self.__profile_log = Logger("profile", dt.now().strftime("%Y%m%d-%H%M") + "/profile", 'profile')
		self.__profile_log.write_labels("PROFILE OF THE ROBOT \n")
		for key, value in self.__robot_profile.items():
			self.__profile_log.write_data(key.title()+": "+str(value), timestamp=False)

		# Define variables of the node
		self.__create_msg_srv()

		# Init of both signals
		for sig in self.__signal_types:
			if sig["type"] == "internal":
				self.__emotion_signal += self.__internal_alpha * self.__signal_params[sig["signal"]] * self.__signal_values[sig["signal"]]
			elif sig["type"] == "external":
				self.__emotion_signal += self.__external_alpha * self.__signal_params[sig["signal"]] * self.__signal_values[sig["signal"]]
				self.__external_emotion_signal += self.__signal_params[sig["signal"]] * self.__signal_values[sig["signal"]]

		self.__prev_emotion_signal = self.__emotion_signal
		self.__prev_external_emotion_signal = self.__external_emotion_signal

		rospy.sleep(1)

	def __create_msg_srv(self):
		"""
		Creation of publishers and subscribers
		"""

		# Publishers
		self.__emotion_pub = rospy.Publisher("motivational_dms/dominant_emotion", String, queue_size=1, latch=True)

		# Subscribers
		self.__sti_sig_sub = rospy.Subscriber("motivational_dms/stimulation_signal", Float32, self.__stimulation_cb)
		self.__user_answers_sub = rospy.Subscriber("motivational_dms/behavior_result", BehaviorResult, self.__result_cb)
		self.__external_stimulus_sub = rospy.Subscriber("perception_manager/external_stimulus", ExternalStimulus, self.__ext_stimulus_cb)
		self.__wellbeing_sub = rospy.Subscriber("motivational_dms/wellbeing", Float32, self.__wellbeing_cb)
		self.__stop_signal_sub = rospy.Subscriber("motivational_dms/stop_simulation", Empty, self.__stop_cb)	

	def evaluate_emotional_state(self):

		weighted_signal_values = self.__signal_values

		self.__emotion_signal = 0
		self.__external_emotion_signal = 0

		for sig in self.__signal_types:
			if sig["type"] == "internal":
				self.__emotion_signal += self.__internal_alpha * self.__signal_params[sig["signal"]] * weighted_signal_values[sig["signal"]]
			elif sig["type"] == "external":
				self.__emotion_signal += self.__external_alpha * self.__signal_params[sig["signal"]] * weighted_signal_values[sig["signal"]]
				self.__external_emotion_signal += self.__signal_params[sig["signal"]] * weighted_signal_values[sig["signal"]]

		variation = self.__emotion_signal - self.__prev_emotion_signal
		external_variation = self.__external_emotion_signal - self.__prev_external_emotion_signal

		for emotion in self.__emotions:
			if emotion["emotion"] in ["joy", "calmness", "sadness"]:
				if emotion["range"][0] <= self.__emotion_signal <= emotion["range"][1]: 
					if emotion["variation"] == 0:
						if emotion["ideal_value"] is not None:
							if emotion["range"][0] == emotion["ideal_value"]:
								other_range = emotion["range"][1]
								old_value = self.__emotion_signal
							elif emotion["range"][1] == emotion["ideal_value"]:
								other_range = emotion["range"][0]
								old_value = self.__emotion_signal
							else:
								other_range = emotion["range"][0]
								old_value = emotion["ideal_value"] - abs(emotion["ideal_value"] - self.__emotion_signal)

							old_range = (emotion["ideal_value"] - other_range)  
							new_range = (inc_max - inc_min)  
							value = (((old_value - other_range) * new_range) / old_range) + inc_min

							emotion["intensity"] += value
						else:
							emotion["intensity"] += 0.1
					else:
						emotion["intensity"] += emotion["decrement_factor"]

				else:
					emotion["intensity"] += emotion["decrement_factor"]
			else:
				if emotion["range"][0] <= self.__external_emotion_signal <= emotion["range"][1]:
					if external_variation > 0 and emotion["variation"] > 0:
						if external_variation > emotion["variation"]:
							emotion["intensity"] = emotion["max_intensity"]
						else:
							emotion["intensity"] += emotion["decrement_factor"]
					elif external_variation < 0 and emotion["variation"] < 0:
						if external_variation < emotion["variation"]:
							emotion["intensity"] = emotion["max_intensity"]
						else:
							emotion["intensity"] += emotion["decrement_factor"] 
					else:
						emotion["intensity"] += emotion["decrement_factor"]
				else:
					emotion["intensity"] += emotion["decrement_factor"]			


			emotion["intensity"] = max(min(emotion["intensity"], emotion["max_intensity"]), emotion["min_intensity"])

		max_intensity = 0
		for emotion in self.__emotions:
			if emotion["intensity"] >= emotion["threshold"]:
				if (self.__dominant_emotion is None or emotion["intensity"] > max_intensity):
					self.__dominant_emotion = emotion.copy()
					max_intensity = emotion["intensity"]

		if self.__dominant_emotion["emotion"] != self.__last_dominant_emotion["emotion"]:
			self.__last_dominant_emotion = self.__dominant_emotion.copy()
			self.__emotion_pub.publish(self.__dominant_emotion["emotion"])

		self.__prev_emotion_signal = self.__emotion_signal
		self.__prev_external_emotion_signal = self.__external_emotion_signal

		if ADAPTATION:
			self.adapt_signal_values()

		if self.__decrese_answer_signal:
			self.__signal_values["user_answers"] -= 0.05
			if self.__signal_values["user_answers"] <= 0:
				self.__signal_values["user_answers"] = 0
				self.__decrese_answer_signal = False

	def print_debug(self):
		
		rospy.loginfo("Value of signals %s", str(self.__signal_values))
		rospy.loginfo("Value of Emotional signal is: %.3f. Value for external emotional signal is: %.3f" ,self.__emotion_signal, self.__external_emotion_signal)
		for emotion in self.__emotions:
			rospy.loginfo("Emotion: %s, Value: %0.3f." ,emotion["emotion"], emotion["intensity"])

		print("\n\n\n")

	def save_log(self):
		# logging
		for emotion in self.__emotions:
			self.__emotions_log[emotion["emotion"]].write_data(emotion["intensity"])
		self.__dom_emotion_log.write_data(self.__dominant_emotion["emotion"])
		self.__emotion_signal_log.write_data(self.__emotion_signal)
		self.__external_emotion_signal_log.write_data(self.__external_emotion_signal)
		aux = ""
		for key, value in self.__signal_values.items():
			aux += str(value) + ";"
		self.__signal_values_log.write_data(aux)

		for key, value in self.__signal_params.items():
			self.__signal_params_log[key].write_data(self.__signal_params[key])
		self.__signal_params_log["internal"].write_data(self.__internal_alpha)
		self.__signal_params_log["external"].write_data(self.__external_alpha)
		
	def adapt_signal_values(self):
		
		# Adaptation of internal and external weights
		variation = (self.__external_emotion_signal-self.__signal_values["wellbeing"])*ADAPTATION_FACTOR
		self.__external_alpha += variation
		self.__external_alpha = min(max(self.__external_alpha ,0), 1)
		self.__internal_alpha = 1.0 - self.__external_alpha

		# Adaptation of external weights
		max_value = 0
		max_key = ""
		for key, value in self.__external_signals_params.items():
			self.__external_signals_params[key] = self.__signal_values[key]
			if self.__external_signals_params[key] > max_value:
				max_value = self.__external_signals_params[key]
				max_key = key


		diffs = dict()
		for key, value in self.__external_signals_params.items():
			diff = (max_value - value)*ADAPTATION_FACTOR 
			if diff > 0 and self.__signal_params[key] > 0.0:
				diffs[key] = round(diff,5)
			elif diff > 0 and self.__signal_params[key] == 0:
				diffs[key] = 0

		for key, value in self.__external_signals_params.items():
			if len(diffs.values())>1:
				if key in diffs.keys():
					self.__signal_params[key] -= diffs[key]
				elif key == max_key:
					self.__signal_params[key] += sum(diffs.values())
			else:
				if key in diffs.keys():
					self.__signal_params[key] -= diffs[key]
				else:
					self.__signal_params[key] += sum(diffs.values())/2

			self.__signal_params[key] = min(max(self.__signal_params[key] ,0), 1)

		for key, value in self.__signal_params.items():
			if key in self.__external_signals_params.keys():
				self.__signal_params[key] = float(value)/(sum(self.__signal_params.values())-1)

	def run(self):

		while not rospy.is_shutdown():

			if self.__stop_signal:
				break

			self.evaluate_emotional_state()
			self.save_log()

			if not SIMULATION:
				self.print_debug()

			rospy.sleep(TIME_STEP)

	def stop(self):

		print "STOPPING THE NODE"

######################################################
############# SUBSCRIBERS FUNCTIONS ##################
######################################################

	def __stimulation_cb(self, msg):

		self.__signal_values["touch_stimulus"] = round(msg.data,3)

	def __result_cb(self, msg):

		if msg.behavior_name == "play_game":

			if msg.answer in [BehaviorResult().NO_ANSWER, BehaviorResult().INCORRECT]:
				if self.__previous_answer in [BehaviorResult().NO_ANSWER, BehaviorResult().INCORRECT]:
					self.__signal_values["user_answers"] += self.__answers_values[self.__previous_answer] + self.__answers_values[msg.answer]
				else:
					self.__signal_values["user_answers"] += self.__answers_values[msg.answer]
			elif msg.answer == BehaviorResult().CORRECT:
				if self.__previous_answer == BehaviorResult().CORRECT:
					self.__signal_values["user_answers"] += self.__answers_values[self.__previous_answer] + self.__answers_values[msg.answer]
				else:
					self.__signal_values["user_answers"] += self.__answers_values[msg.answer]

			if not SIMULATION:
				rospy.loginfo("RECEIVED ANSWER FROM GAME: %s, PREVIOUS WAS: %s", str(msg.answer), str(self.__previous_answer))

			self.__signal_values["user_answers"] = min(max(self.__signal_values["user_answers"], 0), 100)
			self.__previous_answer = msg.answer
			self.__forget_emotion_timer = rospy.Timer(rospy.Duration(60), self.__forget_emotion_cb, oneshot=True)
			self.__decrese_answer_signal = False

	def __ext_stimulus_cb(self, msg):
		
		if msg.stimulus == "user_presence":
			self.__signal_values[msg.stimulus] = round(msg.value,3)

	def __wellbeing_cb(self, msg):
		
		self.__signal_values["wellbeing"] = round(msg.data,3)

	def __forget_emotion_cb(self, event):

		self.__previous_answer = None
		self.__decrese_answer_signal = True

	def __stop_cb(self, msg):
		
		self.__stop_signal = True

if __name__ == '__main__':
    try:
        # start the node
        rospy.init_node(node_name)

        # create and spin the node
        node = EmotionManager()

        rospy.on_shutdown(node.stop)

        # spin the node
        node.run()
    except rospy.ROSInterruptException:
        pass