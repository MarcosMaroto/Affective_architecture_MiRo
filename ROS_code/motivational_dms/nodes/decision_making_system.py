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
node_name = "dms"

import rospy
import random
import numpy

from datetime import datetime as dt
from std_msgs.msg import String, Empty
from motivational_dms.msg import BehaviorResult, BehaviorInfo, ExternalStimulus, ExogenousAction
from classes.load_data import DataImport
from classes.logger import Logger

# File for storing the log files
FILENAME = dt.now().strftime("%Y%m%d-%H%M") + "/decision_making"

# Simulation flag
SIMULATION = rospy.get_param("decision_making/simulate")

# Real time step value used when simulation is not running
REAL_TIME_STEP = 0.2

# Time step used in case of simulation
if SIMULATION:
	TIME_STEP = 0.01
	CORRECTION_TIME = TIME_STEP/REAL_TIME_STEP
else:
	TIME_STEP = REAL_TIME_STEP
	CORRECTION_TIME = 1

class DecisionMaking():
	""" --- BEHAVIOR MANAGER CLASS---
	    --- MIRO touch sensor raw data interpreter ---
	"""

	def __init__(self):
		"""
		Init of the node
		"""

		# Class variables
		self.__dominant_motivation = None
		self.__dominant_motivation_changed = False
		self.__behavior_finished = True
		self.__current_behavior = None
		self.__last_behavior = None
		self.__external_stimuli = {"user_presence": False, "user_near": False, "user_far": False, "balls": False, "lights": False}
		self.__behaviors_value = dict()
		self.__external_stimuli_changed = False
		self.__cancel_behavior = False
		self.__wandering_timer = None
		self.__waiting_result = False
		self.__dominant_emotion = None
		self.__dominant_emotion_changed = False
		self.__behavior_id = ""
		self.__stop_signal = False

		# Load behaviors
		self.__data = DataImport()
		self.__behaviors = self.__data.load_behavior_info()
		self.__emotions = self.__data.load_emotions()

		# Logging decisions
		self.__behavior_log = Logger("active_behavior", FILENAME, 'behavior')
		self.__external_stimulus_log = Logger("external_stimulus", FILENAME, 'behavior')
		self.__external_stimulus_log.write_labels("Lights;User near;Balls;User_present;User_far")

		# Define variables of the node
		self.create_msg_srv()

		rospy.sleep(1)

	def create_msg_srv(self):
		"""
		Creation of publishers and subscribers
		"""

		# Publishers
		self.__behavior_info_pub = rospy.Publisher("motivational_dms/behavior_information", BehaviorInfo, queue_size=5, latch=True)
		self.__cancel_pub = rospy.Publisher("motivational_dms/cancel_behavior", Empty, queue_size=5, latch=True)

		# Subscribers
		self.__dominant_motivation_sub = rospy.Subscriber("motivational_dms/dominant_motivation", String, self.__dominant_motivation_cb)
		self.__behavior_result_sub = rospy.Subscriber("motivational_dms/behavior_result", BehaviorResult, self.__result_cb)
		self.__external_stimuli_sub = rospy.Subscriber("perception_manager/external_stimulus_status", ExternalStimulus, self.__external_stimuli_cb)
		self.__emotion_sub = rospy.Subscriber("motivational_dms/dominant_emotion", String, self.__emotion_cb)
		self.__stop_signal_sub = rospy.Subscriber("motivational_dms/stop_simulation", Empty, self.__stop_cb)	

	def decide_behavior(self):

		if (self.__current_behavior is None or self.__dominant_motivation_changed or self.__dominant_emotion_changed or self.__external_stimuli_changed or self.__cancel_behavior) and not self.__waiting_result:
			
			possible_behaviors = list()
			self.__current_behavior = None
			for behavior in self.__behaviors:
				find = False
				if self.__dominant_motivation in behavior["related_motivations"]:
					if behavior["type"] == "consumatory":
						if behavior["requisites"] is not None:
							for requisite, value in behavior["requisites"].items():
								if self.__external_stimuli[requisite] != value:
									find = True
									break
							if not find:
								possible_behaviors.append(behavior)
						else:
							possible_behaviors.append(behavior)

			if bool(possible_behaviors):
				max_requisites = 0
				for behavior in possible_behaviors:
					if behavior["requisites"] is not None:
						if len(behavior["requisites"].keys()) > max_requisites:
							max_requisites = len(behavior["requisites"].keys())
							self.__current_behavior = behavior
				if not self.__current_behavior:
					self.__current_behavior = random.choice(possible_behaviors)
			else:
				possible_behaviors = list()
				for behavior in self.__behaviors:
					find = False
					if self.__dominant_motivation in behavior["related_motivations"]:
						if behavior["requisites"] is not None:
							for requisite, value in behavior["requisites"].items():
								if self.__external_stimuli[requisite] != value:
									find = True
									break
							if not find:
								possible_behaviors.append(behavior)
						else:
							if behavior["type"] == "appetitive":
								possible_behaviors.append(behavior)
							elif behavior["type"] == "emotional":
								if str(self.__dominant_emotion) in behavior["behavior"]:
									possible_behaviors.append(behavior)
			
				if bool(possible_behaviors):
					if self.__dominant_motivation == "no_motivation":
						n_behaviors = len(possible_behaviors)
						if n_behaviors > 1:
							probabilities = list()
							for idx, behavior in enumerate(possible_behaviors):
								probabilities.append(0)
								if behavior.has_key("probability"):
									probabilities[idx] = behavior["probability"]
									prob = behavior["probability"]
									if behavior["behavior"] == "express_surprise":
										behavior["probability"] = 0.1
								else:
									prob = 1
							for idx, probability in enumerate(probabilities):
								if probabilities[idx] == 0:
									probabilities[idx] = (1-prob)/(n_behaviors-1)
									
							self.__current_behavior = numpy.random.choice(possible_behaviors, p=probabilities)
						else:
							self.__current_behavior = random.choice(possible_behaviors)
					else:
						if not self.__current_behavior:
							self.__current_behavior = random.choice(possible_behaviors)
					
				else:
					possible_behaviors = list()
					for behavior in self.__behaviors:
						if behavior["behavior"] == "wander":
							self.__current_behavior = behavior

			if self.__current_behavior != self.__last_behavior and not self.__behavior_finished and self.__last_behavior["cancel"]:
				self.__cancel_pub.publish()
				self.__cancel_behavior = True
				self.__behavior_finished = False
				self.__waiting_result = True
				if not SIMULATION:
					rospy.loginfo("Cancelling current active behavior")
			elif self.__current_behavior == self.__last_behavior: 
				if self.__current_behavior["behavior"] in ["wander", "play_alone"]:
					self.__wandering_timer = rospy.Timer(rospy.Duration(8*CORRECTION_TIME), self.__wandering_timer_cb, oneshot=True)
					self.__cancel_behavior = False
			
			self.__dominant_motivation_changed = False
			self.__external_stimuli_changed = False
			self.__dominant_emotion_changed = False


		if self.__behavior_finished and self.__current_behavior is not None:

			self.__waiting_result = False	

			value = 0
			msg = BehaviorInfo(id=self.__current_behavior["behavior"], behavior_id=self.__behavior_id,  motivation=self.__dominant_motivation, behaviour_type=self.__current_behavior["type"], value=value, emotion=self.__dominant_emotion)
			self.__behavior_info_pub.publish(msg)
			aux_str = str(self.__dominant_motivation) + ";" + str(self.__dominant_emotion) + ";" + str(self.__current_behavior["behavior"])
			self.__behavior_log.write_data(aux_str)
			self.__last_behavior = self.__current_behavior
			if not SIMULATION:
				rospy.loginfo("Selected behavior is %s", self.__current_behavior["behavior"])

			if self.__current_behavior["behavior"] in ["wander", "play_alone"]:
				self.__wandering_timer = rospy.Timer(rospy.Duration(8*CORRECTION_TIME), self.__wandering_timer_cb, oneshot=True)
			else:
				if self.__wandering_timer is not None:
					self.__wandering_timer.shutdown()
					self.__wandering_timer = None
			
			self.__behavior_finished = False
			self.__dominant_motivation_changed = False
			self.__external_stimuli_changed = False
			self.__cancel_behavior = False
			self.__dominant_emotion_changed = False
			self.__behavior_id = ""
 
	def run(self):

		while not rospy.is_shutdown():

			if self.__stop_signal:
				break

			self.decide_behavior()

			rospy.sleep(TIME_STEP)

	def stop(self):

		pass


######################################################
############# SUBSCRIBERS FUNCTIONS ##################
######################################################

	def __wandering_timer_cb(self, event):

		self.__cancel_behavior = True

	def __dominant_motivation_cb(self, msg):
		
		self.__dominant_motivation = msg.data
		if msg.data != "no_motivation":
			self.__dominant_motivation_changed = True
		if not SIMULATION:
			rospy.loginfo("Dominant motivation changed to %s", msg.data)

	def __result_cb(self, msg):

		if msg.status == "finished" and msg.result == BehaviorResult().SUCCESS:
			self.__behavior_finished = True
			if not SIMULATION:
				rospy.loginfo("Result obtained for behavior")
			if not self.__cancel_behavior:
				self.__current_behavior = None

			if msg.behavior_name == "play_game":
				self.__current_behavior = dict()
				if msg.answer == BehaviorResult().INCORRECT:
					self.__current_behavior["behavior"] = "express_sadness"
					self.__current_behavior["type"] = "emotional"
					self.__current_behavior["cancel"] = False
					self.__behavior_id = "incorrect_answer"
				elif msg.answer == BehaviorResult().CORRECT:
					self.__current_behavior["behavior"] = "express_joy"
					self.__current_behavior["type"] = "emotional"
					self.__current_behavior["cancel"] = False
					self.__behavior_id = "correct_answer"
				elif msg.answer == BehaviorResult().NO_ANSWER:
					self.__current_behavior["behavior"] = "express_sadness"
					self.__current_behavior["type"] = "emotional"
					self.__current_behavior["cancel"] = False
					self.__behavior_id = ""

				self.__waiting_result = True

			if self.__wandering_timer is not None:
				self.__wandering_timer.shutdown()	
				self.__wandering_timer = None			

	def __external_stimuli_cb(self, msg):

		if not SIMULATION:
			rospy.loginfo("External_stimulus changed %s is now %s", msg.stimulus, str(msg.status))
		if self.__external_stimuli.has_key(msg.stimulus):
			self.__external_stimuli[msg.stimulus] = msg.status
			self.__external_stimuli_changed = True
			aux_str = ""
			for key, value in self.__external_stimuli.items():
				aux_str += str(value) + ";"
			self.__external_stimulus_log.write_data(aux_str)

	def __emotion_cb(self, msg):
		
		self.__dominant_emotion = msg.data
		self.__dominant_emotion_changed = True
		if self.__dominant_emotion == "surprise":
			for behavior in self.__behaviors:
				if behavior["behavior"] == "express_surprise":
					behavior["probability"] = 1
		if not SIMULATION:
			rospy.loginfo("Dominant emotion changed to %s", str(msg.data))

	def __stop_cb(self, msg):
		
		self.__stop_signal = True

if __name__ == '__main__':
    try:
        # start the node
        rospy.init_node(node_name)

        # create and spin the node
        node = DecisionMaking()

        rospy.on_shutdown(node.stop)

        # spin the node
        node.run()
    except rospy.ROSInterruptException:
        pass