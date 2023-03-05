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
node_name = "simulation_manager"

import rospy
import random
import numpy

from datetime import datetime as dt
from std_msgs.msg import String, Empty
from classes.logger import Logger
from motivational_dms.msg import BehaviorInfo, BehaviorResult, ExternalStimulus, ExogenousAction
from miro_perception_manager.msg import MiroTouch
from classes.load_data import DataImport
from classes.logger import Logger

# PROFILE TO LOAD
PROFILE = "disengaged_touch"

# File for storing the log files
FILENAME = dt.now().strftime("%Y%m%d-%H%M") + "/profile"

# Every simulation time step corresponds to 0.2 seconds in reality
REAL_TIME_STEP = 0.2
TIME_STEP = 0.01
TIME_CORRECTION = TIME_STEP/REAL_TIME_STEP

# NUMBER OF STEPS RUN IN SIMULATION
SIMULATION_STEPS = 100000

class SimulationManager():
	"""
	--- SIMULATION CLASS---
	"""

	def __init__(self):
		"""
		Init of the node
		"""

		# Load profiles
		self.__data = DataImport()
		self.__profiles = self.__data.load_profiles()
		self.__user_profile = self.__data.find_profile(PROFILE, self.__profiles)

		# Class variables
		self.__external_stimulus_status = {"user_presence": False, "user_near": False, "user_far": False, "balls": False, "lights": False}
		self.__external_stimulus_values = {"user_presence": 0, "user_near": 0, "user_far": 0, "balls": 0, "lights": 0}
		self.__steps = 0
		self.__active_behavior = None

		# User presence signal variables
		self.__user_presence = False
		self.__presence_increment = +0.5
		self.__presence_decrement = -1.0
		self.__presence_range = [0, 100]
		self.__presence_msg = ExternalStimulus(stimulus="user_presence", status=False, value=0)

		# Lights signal variables
		self.__lights_value = 0
		self.__lights_increment = 0.5
		self.__lights_decrement = -0.5
		self.__lights_margin = 1
		self.__lights_set_value = 15
		self.__lights_range = [0, 100]
		self.__lights_msg = ExternalStimulus(stimulus="lights", status=False, value=0)

		# User answers variables
		self.__give_answer = False
		self.__color_received = None
		self.__color_list = ["green", "yellow", "red", "blue"]
		self.__color_steps = 0
		self.__color_steps_counter = 0
		self.__selected_color = None

		# User far and near signals
		self.__user_near_msg = ExternalStimulus(stimulus="user_near", status=False, value=0)
		self.__user_far_msg = ExternalStimulus(stimulus="user_far", status=False, value=0)
		self.__far_steps = 0
		self.__near_steps = 0
		self.__near_counter_steps = 0
		self.__far_counter_steps = 0

		# Touch stimulus
		self.__generate_touch = False
		self.__last_touch_time = 0
		self.__touch_msg = MiroTouch()
		self.__touch_action_msg = ExogenousAction(action_name="touch", agent_object="user", status=True, value=0)
		self.__generate_long_touch = False

		# Balls stimulus
		self.__balls_set = 0
		self.__balls_msg = ExternalStimulus(stimulus="balls", status=False, value=0)
		self.__balls_range = [0, 100]
		self.__balls_increment = 0.2
		self.__balls_decrement = -1
		self.__balls_margin = 1
		self.__balls_timer = None
		self.__balls_duration = random.uniform(1,3) * TIME_CORRECTION

		if bool(self.__user_profile):

			#########   SIMULATION PARAMS   ##########
			# Presence times
			self.__time_in_arena = self.__user_profile["time_in_arena"] * TIME_CORRECTION
			self.__time_out_arena = self.__user_profile["time_out_arena"] * TIME_CORRECTION
			self.__out_timer = rospy.Timer(rospy.Duration(self.__time_out_arena), self.__out_cb, oneshot=True)
			self.__in_timer = None

			# Answers probabilities
			self.__ignore_robot_prob = self.__user_profile["ignore_robot_probabilities"]
			self.__answer_prob = self.__user_profile["answers_probabilities"]

			# Touch stimulus
			self.__touch_frequency = self.__user_profile["touch_frequency"] * TIME_CORRECTION
			self.__touch_timer = None
			self.__response_interaction_request_prob = self.__user_profile["response_interaction_request"]
			self.__generate_touch_prob = self.__user_profile["generate_touch_prob"]

			# Balls stimulus
			self.__balls_prob = self.__user_profile["balls_probabilities"]

			# Logging profile used in simulation
			self.__profile_log = Logger("profile", FILENAME, 'profile')
			self.__profile_log.write_labels("\n\nPROFILE OF THE USER\n")
			for key, value in self.__user_profile.items():
				self.__profile_log.write_data(key.title()+": "+str(value), timestamp=False)

		else:

			rospy.logfatal("Profile set is not correct, stopping simulation")
			self.__stop_simulation_pub.publish(Empty())
			rospy.signal_shutdown()

		# Creating publishers and subscribers
		self.__create_msg_srv()

		rospy.sleep(1)

		self.__progression_timer = rospy.Timer(rospy.Duration(TIME_STEP*100), self.__progression_cb, oneshot=False)

	def __create_msg_srv(self):

		# Publishers
		self.__external_stimulus_pub = rospy.Publisher("perception_manager/external_stimulus", ExternalStimulus, queue_size=5, latch=True)
		self.__external_stimulus_status_pub = rospy.Publisher("perception_manager/external_stimulus_status", ExternalStimulus, queue_size=5, latch=True)
		self.__color_detected_pub = rospy.Publisher("perception_manager/color", String, queue_size=5, latch=True)
		self.__exogenous_actions_pub = rospy.Publisher("perception_manager/exogenous_action", ExogenousAction, queue_size=5, latch=True)
		self.__touch_pub = rospy.Publisher("perception_manager/touch", MiroTouch, queue_size=5, latch=True)
		self.__stop_simulation_pub = rospy.Publisher("motivational_dms/stop_simulation", Empty, queue_size=5, latch=True)

		# Subscribers
		self.__behavior_info_sub = rospy.Subscriber("motivational_dms/behavior_information", BehaviorInfo, self.__behavior_cb)
		self.__selected_color_sub = rospy.Subscriber("motivational_dms/selected_color", String, self.__color_cb)

	def save_log(self):
		# logging
		pass

	def generate_presence_signal(self):
		
		if self.__user_presence:
			self.__presence_msg.value += self.__presence_increment
		else:
			self.__presence_msg.value += self.__presence_decrement

		self.__presence_msg.value = min(max(self.__presence_msg.value, self.__presence_range[0]), self.__presence_range[1])

		if self.__presence_msg.value > 1:
			self.__presence_msg.status = True
		else:
			self.__presence_msg.status = False

		if self.__presence_msg.status != self.__external_stimulus_status["user_presence"]:
			self.__external_stimulus_status_pub.publish(self.__presence_msg)
			self.__external_stimulus_status["user_presence"] = self.__presence_msg.status

		if self.__presence_msg.value != self.__external_stimulus_values["user_presence"]:
			self.__external_stimulus_pub.publish(self.__presence_msg)
			self.__external_stimulus_values["user_presence"] = self.__presence_msg.value

	def generate_balls_stimulus(self):
		
		if self.__user_presence:
			if numpy.random.choice([True, False], p=self.__balls_prob):
				self.__balls_set = random.uniform(10,90)

		if (self.__balls_set-self.__balls_margin) < self.__balls_msg.value < (self.__balls_set+self.__balls_margin):
			self.__balls_msg.value = self.__balls_set
			self.__balls_timer = rospy.Timer(rospy.Duration(self.__balls_duration), self.__balls_cb, oneshot=True)
		elif self.__balls_msg.value > self.__lights_set_value:
			self.__balls_msg.value += self.__balls_decrement
		elif self.__balls_msg.value < self.__lights_set_value:
			self.__balls_msg.value += self.__balls_increment

		self.__balls_msg.value = min(max(self.__balls_msg.value, self.__balls_range[0]), self.__balls_range[1])

		if self.__balls_msg.value > 1:
			self.__balls_msg.status = True
		else:
			self.__balls_msg.status = False

		if self.__balls_msg.status != self.__external_stimulus_status["balls"]:
			self.__external_stimulus_status_pub.publish(self.__balls_msg)
			self.__external_stimulus_status["balls"] = self.__balls_msg.status

		if self.__balls_msg.value != self.__external_stimulus_values["balls"]:
			self.__external_stimulus_pub.publish(self.__balls_msg)
			self.__external_stimulus_values["balls"] = self.__balls_msg.value

	def generate_light_signal(self):


		if (self.__lights_msg.value-self.__lights_margin) < self.__lights_set_value < (self.__lights_msg.value+self.__lights_margin):
			self.__lights_msg.value = self.__lights_set_value
		elif self.__lights_msg.value > self.__lights_set_value:
			self.__lights_msg.value += self.__lights_decrement
		elif self.__lights_msg.value < self.__lights_set_value:
			self.__lights_msg.value += self.__lights_increment

		if numpy.random.choice([True,False],p=[0.01,0.99]):
			if random.choice([True,False]):
				self.__lights_set_value += 0.1
			else:
				self.__lights_set_value -= 0.1


		self.__lights_msg.value = min(max(self.__lights_msg.value, self.__lights_range[0]), self.__lights_range[1])

		if self.__lights_msg.value > 0:
			self.__lights_msg.status = True
		else:
			self.__lights_msg.status = False

		if self.__lights_msg.status != self.__external_stimulus_status["lights"]:
			self.__external_stimulus_status_pub.publish(self.__lights_msg)
			self.__external_stimulus_status["lights"] = self.__lights_msg.status

		if self.__lights_msg.value != self.__external_stimulus_values["lights"]:
			self.__external_stimulus_pub.publish(self.__lights_msg)
			self.__external_stimulus_values["lights"] = self.__lights_msg.value

	def generate_color_response(self):
		
		if self.__give_answer:
			if self.__color_steps > self.__color_steps_counter:
				self.__color_steps_counter += 1
			else:
				self.__color_steps = 0
				self.__color_steps_counter = 0
				self.__give_answer = False
				if self.__user_presence:
					if numpy.random.choice([True,False], p=self.__answer_prob):
						self.__selected_color = self.__color_received
					else:
						aux_list = list(self.__color_list)
						aux_list.remove(self.__color_received)
						self.__selected_color = random.choice(aux_list)
					self.__color_detected_pub.publish(self.__selected_color)
				self.__selected_color = None

	def generate_robot_perception(self):
		
		if self.__active_behavior == "find_user":
			if self.__far_steps > self.__far_counter_steps:
				self.__far_counter_steps += 1
			else:
				self.__far_counter_steps = 0
				self.__user_far_msg.status = True
				self.__user_near_msg.status = False
		elif self.__active_behavior == "approach_user":
			if self.__near_steps > self.__near_counter_steps:
				self.__near_counter_steps += 1
			else:
				self.__near_counter_steps = 0
				self.__user_far_msg.status = False
				self.__user_near_msg.status = True

		if self.__user_far_msg.status != self.__external_stimulus_status["user_far"]:
			self.__external_stimulus_status["user_far"] = self.__user_far_msg.status
			self.__external_stimulus_status_pub.publish(self.__user_far_msg)
			self.__external_stimulus_pub.publish(self.__user_far_msg)

		if self.__user_near_msg.status != self.__external_stimulus_status["user_near"]:
			self.__external_stimulus_status["user_near"] = self.__user_near_msg.status
			self.__external_stimulus_status_pub.publish(self.__user_near_msg)
			self.__external_stimulus_pub.publish(self.__user_near_msg)

	def generate_touch_stimulus(self):
		
		if self.__generate_touch and self.__user_presence:
			
			if self.__generate_long_touch:
				duration = random.uniform(5,10)
				self.__generate_long_touch = False
			else:
				duration = random.uniform(0.2,8)

			if duration < 0.5:
				intensity = 's'
			elif duration < 5:
				intensity = 'm'
			else:
				intensity = 'l'

			area = random.choice(["head", "shell"])

			last_touch = rospy.get_time() - self.__last_touch_time

			self.__touch_msg = MiroTouch(id=area, intensity=intensity, duration=duration, last_touch=last_touch, status=True)
			self.__touch_pub.publish(self.__touch_msg)
			self.__touch_action_msg.value = duration
			self.__exogenous_actions_pub.publish(self.__touch_action_msg)

			self.__generate_touch = False


	def run(self):

		rospy.loginfo("STARTING SIMULATION")

		while not rospy.is_shutdown():

			if self.__steps >= SIMULATION_STEPS:
				rospy.loginfo("END OF SIMULATION REACHED")
				self.__stop_simulation_pub.publish(Empty())
				break
			
			self.generate_presence_signal()
			self.generate_light_signal()
			self.generate_color_response()
			self.generate_robot_perception()
			self.generate_touch_stimulus()
			self.generate_balls_stimulus()
			rospy.sleep(TIME_STEP)
			self.__steps += 1

	def stop(self):

		print "STOPPING THE NODE"

######################################################
############# SUBSCRIBERS FUNCTIONS ##################
######################################################

	def __behavior_cb(self, msg):

		if msg.id in ["find_user", "approach_user"]:
			self.__active_behavior = msg.id
			if self.__active_behavior == "find_user":
				self.__far_counter_steps = random.randint(0,10)
			elif self.__active_behavior == "approach_user":
				self.__near_counter_steps = random.randint(0,10)

		elif msg.id == "request_interaction":
			if numpy.random.choice([True,False],p=self.__response_interaction_request_prob):
				self.__generate_touch = True
				self.__generate_long_touch = True
				variation = random.uniform(-2,2) * TIME_CORRECTION
				self.__touch_timer.shutdown()
				self.__touch_timer = rospy.Timer(rospy.Duration(self.__touch_frequency+variation), self.__touch_timer_cb, oneshot=True)
		else:
			self.__active_behavior = None
			self.__user_far_msg.status = False
			self.__user_near_msg.status = False

	def __color_cb(self, msg):
		
		if msg.data in self.__color_list:
			if numpy.random.choice([True, False], p=self.__ignore_robot_prob):
				self.__color_received = msg.data
				self.__give_answer = True
				self.__color_steps = random.randint(10,40)

######################################################
################## TIMER FUNCTIONS ###################
######################################################

	def __in_cb(self, event):
		
		self.__user_presence = False
		variation = random.uniform(-2,2) * TIME_CORRECTION
		self.__out_timer = rospy.Timer(rospy.Duration(self.__time_out_arena+variation), self.__out_cb, oneshot=True)
		if self.__touch_timer is not None:
			self.__touch_timer.shutdown()
	
	def __out_cb(self, event):
		
		self.__user_presence = True
		variation = random.uniform(-2,2) * TIME_CORRECTION
		self.__in_timer = rospy.Timer(rospy.Duration(self.__time_in_arena+variation), self.__in_cb, oneshot=True)
		variation = random.uniform(-2,2) * TIME_CORRECTION
		self.__touch_timer = rospy.Timer(rospy.Duration(self.__touch_frequency+variation), self.__touch_timer_cb, oneshot=True)

	def __touch_timer_cb(self, event):
		
		if numpy.random.choice([True, False], p=self.__generate_touch_prob):
			self.__generate_touch = True
		
		variation = random.uniform(-2,2) * TIME_CORRECTION
		self.__touch_timer = rospy.Timer(rospy.Duration(self.__touch_frequency+variation), self.__touch_timer_cb, oneshot=True)

	def __balls_cb(self, event):
		
		self.__balls_set = 0
	

	def __progression_cb(self, msg):
		
		# Logging progression
		rospy.loginfo("%d%% Simulation completed", (float(self.__steps)*100/SIMULATION_STEPS))


if __name__ == '__main__':
    try:
        # start the node
        rospy.init_node(node_name)

        # create and spin the node
        node = SimulationManager()

        rospy.on_shutdown(node.stop)

        # spin the node
        node.run()
    except rospy.ROSInterruptException:
        pass