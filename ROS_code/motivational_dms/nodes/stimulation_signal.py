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

import rospy
import math

from std_msgs.msg import Float32, Empty
from miro_perception_manager.msg import MiroTouch

pkg_name = "motivational_dms"
node_name = "stimulation_signal"

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

class Stimulation():
	""" ---Touch Manager Class---
	    --- MIRO touch sensor raw data interpreter ---
	"""

	def __init__(self):
		"""
		Init of the node
		"""

		# Class variables
		self.__stimulation_value = 0
		self.__prev_stimulation_value = None
		self.__variation_value = 0
		self.__variate_value = False
		self.__set_value = 0
		self.__increasement_value = 0.1
		self.__decreasement_value = 0.5
		self.__margin = 0.3
		self.__bonus = 20
		self.__bonus_applied = False
		self.__stop_signal = False

		# Touch variables
		self.__prev_touch = None
		self.__current_touch = None
		self.__previous_touches_limit = 100
		self.__previous_touches_counter = 0

		# Timers and counters
		self.__satisfaction_time = 60 * CORRECTION_TIME
		self.__satisfaction_timer = rospy.Timer(rospy.Duration(self.__satisfaction_time), self.__time_cb, oneshot=True)

		# Define variables of the node
		self.__create_msg_srv()

		rospy.sleep(1)

	def __create_msg_srv(self):
		"""
		Creation of publishers and subscribers
		"""

		# Publishers
		self.__sti_sig_pub = rospy.Publisher("motivational_dms/stimulation_signal", Float32, queue_size=5, latch=True)

		# Subscribers
		self.__touch_sub = rospy.Subscriber("perception_manager/touch", MiroTouch, self.__touch_cb)
		self.__stop_signal_sub = rospy.Subscriber("motivational_dms/stop_simulation", Empty, self.__stop_cb)	

	def update_signal_value(self):
		
		self.evaluate_touch()

		self.__stimulation_value = min(max(self.__stimulation_value, 0), 100)

		if self.__stimulation_value != self.__prev_stimulation_value:
			self.__sti_sig_pub.publish(self.__stimulation_value)
			self.__prev_stimulation_value = self.__stimulation_value

	def evaluate_touch(self):

		if self.__current_touch is not None:
			if self.__prev_touch is None:
				self.__set_value += self.__current_touch["duration"]
				self.__variation_value = self.__increasement_value
				self.__variate_value = True
			else:
				if self.__previous_touches_counter > self.__previous_touches_limit:
					self.__set_value -= self.__current_touch["duration"] * (self.__previous_touches_counter - self.__previous_touches_limit)
					self.__variation_value = self.__decreasement_value * (self.__previous_touches_counter - self.__previous_touches_limit) 
					self.__variate_value = True
					if self.__previous_touches_counter == (self.__previous_touches_limit+1):
						self.__stimulation_value -= self.__bonus
						self.__set_value -= self.__bonus
				else:
					self.__set_value += self.__current_touch["duration"] * (self.__previous_touches_limit - (self.__previous_touches_counter-1))
					self.__variation_value = self.__increasement_value * (self.__previous_touches_limit - (self.__previous_touches_counter-1)) 
					self.__variate_value = True

			if bool(self.__current_touch):
				self.__prev_touch = self.__current_touch.copy()

			self.__current_touch = None

		if self.__variate_value:
			if (self.__set_value - self.__margin) < self.__stimulation_value < (self.__set_value + self.__margin):
				self.__stimulation_value = self.__set_value
				self.__variate_value = False
				if not SIMULATION:
					rospy.loginfo("VALUE ADAPTED TO SET POINT")
			elif self.__stimulation_value > self.__set_value:	
				self.__stimulation_value -= self.__variation_value
				if not SIMULATION:
					rospy.loginfo("DECREASING VALUE")
			elif self.__stimulation_value < self.__set_value:
				if self.__prev_touch is None and not self.__bonus_applied:
					self.__stimulation_value += self.__bonus
					self.__bonus_applied = True
					self.__set_value += self.__bonus
				self.__stimulation_value += self.__variation_value
				if not SIMULATION:
					rospy.loginfo("INCREASING VALUE")

	def run(self):

		while not rospy.is_shutdown():

			if self.__stop_signal:
				break

			self.update_signal_value()

			rospy.sleep(TIME_STEP)

	def stop(self):

		print "Stopping the node"

######################################################
############# SUBSCRIBERS FUNCTIONS ##################
######################################################

	def __touch_cb(self, msg):

		if not SIMULATION:
			rospy.loginfo('NEW TOUCH RECEIVED')
		if not self.__satisfaction_timer is None:
			self.__satisfaction_timer.shutdown()
			self.__satisfaction_timer = None

		self.__current_touch = {'area': msg.id, 'intensity': msg.intensity, \
		 						'duration': msg.duration, 'last_touch_time': msg.last_touch}

		self.__previous_touches_counter += 1
		if self.__previous_touches_counter > 5:
			self.__previous_touches_counter = 5
		self.__set_value = self.__stimulation_value

		self.__satisfaction_timer = rospy.Timer(rospy.Duration(self.__satisfaction_time), self.__time_cb, oneshot=True)

	def __time_cb(self, event):

		if not SIMULATION:
			rospy.loginfo('TIMER FINISHED, RESETTING')
		self.__variate_value = True
		self.__variation_value = self.__decreasement_value
		self.__current_touch = None
		self.__prev_touch = None
		self.__previous_touches_counter = 0
		self.__set_value = 0
		self.__bonus_applied = False

	def __stop_cb(self, msg):
		
		self.__stop_signal = True

if __name__ == '__main__':
    try:
        # start the node
        rospy.init_node(node_name)

        # create and spin the node
        node = Stimulation()

        rospy.on_shutdown(node.stop)

        # spin the node
        node.run()
    except rospy.ROSInterruptException:
        pass