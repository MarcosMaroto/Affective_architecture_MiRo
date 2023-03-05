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

pkg_name = "perception"


import rospy
import numpy

from std_msgs.msg import UInt16MultiArray
from sensor_msgs.msg import JointState, Illuminance
from miro_perception_manager.msg import MiroTouch
from miro_perception_manager.srv import CorrectionAngle
from motivational_dms.msg import ExternalStimulus, ExogenousAction


illu_data_range = [0, 8]
illu_signal_range = [0, 100]
no_faces_limit = 5
faces_limit = 2
threshold = 1

# Touch factor to multiply duration
touch_factor = 4

class PerceptionManager():
	""" ---Perception Manager Class---
	    --- MIRO  sensor raw data interpreter ---
	"""

	def __init__(self):
		"""
		Init of the node
		"""

		# Class variables
		self.__touch_shell_status = (0,0,0,0)
		self.__touch_head_status = (0,0,0,0)
		self.__touch_shell_time = None
		self.__touch_head_time = None
		self.__last_touch_time = 0
		self.__touch_action_msg = ExogenousAction(action_name="touch", agent_object="user", status=True)
		self.__correction_angle = 0
		self.__illumination_value = 0
		self.__prev_illu_value = 0
		self.__illu_mean = list()
		self.__status_dict = {"lights": False}

		# Define variables of the node
		self.__create_msg_srv()


	def __create_msg_srv(self):
		"""
		Creation of publishers and subscribers
		"""

		# Publishers
		self.__touch_pub = rospy.Publisher("perception_manager/touch", MiroTouch, queue_size=5, latch=True)
		self.__ext_stimulus_pub = rospy.Publisher("perception_manager/external_stimulus", ExternalStimulus, queue_size=5, latch=True)
		self.__ext_stimulus_status_pub = rospy.Publisher("perception_manager/external_stimulus_status", ExternalStimulus, queue_size=5, latch=True)
		self.__exogenous_action_pub = rospy.Publisher("perception_manager/exogenous_action", ExogenousAction, queue_size=5, latch=True)

		# Subscribers
		self.__touch_sub = rospy.Subscriber("miro/rob01/sensors/touch", UInt16MultiArray, self.__raw_touch_cb)
		self.__kinematic_sub = rospy.Subscriber("miro/rob01/sensors/kinematic_joints", JointState, self.__kinematic_cb)
		self.__illuminance_sub = rospy.Subscriber("miro/rob01/sensors/light", UInt16MultiArray, self.__illuminance_cb)

	def __check_touch_states(self):
		"""
		Check touch states
		"""

		# If a touch on the head is perceived, start a timer to calculate duration
		# else, calculate duration and publish it
		if True in self.__touch_head_status:
			if self.__touch_head_time is None:
				self.__touch_head_time = rospy.get_time()
				rospy.loginfo("Starting touch head timer")
		else:
			if not self.__touch_head_time is None:
				self.__touch_head_time = rospy.get_time() - self.__touch_head_time
				self.__last_touch_time = rospy.get_time() - self.__last_touch_time
				rospy.loginfo('Touching time on the head is %f', self.__touch_head_time)
				self.__check_touch_duration('head', self.__touch_head_time, self.__last_touch_time)
				self.__touch_head_time = None

		# If a touch on the head is perceived, start a timer to calculate duration
		# else, calculate duration and publish it
		if True in self.__touch_shell_status:
			if self.__touch_shell_time is None:
				self.__touch_shell_time = rospy.get_time()
				rospy.loginfo("Starting touch shell timer")
		else:
			if not self.__touch_shell_time is None:
				self.__touch_shell_time = rospy.get_time() - self.__touch_shell_time
				self.__last_touch_time = rospy.get_time() - self.__last_touch_time
				rospy.loginfo('Touching time on the shell is %f', self.__touch_shell_time)
				self.__check_touch_duration('shell', self.__touch_shell_time, self.__last_touch_time)
				self.__touch_shell_time = None

	def __check_touch_duration(self, place, duration, last_touch):
		"""
		Funcion that calculates the duratin of each touch and publishes it
		"""

		# Custom touch msg
		msg = MiroTouch()

		# Formation of the msg
		msg.id = place
		msg.duration = duration
		self.__touch_action_msg.value = duration * touch_factor
		msg.last_touch = last_touch
		msg.status = True

		if duration < 0.4:
			msg.intensity = 's'
		elif duration < 5:
			msg.intensity = 'm'
		elif duration > 5:
			msg.intensity = 'l'

		self.__touch_pub.publish(msg)
		self.__exogenous_action_pub.publish(self.__touch_action_msg)

	def __check_illumination(self):
		"""
		Function that calculates the level of illumination and publishes it
		"""

		# Change  range of the illumination siganl from [0,20] to [0, 100]
		set_value = self.__illumination_value * illu_signal_range[1] / float(illu_data_range[1])
		set_value = (set_value - 100) * (-1)
		
		if self.__prev_illu_value > set_value + threshold:
			value = self.__prev_illu_value - 0.5
		elif self.__prev_illu_value < set_value - threshold:
			value = self.__prev_illu_value + 0.5
		else:
			value = set_value

		value = min(max(value, illu_signal_range[0]), illu_signal_range[1])

		if value < 5:
			status = False
		else: 
			status = True

		# If value has changed, publish it
		if value != self.__prev_illu_value:
			self.__ext_stimulus_pub.publish(stimulus="lights", status=status, value=value)
			if status != self.__status_dict["lights"]:
				self.__status_dict["lights"] = status
				self.__ext_stimulus_status_pub.publish(stimulus="lights", status=status, value=value)
			self.__prev_illu_value = value		

	def run(self):

		while not rospy.is_shutdown():

			self.__check_touch_states()
			self.__check_illumination()

			rospy.sleep(0.5)

	def stop(self):

		print "Stopping the node"

######################################################
############# SUBSCRIBERS FUNCTIONS ##################
######################################################

	def __raw_touch_cb(self, msg):
		"""
		Touch raw data from Miro's touch sensors
		"""

		# Save shell and head information
		if msg.data[0:4] != self.__touch_head_status:
			self.__touch_head_status = msg.data[0:4]
		if msg.data[4:8] != self.__touch_shell_status:
			self.__touch_shell_status = msg.data[4:8]

	def __kinematic_cb(self, msg):
		"""
		Obtention of the yaw angle of Miro's head
		"""		
		
		self.__correction_angle = msg.position[2]

	def __illuminance_cb(self, msg):
		"""
		Function that gets the light level from the sensors of Miro
		"""
		
		# Mean value of four sensors
		self.__illu_mean.append(numpy.mean(msg.data))
		if len(self.__illu_mean) >= 5:
			del self.__illu_mean[0] 
		self.__illumination_value = numpy.mean(self.__illu_mean)
		self.__illumination_value = min(max(self.__illumination_value, illu_data_range[0]), illu_data_range[1])

	def __faces_cb(self, msg):
		"""
		Function that gets if the user is present
		"""
		if msg.faces:
			self.__face_detected = True
			self.__no_faces_counter = 0
			self.__faces_counter += 1
		else:
			self.__face_detected = False
			self.__no_faces_counter += 1
			self.__faces_counter = 0

######################################################
############# SERVICES FUNCTIONS ##################
######################################################

	def __get_angle(self, req):
		
		return self.__correction_angle

if __name__ == '__main__':
    try:
        # start the node
        rospy.init_node(pkg_name)

        # create and spin the node
        node = PerceptionManager()

        rospy.on_shutdown(node.stop)

        # spin the node
        node.run()
    except rospy.ROSInterruptException:
        pass
