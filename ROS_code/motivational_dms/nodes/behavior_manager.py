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
node_name = "behavior_manager"

import rospy
import random
import numpy

from datetime import datetime as dt
from std_msgs.msg import Float32, String, Empty, UInt16MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from miro_msgs.msg import platform_control, bridge_stream
from motivational_dms.msg import BehaviorInfo, BehaviorResult, ApproachUser
from classes.load_data import DataImport
from miro_perception_manager.msg import MiroTouch
from classes.logger import Logger

# Simulation flag
SIMULATION = rospy.get_param("decision_making/simulate")

# Real time step value used when simulation is not running
REAL_TIME_STEP = 0.2

# FILENAME USED FOR LOGGING DATA
FILENAME = dt.now().strftime("%Y%m%d-%H%M")

# Time step used in case of simulation
if SIMULATION:
	TIME_STEP = 0.01
	CORRECTION_TIME = TIME_STEP/REAL_TIME_STEP
else:
	TIME_STEP = REAL_TIME_STEP
	CORRECTION_TIME = 1

colors = ["green", "yellow", "red"]
color_rgb = {"green": [0,255,0], "yellow": [200,200,0], "red": [255,0,0]}

available_sounds = [14, 15, 16, 17]

class BehaviorsManager():
	""" 
	--- BEHAVIOR MANAGER CLASS---
	"""

	def __init__(self):
		"""
		Init of the node
		"""

		# Class variables
		self.__selected_behavior = None
		self.__active_behavior = None
		self.__behavior_info = None
		self.__executing_behavior = False
		self.__unit_counter = 0
		self.__loop_counter = 0
		self.__unit_timer = None
		self.__timeout = True
		self.__current_unit = None
		self.__cancel_behavior = False
		self.__touch_detected = False
		self.__last_behavior = None
		self.__msg_move = None
		self.__stop_signal = False

		# Emotion variables
		self.__emotion_params = dict()

		# Movement variables
		self.__accumulated_angle = 0
		self.__publish_movement = False
		self.__execute_random_behaviour = False
		self.__approach_turn = False
		self.__sonar_range = 0
		self.__sonar_limit = 0.3
		self.__cliff = [False, False]
		self.__approach = None

		# Game varaibles
		self.__selected_color = None
		self.__waiting_response = False
		self.__color_received = None

		# Load behaviors
		self.__data = DataImport()
		self.__behaviors = self.__data.load_behavior_units()
		self.__emotions = self.__data.load_emotions()

		# Logging profile used in simulation
		self.__joints_log = Logger("joints_data", FILENAME + "/decision_making", 'joints')
		self.__joints_log.write_labels("TIMESTAMP;LIFT;PITCH:YAW;TAIL;EARS;EYELIDS;MOVEMENT SPEED")

		# Define variables of the node
		self.create_msg_srv()

	def create_msg_srv(self):
		"""
		Creation of publishers and subscribers
		"""

		# Publishers
		self.__behavior_pub = rospy.Publisher("miro/rob01/platform/control", platform_control, queue_size=1, latch=True)
		self.__behavior_result_pub = rospy.Publisher("motivational_dms/behavior_result", BehaviorResult, queue_size=1, latch=True)
		self.__behavior_sound_pub = rospy.Publisher("miro/rob01/bridge/stream", bridge_stream, queue_size=1, latch=True)
		self.__selected_color_pub = rospy.Publisher("motivational_dms/selected_color", String, queue_size=1, latch=True)
		
		# Subscribers
		self.__behavior_info_sub = rospy.Subscriber("motivational_dms/behavior_information", BehaviorInfo, self.__behavior_cb)
		self.__cancel_sub = rospy.Subscriber("motivational_dms/cancel_behavior", Empty, self.__cancel_cb)
		self.__sonar_sub = rospy.Subscriber("miro/rob01/sensors/sonar_range", Range, self.__sonar_cb)
		self.__cliff_sub = rospy.Subscriber("miro/rob01/sensors/cliff", UInt16MultiArray, self.__cliff_cb)
		self.__color_detected_sub = rospy.Subscriber("perception_manager/color", String, self.__color_cb)
		self.__approach_sub = rospy.Subscriber("perception_manager/approach", ApproachUser, self.__approach_cb)
		self.__emotion_sub = rospy.Subscriber("motivational_dms/dominant_emotion", String, self.__emotion_cb)
		self.__stop_signal_sub = rospy.Subscriber("motivational_dms/stop_simulation", Empty, self.__stop_cb)	
	
	def select_behavior(self):
		
		possible_behaviors = list()
		probabilities = list()

		self.__selected_behavior = None

		if self.__behavior_info.id:
			for behavior in self.__behaviors:
				if behavior["behavior_name"] == self.__behavior_info.id:
					self.__selected_behavior = behavior.copy()
					if self.__selected_behavior.has_key("behaviors"):
						if self.__behavior_info.behavior_id:
							for idx, beh in enumerate(self.__selected_behavior["behaviors"]):
								if self.__behavior_info.behavior_id == str(self.__selected_behavior["behaviors"][idx]["behavior"]):
									self.__active_behavior = self.__selected_behavior["behaviors"][idx]

								elif str(self.__selected_behavior["behaviors"][idx]["behavior"]) == "finish":
									self.__last_behavior = self.__selected_behavior["behaviors"][idx]
						else:
							possible_behaviors = self.__selected_behavior["behaviors"]
							break
			
			if bool(possible_behaviors):
				for item in possible_behaviors:
					for key, value in item.items():
						if key == "probability":
							probabilities.append(value)
						elif key == "behavior" and value == "finish":
							self.__last_behavior = item
				self.__active_behavior = numpy.random.choice(possible_behaviors,p=probabilities)

			if self.__selected_behavior is not None:
				self.__executing_behavior = True
				if not SIMULATION:
					rospy.loginfo("Executing %s behavior", self.__selected_behavior["behavior_name"])
				if self.__active_behavior is not None and not SIMULATION:
					rospy.loginfo("Active behavior is %s", str(self.__active_behavior["behavior"]))
					
				self.__behavior_result_pub.publish(behavior_name=self.__selected_behavior["behavior_name"], status= "started")

	def select_unit_from_behavior(self, behavior):

		possible_behaviors = list()
		probabilities = list()
		active_behavior = None

		if behavior.has_key("behaviors"):
			possible_behaviors = behavior["behaviors"]
			probabilities = list()
			for item in possible_behaviors:
				for key, value in item.items():
					if key == "probability":
						probabilities.append(value)
			active_behavior = numpy.random.choice(possible_behaviors,p=probabilities)

		return active_behavior

	def execute_unit(self):

		if self.__timeout and not self.__selected_behavior is None:
			if len(self.__active_behavior["units"]) > self.__unit_counter:
				self.__current_unit = self.__active_behavior["units"][self.__unit_counter]
				if bool(self.__emotion_params):
					self.__unit_timer = rospy.Timer(rospy.Duration((self.__current_unit["execution_time"]+self.__emotion_params["time_factor"])*CORRECTION_TIME), self.__unit_timer_cb, oneshot=True)
				else:
					self.__unit_timer = rospy.Timer(rospy.Duration(self.__current_unit["execution_time"]*CORRECTION_TIME), self.__unit_timer_cb, oneshot=True)
				self.__timeout = False
				if not SIMULATION:
					rospy.loginfo("Executing step %d/%d of behavior %s",self.__unit_counter+1, len(self.__active_behavior["units"]), self.__selected_behavior["behavior_name"].upper())
				self.__unit_counter += 1
				self.__publish_movement = True

			if len(self.__active_behavior["units"]) == self.__unit_counter:
				if self.__selected_behavior["loops_number"] > self.__loop_counter + 1:
					self.__loop_counter += 1
					self.__unit_counter = 0
					self.__active_behavior = self.select_unit_from_behavior(self.__selected_behavior)
				elif self.__selected_behavior["loops_number"] > self.__loop_counter:
					self.__loop_counter += 1
					self.__unit_counter = 0
					self.__active_behavior = self.__last_behavior
				else:
					self.__cancel_behavior = True

		elif self.__cancel_behavior:
			self.__behavior_result_pub.publish(behavior_name=self.__selected_behavior["behavior_name"], status= "finished")
			self.reset_all_params()

	def send_unit(self, unit):
		
		self.__msg_move = self.complete_msg(unit["command"])

		# If executing a sequence, check if sonar and cliff sensors are activated for avoiding obstacle
		if True in self.__cliff:
			if self.__msg_move is not None:
				self.__msg_move.body_vel.linear.x = -200

		# Save log before publishing
		aux_str = str(self.__msg_move.body_config[1])+";"+str(self.__msg_move.body_config[3])+";"+str(self.__msg_move.body_config[2])+";"
		aux_str += str(self.__msg_move.tail)+";"+str(numpy.mean(self.__msg_move.ear_rotate))+";"+str(self.__msg_move.eyelid_closure)+";"
		aux_str += str(self.__msg_move.body_vel.linear.x)
		self.__joints_log.write_data(aux_str)

		if not SIMULATION:
			self.__behavior_pub.publish(self.__msg_move)

		if self.__msg_move.sound_index_P1 != 0:
			self.__current_unit["command"]["sound_index_P1"] = 0

	def complete_msg(self, values):

		msg = platform_control()

		for key, value in values.items():
			if key == "body_vel":
				msg_twist = Twist()
				for sub_key, sub_value in value.items():
					for coor_key, coor_value in sub_value.items():
						if sub_key == "linear":
							setattr(msg_twist.linear, coor_key, coor_value)
						elif sub_key == "angular":
							setattr(msg_twist.angular, coor_key, coor_value)
				msg.body_vel = msg_twist

			else:
				if self.__selected_behavior["express_emotion"] and bool(self.__emotion_params)and self.__selected_behavior["type"] != "emotional":
					aux_value = value
					if key == "tail":
						if self.__emotion_params["emotion"] in ["surprise", "joy", "anger"]:
							aux_value = 1
						elif self.__emotion_params["emotion"] in ["sadness"]:
							aux_value = 0

						setattr(msg, key, aux_value)

					elif key == "eyelid_closure":
						if self.__emotion_params["emotion"] in ["surprise", "joy", "calmness"]:
							aux_value = 0
						elif self.__emotion_params["emotion"] in ["sadness"]:
							aux_value = 0.35
						elif self.__emotion_params["emotion"] in ["anger"]:
							aux_value = 0.1

						setattr(msg, key, aux_value)
					elif key == "body_config":
						for idx, val in enumerate(value):
							if idx != 1 and key != "body_config":
								if val > 0:
									aux_value[idx] = val + self.__emotion_params["gestures_factor"]
									aux_value[idx] = min(max(aux_value[idx],0.3), 1.0)

							setattr(msg, key, aux_value)

					elif key == "body_config_speed":
						if self.__emotion_params["emotion"] in ["surprise", "joy"]:
							aux_value = [0, -1, -1, -1]
						elif self.__emotion_params["emotion"] in ["sadness"]:
							aux_value = [0, 0.4, 0.4, 0.4]
						else:
							aux_value = value

						setattr(msg, key, aux_value)

					elif key == "ear_rotate":
						for idx, val in enumerate(value):
							if self.__emotion_params["emotion"] in ["surprise", "joy"]:
								aux_value[idx] = 1
							elif self.__emotion_params["emotion"] in ["sadness"]:
								aux_value[idx] = 0
							else:
								aux_value = value
						setattr(msg, key, aux_value)
				else:
					setattr(msg, key, value)

		if bool(self.__emotion_params) and self.__selected_behavior["behavior_name"] != "play_game" and self.__selected_behavior["type"] != "emotional":
			# LIGHTS #
			msg.lights_max_drive = 255
			msg.lights_dphase = 64
			msg.lights_amp = 255
			msg.lights_rgb = random.choice(self.__emotion_params["color"])

		return msg

	def wandering(self):

		self.__msg_move = platform_control()
		self.__msg_move.msg_flags = 1

		if self.__selected_behavior["behavior_name"] == "play_alone":
			if not self.__execute_random_behaviour:
				self.__execute_random_behaviour = numpy.random.choice([True, False], p=[0.03,0.97])
				if self.__execute_random_behaviour:
					probabilities = list()
					for item in self.__selected_behavior["behaviors"]:
						for key, value in item.items():
							if key == "probability":
								probabilities.append(value)
					self.__active_behavior = numpy.random.choice(self.__selected_behavior["behaviors"],p=probabilities)
					if not SIMULATION:
						rospy.loginfo("Active behavior is %s", str(self.__active_behavior["behavior"]))

		if (self.__sonar_range > self.__sonar_limit or self.__sonar_range == 0) and not True in self.__cliff:

			if self.__execute_random_behaviour and self.__timeout:
				self.__sonar_limit = 0.05
				self.__timeout = False
				self.__current_unit = self.__active_behavior["units"][self.__unit_counter]
				if not SIMULATION:
					rospy.loginfo("Executing step %d/%d of behavior %s",self.__unit_counter+1, len(self.__active_behavior["units"]), self.__selected_behavior["behavior_name"].upper())
				self.__publish_movement = True
				self.__unit_counter += 1
				self.__unit_timer = rospy.Timer(rospy.Duration(self.__current_unit["execution_time"]*CORRECTION_TIME), self.__unit_timer_cb, oneshot=True)
				self.__msg_move = self.complete_msg(self.__current_unit["command"])
				if self.__unit_counter >= len(self.__active_behavior["units"]):
					self.__unit_counter = 0
					self.__execute_random_behaviour = False
					self.__current_unit = None
				
			elif not self.__execute_random_behaviour and self.__timeout:

				turn = numpy.random.choice([True, False], p=[0.001, 0.999])

				if turn:
					self.__msg_move.body_vel.angular.z = round(random.uniform(0.5,2),3)
					if random.choice([True,False]):
						self.__msg_move.body_vel.angular.z *= -1
				
					self.__accumulated_angle += self.__msg_move.body_vel.angular.z
				else:
					self.__msg_move.body_vel.linear.x = 100
		
		elif self.__sonar_range <= self.__sonar_limit or True in self.__cliff: 

			if True in self.__cliff:

				if self.__cliff == [True, True]:
					self.__msg_move.body_vel.linear.x = -200
				elif self.__cliff[0] is True:
					self.__msg_move.body_vel.angular.z = -0.5
				elif self.__cliff[1] is True:
					self.__msg_move.body_vel.angular.z = +0.5

			else:

				self.__msg_move.body_vel.angular.z = random.uniform(2, 3)
				self.__accumulated_angle += self.__msg_move.body_vel.angular.z
			
			self.__execute_random_behaviour = False
			self.__current_unit = False
			self.__sonar_limit = 0.3
			self.__msg_move.body_config = [0, 0.2, 0, -0.2]
			self.__msg_move.body_config_speed = [0,-1,-1,-1]
			

		if self.__cancel_behavior:
			self.__behavior_result_pub.publish(behavior_name=self.__selected_behavior["behavior_name"], status= "finished")
			self.reset_all_params()

	def approach_user(self):

		self.__msg_move = platform_control()

		if self.__approach is not None:
			if self.__approach.status and self.__approach.move:
				self.__msg_move.msg_flags = 1
				self.__msg_move.body_vel.angular.z = self.__approach.angle
				self.__msg_move.body_vel.linear.x = 100
				self.__msg_move.body_config = [0, 0.2, 0, -0.3]
				self.__msg_move.body_config_speed = [0,-1,-1,-1]
			elif self.__approach.status and not self.__approach.move:
				self.__msg_move.msg_flags = 1
				self.__msg_move.body_vel.angular.z = self.__approach.angle
				self.__msg_move.body_config = [0, 0.2, 0, -0.3]
				self.__msg_move.body_config_speed = [0,-1,-1,-1]

		if self.__cancel_behavior:
			self.__behavior_result_pub.publish(behavior_name=self.__selected_behavior["behavior_name"], status= "finished")
			self.reset_all_params()

	def play_game(self):

		if not self.__waiting_response and self.__timeout:
			if len(self.__active_behavior["units"]) > self.__unit_counter:
				self.__current_unit = self.__active_behavior["units"][self.__unit_counter]
				self.__unit_timer = rospy.Timer(rospy.Duration(self.__current_unit["execution_time"]*CORRECTION_TIME), self.__unit_timer_cb, oneshot=True)
				self.__timeout = False
				if not SIMULATION:
					rospy.loginfo("Executing step %d/%d of behavior %s",self.__unit_counter+1, len(self.__active_behavior["units"]), self.__selected_behavior["behavior_name"].upper())
				self.__unit_counter += 1
				self.__publish_movement = True
			
			if len(self.__active_behavior["units"]) == self.__unit_counter:
				self.__selected_color = random.choice(colors)
				self.__selected_color_pub.publish(self.__selected_color)
				self.__current_unit["command"]["lights_rgb"] = color_rgb[self.__selected_color]
				self.__waiting_response = True
				self.__unit_counter = 0
				self.__loop_counter += 1
		
		elif self.__waiting_response:
			if self.__color_received is not None or self.__timeout:
				if not SIMULATION:
					rospy.loginfo("Color selected: %s. Color received: %s", self.__selected_color, self.__color_received)
				if self.__selected_color == self.__color_received:
					answer = BehaviorResult().CORRECT
					str_response = "ANSWER HAS BEEN CORRECT"
			 	elif self.__selected_color != self.__color_received:
			 		answer = BehaviorResult().INCORRECT
			 		str_response = "ANSWER HAS BEEN INCORRECT"
			 	elif self.__timeout:
			 		answer = BehaviorResult().NO_ANSWER
			 		str_response = "NO ANSWER PROVIDED BY THE USER"

			 	if not SIMULATION:
			 		rospy.loginfo(str_response)
			 	self.__behavior_result_pub.publish(behavior_name=self.__selected_behavior["behavior_name"], status= "finished", answer=answer)
			 	self.__color_received = None
			 	self.__unit_timer.shutdown()
			 	self.__waiting_response = False	 	
			 	self.__timeout = True
			 	self.__cancel_behavior = True

		if self.__cancel_behavior:
			#self.__behavior_result_pub.publish(behavior_name=self.__selected_behavior["behavior_name"], status= "finished")
			self.reset_all_params()

	def evaluate_behavior_execution(self):

		if not self.__executing_behavior and self.__behavior_info is not None:
			self.select_behavior()

		if self.__selected_behavior is not None:
			if self.__selected_behavior["execution"] == "sequence":
					self.execute_unit()

		if self.__selected_behavior is not None:
			if self.__selected_behavior["execution"] == "special":
				if self.__selected_behavior["behavior_name"] in ["wander","play_alone"]:
					self.wandering()
				elif self.__selected_behavior["behavior_name"] == "approach_user":
					self.approach_user()
				elif self.__selected_behavior["behavior_name"] == "play_game":
					self.play_game()

	def publish_msg(self):

		if self.__current_unit and self.__executing_behavior:
			self.send_unit(self.__current_unit)
		
		if self.__selected_behavior is not None and not self.__execute_random_behaviour:
			if self.__selected_behavior["behavior_name"] in ["wander", "play_alone", "approach_user"]:
				if bool(self.__emotion_params) and self.__selected_behavior["express_emotion"]:
					# LIGHTS #
					self.__msg_move.lights_max_drive = 127
					self.__msg_move.lights_dphase = 64
					self.__msg_move.lights_amp = 255
					self.__msg_move.lights_rgb = random.choice(self.__emotion_params["color"])

					if not self.__execute_random_behaviour and self.__timeout:

						probability = self.__emotion_params["probability"]
						factor = self.__emotion_params["gestures_factor"]

						if self.__emotion_params["emotion"] == "joy":
							
							if numpy.random.choice([True, False],p=[0.9, 0.1]):
								self.__msg_move.tail = random.uniform(-1,1)
							self.__msg_move.eyelid_closure = 0
							if numpy.random.choice([True, False],p=[0.6, 0.4]):
								self.__msg_move.ear_rotate = [random.uniform(0.2,0.8),random.uniform(0.2,0.8)]
							if numpy.random.choice([True, False],p=[0.03, 0.97]):
								self.__msg_move.sound_index_P1 = random.choice(available_sounds)

							self.__msg_move.body_config = [0, 0.4, 0, -0.1]
							self.__msg_move.body_config_speed = [0,-1,-1,-1]
						elif self.__emotion_params["emotion"] == "sadness":
							self.__msg_move.tail = 0
							self.__msg_move.eyelid_closure = 0.35
							self.__msg_move.ear_rotate = [0, 0]
							if numpy.random.choice([True, False],p=[0.005, 0.995]):
								self.__msg_move.sound_index_P1 = random.choice(available_sounds)

							self.__msg_move.body_config = [0, 0.7, 0, 0]
							self.__msg_move.body_config_speed = [0,-1,-1,-1]

						elif self.__emotion_params["emotion"] == "anger":
							
							if numpy.random.choice([True, False],p=[0.2, 0.8]):
								self.__msg_move.tail = random.uniform(-1,1)
							self.__msg_move.eyelid_closure = 0.2
							self.__msg_move.ear_rotate = [0, 0]
							if numpy.random.choice([True, False],p=[0.05, 0.95]):
								self.__msg_move.sound_index_P1 = random.choice(available_sounds)

							self.__msg_move.body_config = [0, 0.5, 0, 0]
							self.__msg_move.body_config_speed = [0,-1,-1,-1]
						elif self.__emotion_params["emotion"] == "surprise":
							
							if numpy.random.choice([True, False],p=[0.2, 0.8]):
								self.__msg_move.tail = random.uniform(-1,1)
							self.__msg_move.eyelid_closure = 0
							self.__msg_move.blink_time = 0.5
							self.__msg_move.ear_rotate = [0, 0]
							if numpy.random.choice([True, False],p=[0.05, 0.95]):
								self.__msg_move.sound_index_P1 = random.choice(available_sounds)

							self.__msg_move.body_config = [0, 0.2, 0, 0.2]
							self.__msg_move.body_config_speed = [0,-1,-1,-1]
						else:
							if numpy.random.choice([True, False],p=[0.1+probability,0.9-probability]):
								self.__msg_move.tail = random.uniform(0+factor,1)
							if numpy.random.choice([True, False],p=[0.1+probability, 0.9-probability]):
								self.__msg_move.ear_rotate = [random.uniform(0+factor,0.8),random.uniform(0+factor,0.8)]
							if numpy.random.choice([True, False],p=[0.05+probability,0.95-probability]):
								self.__msg_move.eyelid_closure = 1
							if numpy.random.choice([True, False],p=[0.01+probability*0.1,0.99-probability*0.1]):
								self.__msg_move.sound_index_P1 = random.choice(available_sounds)

							self.__msg_move.body_config = [0, 0.3, 0, -0.1]
							self.__msg_move.body_config_speed = [0,-1,-1,-1]

						if self.__msg_move.body_vel.linear.x > 0:
							self.__msg_move.body_vel.linear.x += factor*200
						if self.__msg_move.body_vel.angular.z != 0:
							self.__msg_move.body_vel.angular.z += factor * self.__msg_move.body_vel.angular.z / abs(self.__msg_move.body_vel.angular.z)

				# Save log before publishing
				aux_str = str(self.__msg_move.body_config[1])+";"+str(self.__msg_move.body_config[3])+";"+str(self.__msg_move.body_config[2])+";"
				aux_str += str(self.__msg_move.tail)+";"+str(numpy.mean(self.__msg_move.ear_rotate))+";"+str(self.__msg_move.eyelid_closure)+";"
				aux_str += str(self.__msg_move.body_vel.linear.x)
				self.__joints_log.write_data(aux_str)

				if not SIMULATION:	
					self.__behavior_pub.publish(self.__msg_move)

	def reset_all_params(self):

		# Resetting all params
		self.__cancel_behavior = False
		self.__waiting_response = False
		self.__active_behavior = None
		self.__selected_behavior = None
		self.__unit_counter = 0
		self.__loop_counter = 0
		self.__color_received = None
		self.__timeout = False
		self.__executing_behavior = False
		self.__behavior_info = None
		self.__approach_turn = False
		self.__execute_random_behaviour = False
		self.__current_unit = None
		self.__publish_movement = False
		self.__approach = None

		if self.__unit_timer is not None:
			self.__unit_timer.shutdown()
			self.__unit_timer = None

	def run(self):

		while not rospy.is_shutdown():

			if self.__stop_signal:
				break

			self.evaluate_behavior_execution()
			self.publish_msg()

			rospy.sleep(TIME_STEP)

	def stop(self):

		if self.__unit_timer != None:
			self.__unit_timer.shutdown()
		self.__behavior_info_sub.unregister()

######################################################
############# SUBSCRIBERS FUNCTIONS ##################
######################################################

	def __behavior_cb(self, msg):

		if msg.id:
			self.__behavior_info = msg
			self.__timeout = True

	def __emotion_cb(self, msg):
		
		for emotion in self.__emotions:
			if emotion["emotion"] == msg.data:
				self.__emotion_params = emotion.copy()
				return

	def __unit_timer_cb(self, event):

		self.__timeout = True

	def __sonar_cb(self, msg):
		
		self.__sonar_range = msg.range

	def __color_cb(self, msg):

		if self.__waiting_response:
			self.__color_received = msg.data

	def __cancel_cb(self, msg):

		self.__cancel_behavior = True
		
		# Cancel the timer as soon as possible
		if self.__unit_timer is not None:
			self.__unit_timer.shutdown()

	def __approach_cb(self, msg):
		
		self.__approach = msg

	def __cliff_cb(self, msg):
		
		self.__cliff = [False,False]

		for idx, item in enumerate(msg.data):
			if item > 10:
				self.__cliff[idx] = True

	def __stop_cb(self, msg):
		
		self.__stop_signal = True

if __name__ == '__main__':
    try:
        # start the node
        rospy.init_node(node_name)

        # create and spin the node
        node = BehaviorsManager()

        rospy.on_shutdown(node.stop)

        # spin the node
        node.run()
    except rospy.ROSInterruptException:
        pass