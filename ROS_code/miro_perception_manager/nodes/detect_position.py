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

pkg_name = "position_detection"

import roslib
roslib.load_manifest("miro_perception_manager")
import rospy
import cv2
import numpy
import colorsys

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from motivational_dms.msg import ExternalStimulus, BehaviorInfo, ApproachUser

# define the lower and upper boundaries of the colors in the HSV color space
lower = {'blue_jeans':(95, 50, 0)}
upper = {'blue_jeans':(115, 110, 100)}

area_threshold = 4000
area_near_threshold = 15000

limit = [250,270]

class DetectColor():
	""" --- Detect Color Manager Class ---
	    --- Color detection of balls shown by the use ---
	"""

	def __init__(self):
		"""
		Init of the node
		"""

		# Class variables
		self.__bridge = CvBridge()
		self.__cv_image = None
		self.__color_detected_counter = dict()
		self.__external_sti_near = ExternalStimulus(stimulus="user_near",status=False)
		self.__external_sti_far = ExternalStimulus(stimulus="user_far",status=False)
		self.__prev_value = None
		self.__find_user = False
		self.__user_presence = False
		self.__correction_angle = None
		self.__stimulus_status = {"user_near": None, "user_far": None}
		self.__approach_msg = ApproachUser()

		# Define variables of the node
		self.__create_msg_srv()

	def __create_msg_srv(self):
		"""
		Creation of publishers and subscribers
		"""

		# Publishers
		self.__external_sti_status_pub = rospy.Publisher("perception_manager/external_stimulus_status", ExternalStimulus, queue_size=5, latch=True)
		self.__external_sti_pub = rospy.Publisher("perception_manager/external_stimulus", ExternalStimulus, queue_size=5, latch=True)
		self.__approach_pub = rospy.Publisher("perception_manager/approach", ApproachUser, queue_size=5, latch=True)
		
		# Subscribers
		self.__image_sub = rospy.Subscriber("miro/rob01/platform/caml", Image, self.__image_cb)
		#self.__image_sub = rospy.Subscriber("camera/rgb/image_rect_color", Image, self.__image_cb)
		self.__behavior_info_sub = rospy.Subscriber("motivational_dms/behavior_information", BehaviorInfo, self.__behavior_cb)
		self.__external_stimuli_sub = rospy.Subscriber("perception_manager/external_stimulus", ExternalStimulus, self.__external_stimuli_cb)

	def execute(self):

		if self.__cv_image is not None and self.__find_user:
			
			blurred = cv2.GaussianBlur(self.__cv_image, (11, 11), 0)
			hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

			for key, value in upper.items():
				# construct a mask for the color from dictionary`1, then perform
				# a series of dilations and erosions to remove any small
				# blobs left in the mask
				kernel = numpy.ones((9,9),numpy.uint8)
				mask = cv2.inRange(hsv, lower[key], upper[key])
				mask = cv2.dilate(mask,kernel,iterations = 4)
				mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
				mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
				       
				# find contours in the mask and initialize the current
				cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
				#center = None			
				cv2.imshow(key, mask)
			       
				# only proceed if at least one contour was found
				if len(cnts) > 0:
					# find the largest contour in the mask, then use
					c = max(cnts, key=cv2.contourArea)
					area = cv2.contourArea(c)
					M = cv2.moments(c)

					x = int(M['m10']/M['m00'])
					y = int(M['m01']/M['m00'])

					if area > area_threshold:
						# The user has been detected
						cv2.circle(self.__cv_image,(x,y),5,(0,255,0),-1)

						if area > area_near_threshold:
							self.__external_sti_near.status = True
							self.__external_sti_far.status = False
							self.__approach_msg = ApproachUser(status=False, angle=0)
							self.__approach_pub.publish(self.__approach_msg)
						else:
							self.__external_sti_near.status = False
							self.__external_sti_far.status = True
							angle = self.calculate_angle(x, limit)
							self.__approach_msg = ApproachUser(status=True, angle=angle)
							self.__approach_pub.publish(self.__approach_msg)
					else:
						self.__external_sti_near.status = False
						self.__external_sti_far.status = False
				else:
					self.__external_sti_near.status = False
					self.__external_sti_far.status = False
			
			cv2.imshow("image window", self.__cv_image)
			cv2.waitKey(1)

		for key, value in self.__stimulus_status.items():
			if self.__external_sti_near.stimulus == key:
				if value != self.__external_sti_near.status:
					self.__external_sti_pub.publish(self.__external_sti_near)
					self.__external_sti_status_pub.publish(self.__external_sti_near)
					self.__stimulus_status[key] = self.__external_sti_near.status
			elif self.__external_sti_far.stimulus == key:
				if value != self.__external_sti_far.status:
					self.__external_sti_pub.publish(self.__external_sti_far)
					self.__external_sti_status_pub.publish(self.__external_sti_far)
					self.__stimulus_status[key] = self.__external_sti_far.status

	def calculate_angle(self, x, limit):

		angle = 0

		if limit[0] < x  < limit[1]:
			angle = 0
		elif x  < limit[0]:
			angle = 0.2
		elif x  > limit[1]:
			angle = -0.2

		return angle


	def run(self):
		"""
		Main while loop of the node
		"""

		while not rospy.is_shutdown():

			self.execute()

			rospy.sleep(0.2)

	def stop(self):
		"""
		Stop function
		"""

		cv2.destroyAllWindows()


######################################################
############# SUBSCRIBERS FUNCTIONS ##################
######################################################

	def __image_cb(self, msg):
		"""
		Function that gets the image from Miros left camera
		and transforms it to bgr9 format
		"""
		try:
			self.__cv_image = self.__bridge.imgmsg_to_cv2(msg, "bgr8")
		except CvBridgeError as e:
			print(e)

	def __behavior_cb(self, msg):
		"""
		Function that gets the current active behavior
		"""
		
		if msg.id in ["find_user", "approach_user", "play_game", "request_interaction"]:
			self.__find_user = True
		else:
			self.__find_user = False

	def __external_stimuli_cb(self, msg):

		if msg.stimulus == "user_presence":
			if msg.status:
				self.__user_presence = True
			else:
				self.__user_presence = False


######################################################
############### SERVICES FUNCTIONS ###################
######################################################

if __name__ == '__main__':
    try:
        # start the node
        rospy.init_node(pkg_name)

        # create and spin the node
        node = DetectColor()

        rospy.on_shutdown(node.stop)

        # spin the node
        node.run()
    except rospy.ROSInterruptException:
        pass