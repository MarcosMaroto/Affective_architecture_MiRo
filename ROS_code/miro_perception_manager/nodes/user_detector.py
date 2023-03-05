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

pkg_name = "user_detection"

import roslib
roslib.load_manifest("miro_perception_manager")
import rospy
import cv2
import numpy
import colorsys

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from motivational_dms.msg import ExternalStimulus, BehaviorInfo
from miro_perception_manager.srv import CorrectionAngle
from opencv_apps.msg import FaceArrayStamped

# define the lower and upper boundaries of the colors in the HSV color space
lower = {'blue_jeans':(90, 60, 0)}
upper = {'blue_jeans':(135, 130, 100)}

area_threshold = 2000

faces_limit = 3

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
		self.__user_presence_msg = ExternalStimulus(stimulus="user_detected",status=True)
		self.__face_detected = False
		self.__no_faces_counter = 0
		self.__faces_counter = 0
		self.__color_detected = False
		self.__user_presence_prev_value = ExternalStimulus()

		# Define variables of the node
		self.__create_msg_srv()

	def __create_msg_srv(self):
		"""
		Creation of publishers and subscribers
		"""

		# Publishers
		self.__external_sti_pub = rospy.Publisher("perception_manager/external_stimulus", ExternalStimulus, queue_size=5, latch=True)
		self.__ext_stimulus_status_pub = rospy.Publisher("perception_manager/external_stimulus_status", ExternalStimulus, queue_size=5, latch=True)
		
		# Subscribers
		self.__image_sub = rospy.Subscriber("camera/rgb/image_rect_color", Image, self.__image_cb)
		self.__faces_sub = rospy.Subscriber("face_detection/faces", FaceArrayStamped, self.__faces_cb)

	
	def execute(self):
		
		self.find_user_color()
		self.find_user_face()

		# Create msg for each case
		if self.__face_detected  or self.__color_detected:
			self.__user_presence_msg = ExternalStimulus(stimulus="user_presence", status=True, value=self.__user_presence_msg.value+0.5)
		else:
			self.__user_presence_msg = ExternalStimulus(stimulus="user_presence", status=True, value=self.__user_presence_msg.value-1)

		# Limit value to 0,100 range
		self.__user_presence_msg.value = max(min(self.__user_presence_msg.value, 100), 0)
		if self.__user_presence_msg.value < 2:
			self.__user_presence_msg.status = False
		
		# Publish value
		if self.__user_presence_prev_value.value != self.__user_presence_msg.value:
			self.__external_sti_pub.publish(self.__user_presence_msg)
			if self.__user_presence_prev_value.status != self.__user_presence_msg.status:
				self.__ext_stimulus_status_pub.publish(self.__user_presence_msg)
			self.__user_presence_prev_value = self.__user_presence_msg


	def find_user_color(self):

		if self.__cv_image is not None:
			
			blurred = cv2.GaussianBlur(self.__cv_image, (11, 11), 0)
			hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

			for key, value in upper.items():
				# construct a mask for the color from dictionary`1, then perform
				# a series of dilations and erosions to remove any small
				# blobs left in the mask
				kernel = numpy.ones((9,9),numpy.uint8)
				mask = cv2.inRange(hsv, lower[key], upper[key])
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
					x,y,w,h = cv2.boundingRect(c)

					if (w*h) > area_threshold:
						
						self.__color_detected = True

				else:
					self.__color_detected = False
			
			#cv2.imshow("image window", self.__cv_image)
			cv2.waitKey(1)

	def find_user_face(self):
		"""
		Function that checks if face of the user is detected
		"""

		if self.__faces_counter >= faces_limit:
			self.__face_detected = True
			self.__faces_counter = 0
		elif self.__no_faces_counter >= faces_limit:
			self.__face_detected = False
			self.__no_faces_counter = 0

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

	def __faces_cb(self, msg):
		"""
		Function that gets if the user is present
		"""
		if msg.faces:
			self.__no_faces_counter = 0
			self.__faces_counter += 1
		else:
			self.__no_faces_counter += 1
			self.__faces_counter = 0


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