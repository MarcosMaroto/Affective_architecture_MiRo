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

pkg_name = "color_detection"

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

# define the lower and upper boundaries of the colors in the HSV color space
lower = {'red':(0, 150, 175), 'green':(58, 160, 80), 'blue':(101, 220, 20), 'yellow':(12, 75, 180)} #assigned new item lower['blue'] = (93, 10, 0)
upper = {'red':(12, 210, 255), 'green':(80, 240, 195), 'blue':(130, 255, 180), 'yellow':(35,130,255)}

# define the lower and upper boundaries of the colors in the rgb color space
#lower = {'red':(100, 0, 0), 'green':(54, 122, 129), 'blue':(97, 100, 117), 'yellow':(10, 100, 150)} #assigned new item lower['blue'] = (93, 10, 0)
#upper = {'red':(255,0,0), 'green':(78,255,255), 'blue':(117,255,255), 'yellow':(40,255,255)}
 
# define standard colors for circle around the object
colors = {'red':(0,0,255), 'green':(0,255,0), 'blue':(255,0,0), 'yellow':(0, 255, 217)}

# Detected radius
MAX_RAW = 50
MIN_RAW = 2
MAX_SIGNAL = 100
MIN_SIGNAL = 0

# Number of consecutive detections to consider it True
threshold = 10

# Limit for considering the set value
set_value_threshold = 1

# Balls factor to be add to stimulus per ball detected
balls_factor = 5

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
		for key in colors.keys():
			self.__color_detected_counter[key] = 0
		self.__external_sti_msg = ExternalStimulus(stimulus="balls", status=False)
		self.__prev_value = 0
		self.__playing = False
		self.__prev_status = None

		# Define variables of the node
		self.__create_msg_srv()

	def __create_msg_srv(self):
		"""
		Creation of publishers and subscribers
		"""

		# Publishers
		self.__color_pub = rospy.Publisher("perception_manager/color", String, queue_size=1, latch=False)
		self.__external_sti_pub = rospy.Publisher("perception_manager/external_stimulus", ExternalStimulus, queue_size=5, latch=True)
		self.__external_sti_status_pub = rospy.Publisher("perception_manager/external_stimulus_status", ExternalStimulus, queue_size=5, latch=True)
		
		# Subscribers
		self.__image_sub = rospy.Subscriber("miro/rob01/platform/caml", Image, self.__imagel_cb)
		#self.__image_sub = rospy.Subscriber("camera/rgb/image_rect_color", Image, self.__image_cb)
		self.__behavior_info_sub = rospy.Subscriber("motivational_dms/behavior_information", BehaviorInfo, self.__behavior_cb)


	def execute(self):

		# Reset stimulation signal value
		self.__external_sti_msg.value = 0
		# If an image is available
		if self.__cv_image is not None:
			
			# Transform image to HSV format
			blurred = cv2.GaussianBlur(self.__cv_image, (11, 11), 0)
			hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

			max_value = 0
			balls_detected = 0

			# Find for objects of each color range
			for key, value in upper.items():
				# construct a mask for the color from dictionary`1, then perform
				# a series of dilations and erosions to remove any small
				# blobs left in the mask
				kernel = numpy.ones((9,9),numpy.uint8)
				mask = cv2.inRange(hsv, lower[key], upper[key])
				mask = cv2.dilate(mask,kernel,iterations = 5)
				mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
				mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
				       
				# find contours in the mask and initialize the current
				cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
				#center = None
				
				cv2.imshow(key, mask)
			       
				# only proceed if at least one contour was found
				if len(cnts) > 0:
					# find the largest contour in the mask, then use
					# it to compute the minimum enclosing circle and
					# centroid
					c = max(cnts, key=cv2.contourArea)
					((x, y), radius) = cv2.minEnclosingCircle(c)
					M = cv2.moments(c)
					center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

					# only proceed if the radius meets a minimum size. Correct this value for your obect's size
					if MAX_RAW > radius > MIN_RAW:
						# draw the circle and centroid on the frame,
						# then update the list of tracked points
						#cv2.circle(image, (int(x), int(y)), int(radius), colors[key], 2)
						#cv2.putText(image, key, (int(x-radius),int(y-radius)), cv2.FONT_HERSHEY_SIMPLEX, 0.6,colors[key],2)
						
						self.__color_detected_counter[key] += 1
						# Publish value of the detected color when playing
						if self.__playing:
							if self.__color_detected_counter > threshold:
								self.__color_pub.publish(key)

						# Generate stimulation signal based on the maximum radius of the balls detected inside range [0,100]
						current_stimulus_value = int(MAX_SIGNAL + ((radius - MAX_RAW) * (MAX_SIGNAL - MIN_SIGNAL) / (MAX_RAW - MIN_RAW))) 
						balls_detected += 1
						# Find maximum
						if current_stimulus_value > max_value:
							max_value = current_stimulus_value
				else:
					self.__color_detected_counter[key] = 0

			max_value += balls_detected * balls_factor

			if self.__prev_value > (max_value + set_value_threshold):
				self.__external_sti_msg.value = self.__prev_value - 1
			if self.__prev_value < (max_value - set_value_threshold):
				self.__external_sti_msg.value = self.__prev_value + 1
			else:
				self.__external_sti_msg.value = max_value

			self.__external_sti_msg.value = min(max(self.__external_sti_msg.value, MIN_SIGNAL), MAX_SIGNAL)

			# If value of the signal has changed, publish it
			if self.__external_sti_msg.value != self.__prev_value:
				if self.__external_sti_msg.value < 5:
					self.__external_sti_msg.status = False
				else:
					self.__external_sti_msg.status = True
				self.__external_sti_pub.publish(self.__external_sti_msg)
				self.__prev_value = self.__external_sti_msg.value
				
				if self.__prev_status != self.__external_sti_msg.status:
					self.__prev_status = self.__external_sti_msg.status
					self.__external_sti_status_pub.publish(self.__external_sti_msg)



			#cv2.imshow("general", self.__cv_image)
			cv2.waitKey(1)

	def run(self):
		"""
		Main while loop of the node
		"""

		while not rospy.is_shutdown():

			self.execute()

			rospy.sleep(0.5)

	def stop(self):
		"""
		Stop function
		"""

		cv2.destroyAllWindows()


######################################################
############# SUBSCRIBERS FUNCTIONS ##################
######################################################

	def __imagel_cb(self, msg):
		"""
		Function that gets the image from Miros Left camera and store it in bgr format
		"""

		try:
			self.__cv_image = self.__bridge.imgmsg_to_cv2(msg, "bgr8")
		except CvBridgeError as e:
			print(e)

	def __behavior_cb(self, msg):
		"""
		Get the current active bahavior
		"""

		if msg.id == "play_game":
			self.__playing = True
		else:
			self.__playing = False

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