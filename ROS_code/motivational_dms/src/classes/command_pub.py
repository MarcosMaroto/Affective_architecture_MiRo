#!/usr/bin/python
# +*+ coding: utf+8 +*+

__author__ = "Marcos Maroto Gómez"
__copyright__ = "Social Robots Group. Robotics Lab. University Carlos III of Madrid"
__credits__ = ["Marcos Maroto Gómez"]
__license__ = "LEUC3M v1.0"
__version__ = "0.0.0"
__maintainer__ = "Marcos Maroto Gómez"
__email__ = "marmarot@ing.uc3m.es"
__status__ = "Development"



import rospy 
from miro_msgs.msg import platform_control

rospy.init_node("tests")

command_pub = rospy.Publisher("miro/rob01/platform/control", platform_control, latch=True, queue_size=1)

msg = platform_control()


for i in range(0,3):
	msg.body_config = [0.0, 0.10, -0.90, 0.1]
	msg.body_config_speed = [0.0, -1.0, -1.0, -1.0]
	msg.msg_flags = 1
	msg.tail = 0.0
	msg.ear_rotate = [0.0,0.0]
	msg.eyelid_closure = 0.0
	msg.blink_time = 0
	msg.lights_max_drive = 255
	msg.lights_dphase = 64
	msg.lights_phase = 128
	msg.lights_amp = 128
	msg.lights_off = 0
	msg.lights_rgb = [200,200,0]
	msg.lights_raw = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
	msg.sound_index_P1 = 0
	msg.sound_index_P2 = 0

	msg.body_vel.angular.z = -1

	command_pub.publish(msg)

	msg.body_config = [0.0, 0.80, 0.9, -0.1]
	msg.body_config_speed = [0.0, -1.0, -1.0, -1.0]
	msg.msg_flags = 1
	msg.tail = 0.5
	msg.ear_rotate = [0.5,0.5]
	msg.eyelid_closure = 0.0
	msg.blink_time = 0
	msg.lights_max_drive = 255
	msg.lights_dphase = 64
	msg.lights_phase = 128
	msg.lights_amp = 128
	msg.lights_off = 0
	msg.lights_rgb = [200,200,0]
	msg.lights_raw = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
	msg.sound_index_P1 = 0
	msg.sound_index_P2 = 0

	msg.body_vel.angular.z = 1

	command_pub.publish(msg)

	rospy.sleep(1)
