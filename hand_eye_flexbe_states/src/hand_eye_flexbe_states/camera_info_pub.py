#!/usr/bin/env python3
import rospy,rospkg
from flexbe_core import EventState
import time
import numpy
import cv2
from cv2 import aruco
import pickle
import glob,os
import configparser
from sensor_msgs.msg import CameraInfo



class CameraInfoPubState(EventState):
	"""
	Output a fixed pose to move.

	<= done									   Charuco pose has been received.
	<= go_compute							   Ready to compute the result.

	"""
	
	def __init__(self, file_name):
		"""Constructor"""
		super(CameraInfoPubState, self).__init__(outcomes=['done', 'failed'])

		self.camera_calibration_file = file_name
		

		self.save_pwd = os.path.join(os.path.dirname(__file__), '..','..','..','config/')


		# self.images = glob.glob(self.save_pwd + 'pic/camera-pic-of-charucoboard-*.jpg')

		self.pub_info = rospy.Publisher('/camera/color/camera_info', CameraInfo, queue_size=10)

		self.camera_msg = CameraInfo()
	# 	self.sub_info = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.check_camera_info)

	# def check_camera_info(self, data):
	# 	rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.D)
	# 	# print ('Application subscribes to %s and %s topics.' % ('/camera/color/camera_info'))
	# 	pass

	def on_start(self):
		pass
	def execute(self, userdata):


		try:

			rospy.init_node('camera_calibration_pub', anonymous=True)
			   # rate = rospy.Rate(10) # 10hz
			

				  
			config = configparser.ConfigParser()
			config.optionxform = str 
			#reference: http://docs.python.org/library/configparser.html


			config.read(self.save_pwd + self.camera_calibration_file)
			# internal_name = 'Internal_' + str(color_width) + '_' + str(color_high)
			D00 = float(config.get("Distortion", "k1"))
			D01 = float(config.get("Distortion", "k2"))
			D02 = float(config.get("Distortion", "t1"))
			D03 = float(config.get("Distortion", "t2"))
			D04 = float(config.get("Distortion", "k3"))

			K00 = float(config.get("Intrinsic", "0_0"))
			K01 = float(config.get("Intrinsic", "0_1"))
			K02 = float(config.get("Intrinsic", "0_2"))
			K10 = float(config.get("Intrinsic", "1_0"))
			K11 = float(config.get("Intrinsic", "1_1"))
			K12 = float(config.get("Intrinsic", "1_2"))
			K20 = float(config.get("Intrinsic", "2_0"))
			K21 = float(config.get("Intrinsic", "2_1"))
			K22 = float(config.get("Intrinsic", "2_2"))
			

			self.camera_msg.D = [D00, D01, D02, D03, D04]

			self.camera_msg.K = [K00, K01, K02, K10, K11, K12, K20, K21, K22]

		
			self.pub_info.publish(self.camera_msg)

			
		except rospy.ROSInterruptException:
			return "done"



		
			
	def on_enter(self, userdata):
		self.enter_time = rospy.Time.now()
		pass
