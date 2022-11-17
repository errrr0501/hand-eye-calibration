#!/usr/bin/env python3

import rospy,os
import copy
import configparser
import numpy as np
from visp_hand2eye_calibration.msg import TransformArray
from math import pi, radians
from std_msgs.msg import String
from geometry_msgs.msg import Transform
import tf
# from tf import transformations
from visp_hand2eye_calibration.msg import TransformArray
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient
from flexbe_core.proxy import ProxyServiceCaller

from charuco_detector.srv import eye2base, eye2baseRequest, eye2baseResponse
# from charuco_detector import HandEyeTrans



class obj_info(dict):

    def __init__(self):
        self['trans']      = [] 
        self['pos]']       = []
        self['euler']      = []


class ObjTransToArm(EventState):
	"""
	Output obj pose for arm.

	<= done									   points has been created.
	<= failed								   create points fail.

	"""


	def __init__(self, eye_in_hand_mode, base_link, tip_link):
		'''
		Constructor
		'''
		super(ObjTransToArm, self).__init__(outcomes=['done', 'failed'],
											input_keys=['camera_h_charuco'],
											output_keys=['obj_position'])
		print(eye_in_hand_mode)
		if eye_in_hand_mode:
			self.eye_in_hand_mode = 1
		else:
			self.eye_in_hand_mode = -1
		self.base_link = base_link
		self.tip_link = tip_link
		self.tf_listener = tf.TransformListener()
		# self.tool_h_base = TransformArray()
		self.First_charuco_array = []
		self._origin_euler  = [0, 0, 0]

		self.save_pwd = os.path.join(os.path.dirname(__file__), '..','..','..','config/')

		self.pos2robot_service = '/eye_trans2base'
		self.pos2robot_client = ProxyServiceCaller({self.pos2robot_service: eye2base})
		self.pix2robot_service = '/pix2base'
		self.pix2robot_client = ProxyServiceCaller({self.pix2robot_service: eye2base})




	def on_start(self):
		pass

	def execute(self, userdata):
		'''
		Execute this state
		'''
		return 'done'

	def on_enter(self, userdata):


		obj_pose = self.tf_listener.fromTranslationRotation((userdata.camera_h_charuco.transforms[0].translation.x,
															userdata.camera_h_charuco.transforms[0].translation.y,
															userdata.camera_h_charuco.transforms[0].translation.z)
															,(userdata.camera_h_charuco.transforms[0].rotation.x,
															userdata.camera_h_charuco.transforms[0].rotation.y,
															userdata.camera_h_charuco.transforms[0].rotation.z,
															userdata.camera_h_charuco.transforms[0].rotation.w))
		# obj_pose_list = []
		print(obj_pose)
		# self.vector(obj_pose)
		# obj_pose = np.float_(obj_pose).tolist()
		# for i in obj_pose:
		# 	obj_pose_list.append(i) 
		obj_pose = obj_pose.flatten().tolist()
		# obj_pose = [float(i) for i in obj_pose]
		print(obj_pose)
		# assert len(obj_pose) == 16

		# print(np.mat(obj_pose).reshape(4, 4))


		aruco_rotation = [userdata.camera_h_charuco.transforms[0].rotation.x,
								userdata.camera_h_charuco.transforms[0].rotation.y,
								userdata.camera_h_charuco.transforms[0].rotation.z,
								userdata.camera_h_charuco.transforms[0].rotation.w]
		aruco_rotation_degree = list(euler_from_quaternion(aruco_rotation))
		aruco_rotation_degree[0] = aruco_rotation_degree[0]/3.14*180
		aruco_rotation_degree[1] = aruco_rotation_degree[1]/3.14*180
		aruco_rotation_degree[2] = aruco_rotation_degree[2]/3.14*180

		config = configparser.ConfigParser()
		config.optionxform = str 

		# config.read(self.save_pwd + self.camera_calibration_file)
		# x  = float(config.get("hand_eye_calibration", "x"))
		# y  = float(config.get("hand_eye_calibration", "y"))
		# z  = float(config.get("hand_eye_calibration", "z"))
		# qw = float(config.get("hand_eye_calibration", "qw"))
		# qx = float(config.get("hand_eye_calibration", "qx"))
		# qy = float(config.get("hand_eye_calibration", "qy"))
		# qz = float(config.get("hand_eye_calibration", "qz"))

		# self.First_charuco_list = np.array(userdata.camera_h_charuco.transforms).shape
		# self._result = self._move_group.execute(userdata.joint_trajectory)
		# self.points = self.points.append(self._move_group.execute(userdata.camera_h_charuco))
		# self.points.hand_eye_points['x'] = userdata.camera_h_charuco.transforms.x
		req = eye2baseRequest()
		req.ini_pose = obj_pose

		try:
			res = self.pos2robot_client.call(self.pos2robot_service, req)
		except rospy.ServiceException as e:
			rospy.logerr("Service call failed: %s" % e)
			return 'failed'
		userdata.obj_position = obj_info()
		userdata.obj_position['trans'] = res.trans
		userdata.obj_position['pos]']  = res.pos
		userdata.obj_position['euler'] = res.euler




		print(userdata.obj_position)
		
		pass

	def on_stop(self):
		pass

	def on_pause(self):
		pass

	def on_resume(self, userdata):
		# self.on_enter(userdata)
		pass
