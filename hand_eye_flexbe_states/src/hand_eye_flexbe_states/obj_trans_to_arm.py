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
        self['x']       = [] 
        self['y']       = []
        self['z']       = []
        self['qx']      = []
        self['qy']      = []
        self['qz']      = []
        self['qw']      = []


class ObjTransToArmState(EventState):
	"""
	Output obj pose for arm.

	<= done									   points has been created.
	<= failed								   create points fail.

	"""


	def __init__(self, eye_in_hand_mode, base_link, tip_link):
		'''
		Constructor
		'''
		super(ObjTransToArmState, self).__init__(outcomes=['done', 'failed'],
											input_keys=['camera_h_charuco'],
											output_keys=['obj_position'])
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

		self.save_pwd = os.path.join(os.path.dirname(__file__), '..','..','..','charuco_detector/','config/','hand_eye_calibration/')

		self.trans2robot_service = '/eye_trans2base'
		self.trans2robot_client = ProxyServiceCaller({self.trans2robot_service: eye2base})
		self.pos2robot_service = '/eye2base'
		self.pos2robot_client = ProxyServiceCaller({self.pos2robot_service: eye2base})
		self.pix2robot_service = '/pix2base'
		self.pix2robot_client = ProxyServiceCaller({self.pix2robot_service: eye2base})


		self.quaternion = []
		self.excute_pos = []




	def on_start(self):
		pass

	def execute(self, userdata):
		'''
		Execute this state
		'''

		userdata.obj_position  = obj_info()
		userdata.obj_position['x'].append(self.excute_pos[0])
		userdata.obj_position['y'].append(self.excute_pos[1])
		userdata.obj_position['z'].append(self.excute_pos[2])
		userdata.obj_position['qx'].append(self.quaternion[0])
		userdata.obj_position['qy'].append(self.quaternion[1])
		userdata.obj_position['qz'].append(self.quaternion[2])
		userdata.obj_position['qw'].append(self.quaternion[3])


		print(userdata.obj_position)
		return 'done'

	def on_enter(self, userdata):




		origindegree = list(euler_from_quaternion([userdata.camera_h_charuco.transforms[0].rotation.x, 
													userdata.camera_h_charuco.transforms[0].rotation.y, 
													userdata.camera_h_charuco.transforms[0].rotation.z, 
													userdata.camera_h_charuco.transforms[0].rotation.w]))
		origindegree[0] = origindegree[0]/3.14*180.0
		origindegree[1] = origindegree[1]/3.14*180.0
		origindegree[2] = origindegree[2]/3.14*180.0
		print(origindegree)
		# quaternion = quaternion_from_euler(np.radians(180+origindegree[0]),
		# 									np.radians(origindegree[1]), 
		# 									np.radians(180+origindegree[2]))
		quaternion = quaternion_from_euler(np.radians(-180-origindegree[0]),
											np.radians(-origindegree[1]), 
											np.radians(-180-origindegree[2]))

		
		obj_pose = self.tf_listener.fromTranslationRotation((userdata.camera_h_charuco.transforms[0].translation.x,
															userdata.camera_h_charuco.transforms[0].translation.y,
															userdata.camera_h_charuco.transforms[0].translation.z)
															,(quaternion[0],
															quaternion[1],
															quaternion[2],
															quaternion[3]))

		# obj_pose = self.tf_listener.fromTranslationRotation((userdata.camera_h_charuco.transforms[0].translation.x,
		# 													userdata.camera_h_charuco.transforms[0].translation.y,
		# 													userdata.camera_h_charuco.transforms[0].translation.z)
		# 													,(userdata.camera_h_charuco.transforms[0].rotation.x,
		# 													userdata.camera_h_charuco.transforms[0].rotation.y,
		# 													userdata.camera_h_charuco.transforms[0].rotation.z,
		# 													userdata.camera_h_charuco.transforms[0].rotation.w))

		# quaternion = quaternion_from_euler(np.radians(-180),
		# 									np.radians(0), 
		# 									np.radians(0))  
		# rotation_matrix = tf.transformations.quaternion_matrix(quaternion)

		# quaternion2 = quaternion_from_euler(np.radians(0),
		# 									np.radians(0), 
		# 									np.radians(-180))  

		# rotation_matrix2 = tf.transformations.quaternion_matrix(quaternion2)

		# rotation_mat =  rotation_matrix * rotation_matrix2

		# # obj_pose = obj_pose * rotation_mat
		# obj_pose = rotation_mat * obj_pose 

		# obj_pose = [userdata.camera_h_charuco.transforms[0].translation.x, 
		# 			userdata.camera_h_charuco.transforms[0].translation.y,
		# 			userdata.camera_h_charuco.transforms[0].translation.z]
										
		# print(obj_pose)
		# self.vector(obj_pose)
		# obj_pose = np.float_(obj_pose).tolist()
		# for i in obj_pose:
		# 	obj_pose_list.append(i) 
		obj_pose = obj_pose.flatten().tolist()
		# print(obj_pose)


		# origindegree = list(euler_from_quaternion([userdata.camera_h_charuco.transforms[0].rotation.x,
		# 													userdata.camera_h_charuco.transforms[0].rotation.y,
		# 													userdata.camera_h_charuco.transforms[0].rotation.z,
		# 													userdata.camera_h_charuco.transforms[0].rotation.w]))
		# # origindegree = origindegree/3.14*180.0
		# origindegree[0] = origindegree[0]/3.14*180.0
		# origindegree[1] = origindegree[1]/3.14*180.0
		# origindegree[2] = origindegree[2]/3.14*180.0
		# print(origindegree)

		# quaternion = quaternion_from_euler(0,0,0)
		# print(quaternion)


		# obj_pose = [float(i) for i in obj_pose]
		# print(obj_pose)
		# assert len(obj_pose) == 16

		# print(np.mat(obj_pose).reshape(4, 4))


		# aruco_rotation = [userdata.camera_h_charuco.transforms[0].rotation.x,
		# 						userdata.camera_h_charuco.transforms[0].rotation.y,
		# 						userdata.camera_h_charuco.transforms[0].rotation.z,
		# 						userdata.camera_h_charuco.transforms[0].rotation.w]
		# aruco_rotation_degree = list(euler_from_quaternion(aruco_rotation))
		# aruco_rotation_degree[0] = aruco_rotation_degree[0]/3.14*180
		# aruco_rotation_degree[1] = aruco_rotation_degree[1]/3.14*180
		# aruco_rotation_degree[2] = aruco_rotation_degree[2]/3.14*180

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
			res = self.trans2robot_client.call(self.trans2robot_service, req)
		except rospy.ServiceException as e:
			rospy.logerr("Service call failed: %s" % e)
			return 'failed'

		# print(res)
		# print(res.pos[0])
		# print(res)
		self.excute_pos = res.pos
		# self.quaternion = quaternion_from_euler(res.euler[0], res.euler[1], res.euler[2])
		# self.quaternion = [res.quat[0], res.quat[1], res.quat[2], res.quat[3]]
		self.quaternion = [res.quat[0], res.quat[1], res.quat[2], res.quat[3]]

		origindegree = list(euler_from_quaternion(self.quaternion))
		# origindegree = origindegree/3.14*180.0
		origindegree[0] = origindegree[0]/3.14*180.0
		origindegree[1] = origindegree[1]/3.14*180.0
		origindegree[2] = origindegree[2]/3.14*180.0

		print(origindegree)


		
		pass

	def on_stop(self):
		pass

	def on_pause(self):
		pass

	def on_resume(self, userdata):
		# self.on_enter(userdata)
		pass
