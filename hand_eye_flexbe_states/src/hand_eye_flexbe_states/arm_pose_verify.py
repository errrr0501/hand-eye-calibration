#!/usr/bin/env python3

import rospy,os
import copy
import configparser
import numpy as np
import math as m
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



import moveit_commander
from moveit_msgs.msg import MoveItErrorCodes
from charuco_detector.srv import eye2base, eye2baseRequest, eye2baseResponse
# from charuco_detector import HandEyeTrans



class ar_mark_info(dict):

	def __init__(self):
		self['x']       = [] 
		self['y']       = []
		self['z']       = []
		self['qx']      = []
		self['qy']      = []
		self['qz']      = []
		self['qw']      = []


class ArmPoseVerify(EventState):
	"""
	Output obj pose for arm.

	<= done									   points has been created.
	<= failed								   create points fail.

	"""


	def __init__(self, eye_in_hand_mode, base_link, tip_link, group_name, reference_frame):
		'''
		Constructor
		'''
		super(ArmPoseVerify, self).__init__(outcomes=['done', 'failed'],
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

		self._group_name = group_name
		self._reference_frame = reference_frame
		self._move_group = moveit_commander.MoveGroupCommander(self._group_name)
		self._result = MoveItErrorCodes.FAILURE
		self._move_group.set_pose_reference_frame(self._reference_frame)
		self._end_effector_link = self._move_group.get_end_effector_link()
		self._move_group.set_end_effector_link(self._end_effector_link)
		self._move_group.set_max_acceleration_scaling_factor(0.1)
		self._move_group.set_max_velocity_scaling_factor(0.1)
		self._first_joints= self._move_group.get_current_joint_values()
		self._first_pose = self._move_group.get_current_pose()


		self.quaternion = []
		self.excute_pos = []




	def on_start(self):
		pass

	def execute(self, userdata):
		'''
		Execute this state
		'''

		userdata.obj_position  = ar_mark_info()
		userdata.obj_position['x'].append(0.00)
		userdata.obj_position['y'].append(0.00)
		userdata.obj_position['z'].append(0.00)
		userdata.obj_position['qx'].append(self.quaternion[0])
		userdata.obj_position['qy'].append(self.quaternion[1])
		userdata.obj_position['qz'].append(self.quaternion[2])
		userdata.obj_position['qw'].append(self.quaternion[3])


		print(userdata.obj_position)
		return 'done'

	def on_enter(self, userdata):



		origindegree = euler_from_quaternion([self._first_pose.pose.orientation.x, self._first_pose.pose.orientation.y, self._first_pose.pose.orientation.z, self._first_pose.pose.orientation.w])
		self._origin_euler[0] = origindegree[0]/3.14*180.0
		self._origin_euler[1] = origindegree[1]/3.14*180.0
		self._origin_euler[2] = origindegree[2]/3.14*180.0


		aruco_rotation = list(euler_from_quaternion([userdata.camera_h_charuco.transforms[0].rotation.x, 
													userdata.camera_h_charuco.transforms[0].rotation.y, 
													userdata.camera_h_charuco.transforms[0].rotation.z, 
													userdata.camera_h_charuco.transforms[0].rotation.w]))
		aruco_rotation[0] = aruco_rotation[0]/3.14*180.0
		aruco_rotation[1] = aruco_rotation[1]/3.14*180.0
		aruco_rotation[2] = aruco_rotation[2]/3.14*180.0



		# print(origindegree)
		
		# self.quaternion = quaternion_from_euler(np.radians(self._origin_euler[0]+(180+aruco_rotation[0])),
		# 									    np.radians(self._origin_euler[1]-(0+aruco_rotation[1])), 
		# 									    np.radians(self._origin_euler[2]-(90+aruco_rotation[2])))  


		self.quaternion = quaternion_from_euler(np.radians(self._origin_euler[0]+-1*(0+aruco_rotation[1])),
											    np.radians(self._origin_euler[1]+1*(180+aruco_rotation[0])), 
											    np.radians(self._origin_euler[2]+1*(90+aruco_rotation[2])))
		# self.quaternion = quaternion_from_euler(np.radians(self._origin_euler[0]+10),
		# 									    np.radians(self._origin_euler[1]+0), 
		# 									    np.radians(self._origin_euler[2]+0))  



		check_origindegree = list(euler_from_quaternion(self.quaternion))
		# origindegree = origindegree/3.14*180.0
		check_origindegree[0] = check_origindegree[0]/3.14*180.0
		check_origindegree[1] = check_origindegree[1]/3.14*180.0
		check_origindegree[2] = check_origindegree[2]/3.14*180.0

		print(self._origin_euler)

		print(check_origindegree)

		
		
		pass

	def on_stop(self):
		pass

	def on_pause(self):
		pass

	def on_resume(self, userdata):
		# self.on_enter(userdata)
		pass
