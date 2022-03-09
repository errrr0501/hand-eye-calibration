#!/usr/bin/env python

import rospy
import moveit_commander
from moveit_msgs.msg import MoveItErrorCodes
from visp_hand2eye_calibration.msg import TransformArray
from math import pi, radians
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
# from tf import transformations as tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from flexbe_core import EventState, Logger
import numpy as np
from flexbe_core.proxy import ProxyActionClient

# from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, Constraints, JointConstraint, MoveItErrorCodes
# from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction, JointTrajectoryControllerState, FollowJointTrajectoryResult
i = 0

class hand_eye_point(dict):

    def __init__(self):
        self['x']       = [] 
        self['y']       = []
        self['z']       = []
        self['qw']      = []
        self['qx']      = []
        self['qy']      = []
        self['qz']      = []

class GenerateHandEyePoint(EventState):
	"""
	Output a fixed pose to move.

	<= done									   points has been created.
	<= fail									   create points fail.

	"""


	def __init__(self, move_distance, group_name, reference_frame, cam_x, cam_y, cam_z , axis):
		'''
		Constructor
		'''
		super(GenerateHandEyePoint, self).__init__(outcomes=['done', 'failed'],
											input_keys=['camera_h_charuco'],
											output_keys=['hand_eye_points'])
		self.move_distance = float(move_distance)
		# self.times = int(times)
		self.points_x = []
		self.points_y = []
		self.points_z = []
		self.points_qw = []
		self.points_qx = []
		self.points_qy = []
		self.points_qz = []
		self.First_charuco_array = []
		self._group_name = group_name
		self._reference_frame = reference_frame
		self._move_group = moveit_commander.MoveGroupCommander(self._group_name)
		self._result = MoveItErrorCodes.FAILURE
		self._move_group.set_pose_reference_frame(self._reference_frame)
		self._end_effector_link = self._move_group.get_end_effector_link()
		self._current_pose = self._move_group.get_current_pose()
		self._origin_euler  = [0, 0, 0]
		self.base_rotation_x = 1
		self.base_rotation_y = 1
		self.base_rotation_z = 1
		self._axis = axis
		self.cam_axis_x = cam_x
		self.cam_axis_y = cam_y
		self.cam_axis_z = cam_z



	def stop(self):
		pass

	def execute(self, userdata):
		'''
		Execute this state
		'''

		if self.move_distance <= 11.0:
			print("robot_now")
			print(self._current_pose.pose.position.x)
			print(self._current_pose.pose.position.y)
			print(self._current_pose.pose.position.z)
			# print(self._current_pose.pose.orientation.x)
			# print(self._current_pose.pose.orientation.y)
			# print(self._current_pose.pose.orientation.z)
			# print(self._current_pose.pose.orientation.w)
			print("first aruco")
			print(userdata.camera_h_charuco.transforms[0].translation.x)
			print(userdata.camera_h_charuco.transforms[0].translation.y)
			print(userdata.camera_h_charuco.transforms[0].translation.z)
			if  self._axis == "xyz":
				self.points_x.append(self.base_rotation_x*self._current_pose.pose.position.x  + self.cam_axis_x*(-0.08 - userdata.camera_h_charuco.transforms[0].translation.x))
				self.points_y.append(self.base_rotation_y*self._current_pose.pose.position.y  + self.cam_axis_y*(-0.05 - userdata.camera_h_charuco.transforms[0].translation.y))
				self.points_z.append(self.base_rotation_z*self._current_pose.pose.position.z  + self.cam_axis_z*(0.30 - userdata.camera_h_charuco.transforms[0].translation.z))
				self.points_qx.append(self._current_pose.pose.orientation.x)
				self.points_qy.append(self._current_pose.pose.orientation.y)
				self.points_qz.append(self._current_pose.pose.orientation.z)
				self.points_qw.append(self._current_pose.pose.orientation.w)
				quaternion = quaternion_from_euler(np.radians(self._origin_euler[0]+15),np.radians(self._origin_euler[1]), np.radians(self._origin_euler[2]))  
				##############################################yaw_angle, ##########################pitch_angle, ######################roll_angle  

				for i in range(6):
					self.points_x.append(self.points_x[0]  + self.cam_axis_x*self.move_distance*0.01)
					self.points_y.append(self.points_y[0]  - self.cam_axis_y*self.move_distance*0.01 + self.cam_axis_y*0.035*i)
					self.points_z.append(self.points_z[0])
					self.points_qx.append(quaternion[0])
					self.points_qy.append(quaternion[1])
					self.points_qz.append(quaternion[2])
					self.points_qw.append(quaternion[3])
					i += 1
					
				quaternion = quaternion_from_euler(np.radians(self._origin_euler[0]),np.radians(self._origin_euler[1]-15), np.radians(self._origin_euler[2]))  
				##############################################yaw_angle, ##########################pitch_angle, ######################roll_angle  
				for i in range(6,12):
					self.points_x.append(self.points_x[6]  + 0.035*(i-6))
					self.points_y.append(self.points_y[6])
					self.points_z.append(self.points_z[0])
					self.points_qx.append(quaternion[0])
					self.points_qy.append(quaternion[1])
					self.points_qz.append(quaternion[2])
					self.points_qw.append(quaternion[3])
					i += 1
				quaternion = quaternion_from_euler(np.radians(self._origin_euler[0]-15),np.radians(self._origin_euler[1]), np.radians(self._origin_euler[2]))  
				##############################################yaw_angle, ##########################pitch_angle, ######################roll_angle  
				for i in range(12,18):
					self.points_x.append(self.points_x[12])
					self.points_y.append(self.points_y[6]  - self.cam_axis_y*0.035*(i-12))
					self.points_z.append(self.points_z[0])
					self.points_qx.append(quaternion[0])
					self.points_qy.append(quaternion[1])
					self.points_qz.append(quaternion[2])
					self.points_qw.append(quaternion[3])
					i += 1
				quaternion = quaternion_from_euler(np.radians(self._origin_euler[0]),np.radians(self._origin_euler[1]-15), np.radians(self._origin_euler[2]))  
				##############################################yaw_angle, ##########################pitch_angle, ######################roll_angle  
				for i in range(18,24):
					self.points_x.append(self.points_x[18]  - 0.035*(i-18))
					self.points_y.append(self.points_y[18])
					self.points_z.append(self.points_z[0])
					self.points_qx.append(quaternion[0])
					self.points_qy.append(quaternion[1])
					self.points_qz.append(quaternion[2])
					self.points_qw.append(quaternion[3])
					i += 1
			elif self._axis == "zxy":
				self.points_x.append(self._current_pose.pose.position.y  + self.cam_axis_x*(-0.08 - userdata.camera_h_charuco.transforms[0].translation.x))
				self.points_y.append(self._current_pose.pose.position.z  + self.cam_axis_y*(-0.05 - userdata.camera_h_charuco.transforms[0].translation.y))
				self.points_z.append(self._current_pose.pose.position.x  + self.cam_axis_z*(0.30 - userdata.camera_h_charuco.transforms[0].translation.z))
			
			# if self._axis == "xzy":
			# 	self.points_x.append(self._current_pose.pose.position.x  + self.axis_x*(0.08 + userdata.camera_h_charuco.transforms[0].translation.x))
			# 	self.points_y.append(self._current_pose.pose.position.z  + self.axis_y*(0.05 + userdata.camera_h_charuco.transforms[0].translation.y))
			# 	self.points_z.append(self._current_pose.pose.position.y  + self.axis_z*(-0.30 + userdata.camera_h_charuco.transforms[0].translation.z))
			# elif self._axis == "yxz":
			# 	self.points_x.append(self._current_pose.pose.position.y  + self.axis_x*(0.08 + userdata.camera_h_charuco.transforms[0].translation.x))
			# 	self.points_y.append(self._current_pose.pose.position.x  + self.axis_y*(0.05 + userdata.camera_h_charuco.transforms[0].translation.y))
			# 	self.points_z.append(self._current_pose.pose.position.z  + self.axis_z*(-0.30 + userdata.camera_h_charuco.transforms[0].translation.z))
			# elif self._axis == "yzx":
			# 	self.points_x.append(self._current_pose.pose.position.y  + self.axis_x*(0.08 + userdata.camera_h_charuco.transforms[0].translation.x))
			# 	self.points_y.append(self._current_pose.pose.position.z  + self.axis_y*(0.05 + userdata.camera_h_charuco.transforms[0].translation.y))
			# 	self.points_z.append(self._current_pose.pose.position.x  + self.axis_z*(-0.30 + userdata.camera_h_charuco.transforms[0].translation.z))
			# elif self._axis == "zxy":
			# 	self.points_x.append(self._current_pose.pose.position.y  + self.axis_x*(0.08 + userdata.camera_h_charuco.transforms[0].translation.x))
			# 	self.points_y.append(self._current_pose.pose.position.z  + self.axis_y*(0.05 + userdata.camera_h_charuco.transforms[0].translation.y))
			# 	self.points_z.append(self._current_pose.pose.position.x  + self.axis_z*(-0.30 + userdata.camera_h_charuco.transforms[0].translation.z))
			# elif self._axis == "zxy":
			# 	self.points_x.append(self._current_pose.pose.position.y  + self.axis_x*(0.08 + userdata.camera_h_charuco.transforms[0].translation.x))
			# 	self.points_y.append(self._current_pose.pose.position.z  + self.axis_y*(0.05 + userdata.camera_h_charuco.transforms[0].translation.y))
			# 	self.points_z.append(self._current_pose.pose.position.x  + self.axis_z*(-0.30 + userdata.camera_h_charuco.transforms[0].translation.z))
			
				self.points_qx.append(self._current_pose.pose.orientation.x)
				self.points_qy.append(self._current_pose.pose.orientation.y)
				self.points_qz.append(self._current_pose.pose.orientation.z)
				self.points_qw.append(self._current_pose.pose.orientation.w)
				quaternion = quaternion_from_euler(np.radians(self._origin_euler[0]+15),np.radians(self._origin_euler[1]), np.radians(self._origin_euler[2]))  
				##############################################yaw_angle, ##########################pitch_angle, ######################roll_angle  

				for i in range(6):
					self.points_x.append(self.points_x[0]  + self.cam_axis_y*self.move_distance*0.01)
					self.points_y.append(self.points_y[0]  - self.cam_axis_z*self.move_distance*0.01 + self.cam_axis_z*0.035*i)
					self.points_z.append(self.points_z[0])
					self.points_qx.append(quaternion[0])
					self.points_qy.append(quaternion[1])
					self.points_qz.append(quaternion[2])
					self.points_qw.append(quaternion[3])
					i += 1
					
				quaternion = quaternion_from_euler(np.radians(self._origin_euler[0]),np.radians(self._origin_euler[1]-15), np.radians(self._origin_euler[2]))  
				##############################################yaw_angle, ##########################pitch_angle, ######################roll_angle  
				for i in range(6,12):
					self.points_x.append(self.points_x[6]  + 0.035*(i-6))
					self.points_y.append(self.points_y[6])
					self.points_z.append(self.points_z[0])
					self.points_qx.append(quaternion[0])
					self.points_qy.append(quaternion[1])
					self.points_qz.append(quaternion[2])
					self.points_qw.append(quaternion[3])
					i += 1
				quaternion = quaternion_from_euler(np.radians(self._origin_euler[0]-15),np.radians(self._origin_euler[1]), np.radians(self._origin_euler[2]))  
				##############################################yaw_angle, ##########################pitch_angle, ######################roll_angle  
				for i in range(12,18):
					self.points_x.append(self.points_x[12])
					self.points_y.append(self.points_y[6]  - self.cam_axis_z*0.035*(i-12))
					self.points_z.append(self.points_z[0])
					self.points_qx.append(quaternion[0])
					self.points_qy.append(quaternion[1])
					self.points_qz.append(quaternion[2])
					self.points_qw.append(quaternion[3])
					i += 1
				quaternion = quaternion_from_euler(np.radians(self._origin_euler[0]),np.radians(self._origin_euler[1]-15), np.radians(self._origin_euler[2]))  
				##############################################yaw_angle, ##########################pitch_angle, ######################roll_angle  
				for i in range(18,24):
					self.points_x.append(self.points_x[18]  - 0.035*(i-18))
					self.points_y.append(self.points_y[18])
					self.points_z.append(self.points_z[0])
					self.points_qx.append(quaternion[0])
					self.points_qy.append(quaternion[1])
					self.points_qz.append(quaternion[2])
					self.points_qw.append(quaternion[3])
					i += 1


		# elif self.move_distance <= 20 and self.move_distance >= 10:

		# 	self.points_x  =  self.points_x.append(self._move_group.get_current_pose().pose.position.x  + 0.08 + userdata.camera_h_charuco.transforms.tanslation.x)
		# 	self.points_y  =  self.points_y.append(self._move_group.get_current_pose().pose.position.y  - 0.05 + userdata.camera_h_charuco.transforms.tanslation.y)
		# 	if userdata.camera_h_charuco.transforms.tanslation.z <= 0.70:
		# 		self.points_z  =  self.points_z.append(self._move_group.get_current_pose().pose.position.z  - 0.30 - userdata.camera_h_charuco.transforms.tanslation.z)
		# 	self.points_qw = self.points_qw.append(self._move_group.get_current_pose().pose.orientation.w)
		# 	self.points_qx = self.points_qx.append(self._move_group.get_current_pose().pose.orientation.x)
		# 	self.points_qy = self.points_qy.append(self._move_group.get_current_pose().pose.orientation.y)
		# 	self.points_qz = self.points_qz.append(self._move_group.get_current_pose().pose.orientation.z)

		# 	quaternion = quaternion_from_euler(np.radians(0),np.radians(90.), np.radians(0))   #yaw_angle, pitch_angle, roll_angle
			# print(self.points_x[0])
			userdata.hand_eye_points = hand_eye_point()
			userdata.hand_eye_points['x'] = self.points_x
			userdata.hand_eye_points['y'] = self.points_y
			userdata.hand_eye_points['z'] = self.points_z
			userdata.hand_eye_points['qw'] = self.points_qw
			userdata.hand_eye_points['qx'] = self.points_qx
			userdata.hand_eye_points['qy'] = self.points_qy
			userdata.hand_eye_points['qz'] = self.points_qz
			if self._axis == "zxy":
				userdata.hand_eye_points['x'] = self.points_z
				userdata.hand_eye_points['y'] = self.points_x
				userdata.hand_eye_points['z'] = self.points_y
				userdata.hand_eye_points['qw'] = self.points_qw
				userdata.hand_eye_points['qx'] = self.points_qx
				userdata.hand_eye_points['qy'] = self.points_qy
				userdata.hand_eye_points['qz'] = self.points_qz

			# print(userdata.hand_eye_points)
			# print("points")
			print(userdata.hand_eye_points['x'])
			print(userdata.hand_eye_points['y'])
			print(userdata.hand_eye_points['z'])
			# print(userdata.hand_eye_points['x'][1])
			# print(userdata.hand_eye_points['y'][1])
			# print(userdata.hand_eye_points['z'][1])
			return 'done'
		else:
			# userdata.hand_eye_points['x']  = userdata.hand_eye_points['x'].append(self._move_group.get_current_pose().pose.position.x)
			# userdata.hand_eye_points['y']  = userdata.hand_eye_points['y'].append(self._move_group.get_current_pose().pose.position.y)
			# userdata.hand_eye_points['z']  = userdata.hand_eye_points['z'].append(self._move_group.get_current_pose().pose.position.z)
			# userdata.hand_eye_points['qw'] = userdata.hand_eye_points['qw'].append(self._move_group.get_current_pose().pose.orientation.w)
			# userdata.hand_eye_points['qx'] = userdata.hand_eye_points['qx'].append(self._move_group.get_current_pose().pose.orientation.x)
			# userdata.hand_eye_points['qy'] = userdata.hand_eye_points['qy'].append(self._move_group.get_current_pose().pose.orientation.y)
			# userdata.hand_eye_points['qz'] = userdata.hand_eye_points['qz'].append(self._move_group.get_current_pose().pose.orientation.z)
			return 'failed'
			
		# if self._result =origindegree
	def on_enter(self, userdata):
		origindegree = euler_from_quaternion([self._current_pose.pose.orientation.x, self._current_pose.pose.orientation.y, self._current_pose.pose.orientation.z, self._current_pose.pose.orientation.w])
		self._origin_euler[0] = origindegree[0]/3.14*180.0
		self._origin_euler[1] = origindegree[1]/3.14*180.0
		self._origin_euler[2] = origindegree[2]/3.14*180.0
		# self.First_charuco_list = np.array(userdata.camera_h_charuco.transforms).shape
		# self._result = self._move_group.execute(userdata.joint_trajectory)
		# self.points = self.points.append(self._move_group.execute(userdata.camera_h_charuco))
		# self.points.hand_eye_points['x'] = userdata.camera_h_charuco.transforms.x
		pass

	def on_stop(self):
		pass

	def on_pause(self):
		pass

	def on_resume(self, userdata):
		self.on_enter(userdata)
