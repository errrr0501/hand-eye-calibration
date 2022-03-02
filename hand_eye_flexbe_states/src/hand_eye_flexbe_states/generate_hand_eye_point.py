#!/usr/bin/env python

import rospy
import moveit_commander
from moveit_msgs.msg import MoveItErrorCodes
from math import pi, radians
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf import transformations as tf

from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyActionClient

# from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, Constraints, JointConstraint, MoveItErrorCodes
# from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction, JointTrajectoryControllerState, FollowJointTrajectoryResult

class hand_eye_points(dict):

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


	def __init__(self, move_distance, times):
		'''
		Constructor
		'''
		super(GenerateHandEyePoint, self).__init__(outcomes=['done', 'failed'],
											input_keys=['camera_h_charuco'],
											output_keys=['hand_eye_points'])
		self.move_distance = move_distance
		self.times = times
		self.points = []



	def stop(self):
		pass

	def execute(self, userdata):
		'''
		Execute this state
		'''

		if self._result == MoveItErrorCodes.SUCCESS:
			return 'done'
		else:
			userdata.hand_eye_points['x']  = userdata.target_joints
			userdata.hand_eye_points['y']  = userdata.target_joints
			userdata.hand_eye_points['z']  = userdata.target_joints
			userdata.hand_eye_points['qw'] = userdata.target_joints
			userdata.hand_eye_points['qx'] = userdata.target_joints
			userdata.hand_eye_points['qy'] = userdata.target_joints
			userdata.hand_eye_points['qz'] = userdata.target_joints
			
		# if self._result == MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
		# 	userdata.joint_config = userdata.target_joints
		# 	return 'collision'
		# else:
		# 	return 'failed'

	def on_enter(self, userdata):
		# self._result = self._move_group.execute(userdata.joint_trajectory)
		self.points = self.points.append(self._move_group.execute(userdata.camera_h_charuco))

	def on_stop(self):
		pass

	def on_pause(self):
		pass

	def on_resume(self, userdata):
		self.on_enter(userdata)
