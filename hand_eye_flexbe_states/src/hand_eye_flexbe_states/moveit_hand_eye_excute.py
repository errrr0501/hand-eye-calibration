#!/usr/bin/env python3

import rospy
import numpy as np
import moveit_commander
from moveit_msgs.msg import MoveItErrorCodes
from math import pi, radians
from std_msgs.msg import String
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list
from tf import transformations as tf

from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyActionClient

from tf.transformations import quaternion_from_euler, euler_from_quaternion


# from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, Constraints, JointConstraint, MoveItErrorCodes
# from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction, JointTrajectoryControllerState, FollowJointTrajectoryResult

'''
Created on 15.06.2015

@author: Philipp Schillinger
'''

class MoveitHandEyeExecuteState(EventState):
	'''
	Move robot by planned trajectory.

	-- group_name         string      move group name

	># joint_trajectory             JointTrajectory  planned trajectory

	<= done 						Robot move done.
	<= failed 						Robot move failed.
	<= collision 				    Robot during collision.
	'''


	def __init__(self, group_name,reference_frame, points_num):
		'''
		Constructor
		'''
		super(MoveitHandEyeExecuteState, self).__init__(outcomes=['received','done', 'finish_correction', 'collision'],
											input_keys=['hand_eye_points'],
											output_keys=['result_compute'])
		# group_name = ""
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
		self.points_num  = points_num
		self._execute_times = 0
		self.plan, self.fraction =0,0
		self.centralized = False

	def on_start(self):
		# self.points_num = np.size(userdata.hand_eye_points)
		# print(np.size(userdata.hand_eye_points))
		pass

	def execute(self, userdata):
		'''
		Execute this state
		'''
		# print("")
		# print("==================================================================")
		# print(self._result)
		# print("==================================================================")
		print(np.size(userdata.hand_eye_points))
		print(self._execute_times)
		self.plan = userdata.hand_eye_points[self._execute_times]
		# input()
		# excute planing path
		self._result = self._move_group.execute(self.plan, wait=True)
		# self._move_group.stop()
		# self._move_group.clear_pose_targets()
		userdata.result_compute = self._execute_times >= self.points_num
		# print("==================================================================")
		# print(userdata.result_compute)
		# print("==================================================================")
		# print(self._execute_times)
		# print(self.points_num)


		current_pose = self._move_group.get_current_pose()

		# current_degree = list(euler_from_quaternion([current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z,  current_pose.pose.orientation.w]))
		# current_degree[0] = current_degree[0]/3.14*180
		# current_degree[1] = current_degree[1]/3.14*180
		# current_degree[2] = current_degree[2]/3.14*180
		print("current pose",current_pose)
		# print("current_degree",current_degree)

		if userdata.result_compute:
			return 'done'

		if self._result == MoveItErrorCodes.SUCCESS:
			self._execute_times += 1
			# print(self.execute_num)
			if not self.centralized:
				self._first_joints= self._move_group.get_current_joint_values()
				self.centralized = True
				self._execute_times = 0
				return 'finish_correction'
			return 'received'
			
		elif self._result == MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
			return 'collision'
		# else:
		# 	return 'failed'

	def on_enter(self, userdata):
		# back to first pose
		# elif self._execute_times == 1:
		# self._first_joints= self._move_group.get_current_joint_values()
		self._result = self._move_group.go(self._first_joints, wait=True)
		self._move_group.stop()
		self._move_group.clear_pose_targets()
		# self._execute_times += 1
		# self.points_num = np.size(userdata.hand_eye_points)
		# print(np.size(userdata.hand_eye_points))
		pass

	def on_stop(self):
		pass

	def on_pause(self):
		pass

	def on_resume(self, userdata):
		# self.on_enter(userdata)
		pass
