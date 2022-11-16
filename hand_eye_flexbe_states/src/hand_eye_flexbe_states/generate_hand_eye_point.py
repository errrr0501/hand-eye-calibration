#!/usr/bin/env python3

import rospy
import copy
import moveit_commander
from moveit_msgs.msg import MoveItErrorCodes
from visp_hand2eye_calibration.msg import TransformArray
from math import pi, radians
from std_msgs.msg import String
from geometry_msgs.msg import Transform
from moveit_commander.conversions import pose_to_list
import tf
# from tf import transformations
from visp_hand2eye_calibration.msg import TransformArray
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from flexbe_core import EventState, Logger
import numpy as np
from flexbe_core.proxy import ProxyActionClient

# from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, Constraints, JointConstraint, MoveItErrorCodes
# from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction, JointTrajectoryControllerState, FollowJointTrajectoryResult

# class hand_eye_point(dict):

#     def __init__(self):
#         self['x']       = [] 
#         self['y']       = []
#         self['z']       = []
#         self['qw']      = []
#         self['qx']      = []
#         self['qy']      = []
#         self['qz']      = []

class GenerateHandEyePoint(EventState):
	"""
	Output a fixed pose to move.

	<= done									   points has been created.
	<= fail									   create points fail.

	"""


	def __init__(self, eye_in_hand_mode, base_link, tip_link, move_distance, group_name, reference_frame, points_num, cam_x, cam_y, cam_z , axis):
		'''
		Constructor
		'''
		super(GenerateHandEyePoint, self).__init__(outcomes=['done', 'failed'],
											input_keys=['camera_h_charuco'],
											output_keys=['hand_eye_points'])
		self.move_distance = float(move_distance)
		print(eye_in_hand_mode)
		if eye_in_hand_mode:
			self.eye_in_hand_mode = 1
		else:
			self.eye_in_hand_mode = -1
		self.base_link = base_link
		self.tip_link = tip_link
		self.tf_listener = tf.TransformListener()
		# self.tool_h_base = TransformArray()
		self.waypoints = []
		self.plan_path = []
		
		self.execute_num = 0
		self.points_num = points_num
		self.loop_size = int(self.points_num/4)
		self.loop_num = 0
		self.First_charuco_array = []
		self._group_name = group_name
		self._reference_frame = reference_frame
		self._move_group = moveit_commander.MoveGroupCommander(self._group_name)
		self._result = MoveItErrorCodes.FAILURE
		self._move_group.set_pose_reference_frame(self._reference_frame)
		self._end_effector_link = self._move_group.get_end_effector_link()
		self._first_pose = self._move_group.get_current_pose()
		self._first_joints= self._move_group.get_current_joint_values()
		self.wpose = self._first_pose.pose
		self._origin_euler  = [0, 0, 0]
		# self.base_rotation_x = 1
		# self.base_rotation_y = 1
		# self.base_rotation_z = 1
		self._axis = axis
		self.cam_axis_x = cam_x
		self.cam_axis_y = cam_y
		self.cam_axis_z = cam_z
		self.pan_vector = np.arange(1.0, 5.0).reshape(4,1)
		self.pan_martix = np.arange(1.0, 5.0).reshape(4,1)



	def on_start(self):
		pass

	def execute(self, userdata):
		'''
		Execute this state
		'''
		return 'done'
		# else:
		# 	# userdata.hand_eye_points['x']  = userdata.hand_eye_points['x'].append(self._move_group.get_current_pose().pose.position.x)
		# 	# userdata.hand_eye_points['y']  = userdata.hand_eye_points['y'].append(self._move_group.get_current_pose().pose.position.y)
		# 	# userdata.hand_eye_points['z']  = userdata.hand_eye_points['z'].append(self._move_group.get_current_pose().pose.position.z)
		# 	# userdata.hand_eye_points['qw'] = userdata.hand_eye_points['qw'].append(self._move_group.get_current_pose().pose.orientation.w)
		# 	# userdata.hand_eye_points['qx'] = userdata.hand_eye_points['qx'].append(self._move_group.get_current_pose().pose.orientation.x)
		# 	# userdata.hand_eye_points['qy'] = userdata.hand_eye_points['qy'].append(self._move_group.get_current_pose().pose.orientation.y)
		# 	# userdata.hand_eye_points['qz'] = userdata.hand_eye_points['qz'].append(self._move_group.get_current_pose().pose.orientation.z)
		# 	return 'failed'
			
		# if self._result =origindegree
	def on_enter(self, userdata):

		origindegree = euler_from_quaternion([self._first_pose.pose.orientation.x, self._first_pose.pose.orientation.y, self._first_pose.pose.orientation.z, self._first_pose.pose.orientation.w])
		self._origin_euler[0] = origindegree[0]/3.14*180.0
		self._origin_euler[1] = origindegree[1]/3.14*180.0
		self._origin_euler[2] = origindegree[2]/3.14*180.0
		# self.First_charuco_list = np.array(userdata.camera_h_charuco.transforms).shape
		# self._result = self._move_group.execute(userdata.joint_trajectory)
		# self.points = self.points.append(self._move_group.execute(userdata.camera_h_charuco))
		# self.points.hand_eye_points['x'] = userdata.camera_h_charuco.transforms.x

		
		print(self.move_distance)
		print("=====================================================================")
		if self.move_distance <= 0.11:
			

			# baseltrans = tf.transformations.translation_matrix((-self._first_pose.pose.position.x, -self._first_pose.pose.position.y, self._first_pose.pose.position.z))
			# baserot = tf.transformations.quaternion_matrix((self._first_pose.pose.orientation.x, self._first_pose.pose.orientation.y, self._first_pose.pose.orientation.z, self._first_pose.pose.orientation.w))
			# current_base_h_tool = np.matmul(baseltrans, baserot)

			# print("robot_now")
			# print(self._first_pose.pose.position.x)
			# print(self._first_pose.pose.position.y)
			# print(self._first_pose.pose.position.z)
			# print(self._origin_euler)
			# print("first aruco")
			# print(userdata.camera_h_charuco.transforms[0].translation.x)
			# print(userdata.camera_h_charuco.transforms[0].translation.y)
			# print(userdata.camera_h_charuco.transforms[0].translation.z)
			aruco_rotation = [userdata.camera_h_charuco.transforms[0].rotation.x,
									userdata.camera_h_charuco.transforms[0].rotation.y,
									userdata.camera_h_charuco.transforms[0].rotation.z,
									userdata.camera_h_charuco.transforms[0].rotation.w]
			aruco_rotation_degree = list(euler_from_quaternion(aruco_rotation))
			aruco_rotation_degree[0] = aruco_rotation_degree[0]/3.14*180
			aruco_rotation_degree[1] = aruco_rotation_degree[1]/3.14*180
			aruco_rotation_degree[2] = aruco_rotation_degree[2]/3.14*180
			# print(aruco_rotation_degree)


			if  self._axis == "xyz":
				print("=====================================================================")
				self.wpose.position.x += self.cam_axis_x*(0.096-abs(userdata.camera_h_charuco.transforms[0].translation.y))
				self.wpose.position.y += self.cam_axis_y*(0.135-abs(userdata.camera_h_charuco.transforms[0].translation.x))
				self.wpose.position.z += self.cam_axis_z*(0.55-userdata.camera_h_charuco.transforms[0].translation.z)  # First move up (z)
				quaternion = quaternion_from_euler(np.radians(self._origin_euler[0]+(-180-aruco_rotation_degree[0])),
													np.radians(self._origin_euler[1]+(0-aruco_rotation_degree[1])), 
													np.radians(self._origin_euler[2]+(-90-aruco_rotation_degree[2])))  	
				self.wpose.orientation.x, self.wpose.orientation.y, self.wpose.orientation.z, self.wpose.orientation.w = (quaternion[0],
																										quaternion[1],
																										quaternion[2],
																										quaternion[3])
				self.waypoints.append(copy.deepcopy(self.wpose))

				(plan, fraction) = self._move_group.compute_cartesian_path(
                   																self.waypoints,   # waypoints to follow
                   																0.01,        # eef_step
                   																0.0)         # jump_threshold
				self.plan_path.append(plan)
				self.waypoints.clear()
				quaternion = quaternion_from_euler(np.radians(self._origin_euler[0]+(-180-aruco_rotation_degree[0])+15*self.eye_in_hand_mode),
													np.radians(self._origin_euler[1]+(0-aruco_rotation_degree[1])), 
													np.radians(self._origin_euler[2]+(-90-aruco_rotation_degree[2])))  	
				##############################################roll_angle, ##########################pitch_angle, ######################yaw_angle  
				self.wpose.position.x += self.cam_axis_x*self.move_distance
				self.wpose.position.y -= self.cam_axis_y*self.move_distance
				for self.loop_num in range(self.loop_size):
					self.wpose.position.y += self.cam_axis_y*(self.move_distance*2/self.loop_size)
					self.wpose.orientation.x, self.wpose.orientation.y, self.wpose.orientation.z, self.wpose.orientation.w = (quaternion[0],
																										quaternion[1],
																										quaternion[2],
																										quaternion[3])
					self.waypoints.append(copy.deepcopy(self.wpose))
					
					(plan, fraction) = self._move_group.compute_cartesian_path(
                   																self.waypoints,   # waypoints to follow
                   																0.01,        # eef_step
                   																0.0)         # jump_threshold
					self.plan_path.append(plan)
					self.waypoints.clear()
					self.loop_num += 1
				quaternion = quaternion_from_euler(np.radians(self._origin_euler[0]+(-180-aruco_rotation_degree[0])),
													np.radians(self._origin_euler[1]+(0-aruco_rotation_degree[1])+15), 
													np.radians(self._origin_euler[2]+(-90-aruco_rotation_degree[2])))  	
				##############################################roll_angle, ##########################pitch_angle, ######################yaw_angle  
				
				for self.loop_num in range(self.loop_size,self.loop_size*2):
					self.wpose.position.x -= self.cam_axis_x*(self.move_distance*2/self.loop_size)
					self.wpose.orientation.x, self.wpose.orientation.y, self.wpose.orientation.z, self.wpose.orientation.w = (quaternion[0],
																										quaternion[1],
																										quaternion[2],
																										quaternion[3])
					self.waypoints.append(copy.deepcopy(self.wpose))
					(plan, fraction) = self._move_group.compute_cartesian_path(
                   																self.waypoints,   # waypoints to follow
                   																0.01,        # eef_step
                   																0.0)         # jump_threshold
					self.plan_path.append(plan)
					self.waypoints.clear()
					self.loop_num += 1
				quaternion = quaternion_from_euler(np.radians(self._origin_euler[0]+(-180-aruco_rotation_degree[0])-15*self.eye_in_hand_mode),
													np.radians(self._origin_euler[1]+(0-aruco_rotation_degree[1])), 
													np.radians(self._origin_euler[2]+(-90-aruco_rotation_degree[2])))  	
				##############################################roll_angle, ##########################pitch_angle, ######################yaw_angle  
				for self.loop_num in range(self.loop_size*2,self.loop_size*3):
					self.wpose.position.y -= self.cam_axis_y*(self.move_distance*2/self.loop_size)
					self.wpose.orientation.x, self.wpose.orientation.y, self.wpose.orientation.z, self.wpose.orientation.w = (quaternion[0],
																										quaternion[1],
																										quaternion[2],
																										quaternion[3])
					self.waypoints.append(copy.deepcopy(self.wpose))
					(plan, fraction) = self._move_group.compute_cartesian_path(
                   																self.waypoints,   # waypoints to follow
                   																0.01,        # eef_step
                   																0.0)         # jump_threshold
					self.plan_path.append(plan)
					self.waypoints.clear()
					self.loop_num += 1
				quaternion = quaternion_from_euler(np.radians(self._origin_euler[0]+(-180-aruco_rotation_degree[0])),
													np.radians(self._origin_euler[1]+(0-aruco_rotation_degree[1])-15), 
													np.radians(self._origin_euler[2]+(-90-aruco_rotation_degree[2])))  	
				##############################################roll_angle, ##########################pitch_angle, ######################yaw_angle  
				for self.loop_num in range(self.loop_size*3,self.points_num):
					self.wpose.position.x += self.cam_axis_x*(self.move_distance*2/self.loop_size)
					self.wpose.orientation.x, self.wpose.orientation.y, self.wpose.orientation.z, self.wpose.orientation.w = (quaternion[0],
																										quaternion[1],
																										quaternion[2],
																										quaternion[3])
					self.waypoints.append(copy.deepcopy(self.wpose))
					(plan, fraction) = self._move_group.compute_cartesian_path(
                   																self.waypoints,   # waypoints to follow
                   																0.01,        # eef_step
                   																0.0)         # jump_threshold
					self.plan_path.append(plan)
					self.waypoints.clear()
					self.loop_num += 1

				
				self.plan_path.append(self.plan_path[0])
				userdata.hand_eye_points = self.plan_path
				# print(userdata.hand_eye_points)
		pass

	def on_stop(self):
		pass

	def on_pause(self):
		pass

	def on_resume(self, userdata):
		# self.on_enter(userdata)
		pass
