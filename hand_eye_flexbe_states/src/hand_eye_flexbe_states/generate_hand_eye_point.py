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
		# self.times = int(times)
		# self.points_x = []
		# self.points_y = []
		# self.points_z = []
		# self.points_qw = []
		# self.points_qx = []
		# self.points_qy = []
		# self.points_qz = []
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



	def start(self):
		pass

	def execute(self, userdata):
		'''
		Execute this state
		'''
		# self._result = self._move_group.go(self._first_joints, wait=True)
		# self._move_group.stop()
		# self._move_group.clear_pose_targets()
		# try:
		# 	(tool_trans_base, tool_rot_base) = self.tf_listener.lookupTransform(self.tip_link, self.base_link, rospy.Time(0))
		# except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		# 	rospy.logwarn('lookupTransform for robot failed!, ' + self.base_link + ', ' + self.tip_link)
		# 	return
		# # print(tool_trans_base)
		# # print(tool_rot_base)
		# tooltrans = tf.transformations.translation_matrix(tool_trans_base)
		# toolrot = tf.transformations.quaternion_matrix(tool_rot_base)
		# tool_h_base = np.matmul(tooltrans, toolrot)
		# print(tooltrans)
		# print(toolrot)
		# print(tool_h_base)
		# trans = Transform()
		# trans.translation.x = tool_trans_base[0]
		# trans.translation.y = tool_trans_base[1]
		# trans.translation.z = tool_trans_base[2]
		# trans.rotation.x = tool_rot_base[0]
		# trans.rotation.y = tool_rot_base[1]
		# trans.rotation.z = tool_rot_base[2]
		# trans.rotation.w = tool_rot_base[3]
		# self.tool_h_base.transforms.append(trans)
		

		# give next move a ref
		# pos_x = self.Grasp_point[0] 
		# pos_y = self.Grasp_point[1]
		# pos_z = self.Grasp_point[2]

			# if  self._axis == "xyz":
			# 	quaternion = quaternion_from_euler(np.radians(self._origin_euler[0]+(45-aruco_rotation_degree[0])),
			# 										np.radians(self._origin_euler[1]+(10-aruco_rotation_degree[1])), 
			# 										np.radians(self._origin_euler[2]))  	
			# 	# self.points_x.append(self.base_rotation_x*self._first_pose.pose.position.x  + self.cam_axis_x*(- userdata.camera_h_charuco.transforms[0].translation.x))
			# 	# self.points_y.append(self.base_rotation_y*self._first_pose.pose.position.y  + self.cam_axis_y*(- userdata.camera_h_charuco.transforms[0].translation.y))
			# 	# self.points_x.append(self._first_pose.pose.position.x  + self.cam_axis_x*(userdata.camera_h_charuco.transforms[0].translation.x))
			# 	self.points_x.append(self._first_pose.pose.position.x)
			# 	# self.points_y.append(self._first_pose.pose.position.y  + self.cam_axis_y*(-userdata.camera_h_charuco.transforms[0].translation.y/2))
			# 	self.points_y.append(self._first_pose.pose.position.y)
			# 	# self.points_x.append(self._first_pose.pose.position.x )
			# 	# self.points_y.append(self._first_pose.pose.position.y )
			# 	self.points_z.append(self._first_pose.pose.position.z  + self.cam_axis_z*(0.45-userdata.camera_h_charuco.transforms[0].translation.z))
			# 	# self.points_z.append(self._first_pose.pose.position.z)
			# 	# self.points_qx.append(self._first_pose.pose.orientation.x)
			# 	# self.points_qy.append(self._first_pose.pose.orientation.y)
			# 	# self.points_qz.append(self._first_pose.pose.orientation.z)
			# 	# self.points_qw.append(self._first_pose.pose.orientation.w)
			# 	self.points_qx.append(quaternion[0])
			# 	self.points_qy.append(quaternion[1])
			# 	self.points_qz.append(quaternion[2])
			# 	self.points_qw.append(quaternion[3])
			# 	quaternion = quaternion_from_euler(np.radians(self._origin_euler[0]+(45-aruco_rotation_degree[0])+15*self.eye_in_hand_mode),
			# 										np.radians(self._origin_euler[1]+(10-aruco_rotation_degree[1])), 
			# 										np.radians(self._origin_euler[2]))  	
			# 	##############################################roll_angle, ##########################pitch_angle, ######################yaw_angle  

			# 	for i in range(6):
			# 		self.points_x.append(self.points_x[0]  + self.cam_axis_x*self.move_distance*0.01)
			# 		self.points_y.append(self.points_y[0]  - self.cam_axis_y*self.move_distance*0.01 + self.cam_axis_y*0.035*i)
			# 		self.points_z.append(self.points_z[0])
			# 		self.points_qx.append(quaternion[0])
			# 		self.points_qy.append(quaternion[1])
			# 		self.points_qz.append(quaternion[2])
			# 		self.points_qw.append(quaternion[3])
			# 		i += 1
					
			# 	quaternion = quaternion_from_euler(np.radians(self._origin_euler[0]+(45-aruco_rotation_degree[0])),
			# 										np.radians(self._origin_euler[1]+(10-aruco_rotation_degree[1])), 
			# 										np.radians(self._origin_euler[2]))  	
			# 	##############################################roll_angle, ##########################pitch_angle, ######################yaw_angle  
			# 	for i in range(6,12):
			# 		self.points_x.append(self.points_x[6]  - 0.035*(i-6))
			# 		self.points_y.append(self.points_y[6])
			# 		self.points_z.append(self.points_z[0])
			# 		self.points_qx.append(quaternion[0])
			# 		self.points_qy.append(quaternion[1])
			# 		self.points_qz.append(quaternion[2])
			# 		self.points_qw.append(quaternion[3])
			# 		i += 1
			# 	quaternion = quaternion_from_euler(np.radians(self._origin_euler[0]+(45-aruco_rotation_degree[0])-15*self.eye_in_hand_mode),
			# 										np.radians(self._origin_euler[1]+(10-aruco_rotation_degree[1])), 
			# 										np.radians(self._origin_euler[2]))  	
			# 	##############################################roll_angle, ##########################pitch_angle, ######################yaw_angle  
			# 	for i in range(12,18):
			# 		self.points_x.append(self.points_x[12])
			# 		self.points_y.append(self.points_y[6]  - self.cam_axis_y*0.035*(i-12))
			# 		self.points_z.append(self.points_z[0])
			# 		self.points_qx.append(quaternion[0])
			# 		self.points_qy.append(quaternion[1])
			# 		self.points_qz.append(quaternion[2])
			# 		self.points_qw.append(quaternion[3])
			# 		i += 1
			# 	quaternion = quaternion_from_euler(np.radians(self._origin_euler[0]+(45-aruco_rotation_degree[0])),
			# 										np.radians(self._origin_euler[1]+(10-aruco_rotation_degree[1])), 
			# 										np.radians(self._origin_euler[2]))  	
			# 	##############################################roll_angle, ##########################pitch_angle, ######################yaw_angle  
			# 	for i in range(18,24):
			# 		self.points_x.append(self.points_x[18]  + 0.035*(i-18))
			# 		self.points_y.append(self.points_y[18])
			# 		self.points_z.append(self.points_z[0])
			# 		self.points_qx.append(quaternion[0])
			# 		self.points_qy.append(quaternion[1])
			# 		self.points_qz.append(quaternion[2])
			# 		self.points_qw.append(quaternion[3])
			# 		i += 1
			# elif self._axis == "zxy":
			# 	self.points_x.append(self._first_pose.pose.position.y  + self.cam_axis_x*(-0.08 - userdata.camera_h_charuco.transforms[0].translation.x))
			# 	self.points_y.append(self._first_pose.pose.position.z  + self.cam_axis_y*(-0.05 - userdata.camera_h_charuco.transforms[0].translation.y))
			# 	self.points_z.append(self._first_pose.pose.position.x  + self.cam_axis_z*(0.30 - userdata.camera_h_charuco.transforms[0].translation.z))
			
			# # if self._axis == "xzy":
			# # 	self.points_x.append(self._first_pose.pose.position.x  + self.axis_x*(0.08 + userdata.camera_h_charuco.transforms[0].translation.x))
			# # 	self.points_y.append(self._first_pose.pose.position.z  + self.axis_y*(0.05 + userdata.camera_h_charuco.transforms[0].translation.y))
			# # 	self.points_z.append(self._first_pose.pose.position.y  + self.axis_z*(-0.30 + userdata.camera_h_charuco.transforms[0].translation.z))
			# # elif self._axis == "yxz":
			# # 	self.points_x.append(self._first_pose.pose.position.y  + self.axis_x*(0.08 + userdata.camera_h_charuco.transforms[0].translation.x))
			# # 	self.points_y.append(self._first_pose.pose.position.x  + self.axis_y*(0.05 + userdata.camera_h_charuco.transforms[0].translation.y))
			# # 	self.points_z.append(self._first_pose.pose.position.z  + self.axis_z*(-0.30 + userdata.camera_h_charuco.transforms[0].translation.z))
			# # elif self._axis == "yzx":
			# # 	self.points_x.append(self._first_pose.pose.position.y  + self.axis_x*(0.08 + userdata.camera_h_charuco.transforms[0].translation.x))
			# # 	self.points_y.append(self._first_pose.pose.position.z  + self.axis_y*(0.05 + userdata.camera_h_charuco.transforms[0].translation.y))
			# # 	self.points_z.append(self._first_pose.pose.position.x  + self.axis_z*(-0.30 + userdata.camera_h_charuco.transforms[0].translation.z))
			# # elif self._axis == "zxy":
			# # 	self.points_x.append(self._first_pose.pose.position.y  + self.axis_x*(0.08 + userdata.camera_h_charuco.transforms[0].translation.x))
			# # 	self.points_y.append(self._first_pose.pose.position.z  + self.axis_y*(0.05 + userdata.camera_h_charuco.transforms[0].translation.y))
			# # 	self.points_z.append(self._first_pose.pose.position.x  + self.axis_z*(-0.30 + userdata.camera_h_charuco.transforms[0].translation.z))
			# # elif self._axis == "zxy":
			# # 	self.points_x.append(self._first_pose.pose.position.y  + self.axis_x*(0.08 + userdata.camera_h_charuco.transforms[0].translation.x))
			# # 	self.points_y.append(self._first_pose.pose.position.z  + self.axis_y*(0.05 + userdata.camera_h_charuco.transforms[0].translation.y))
			# # 	self.points_z.append(self._first_pose.pose.position.x  + self.axis_z*(-0.30 + userdata.camera_h_charuco.transforms[0].translation.z))
			
			# 	self.points_qx.append(self._first_pose.pose.orientation.x)
			# 	self.points_qy.append(self._first_pose.pose.orientation.y)
			# 	self.points_qz.append(self._first_pose.pose.orientation.z)
			# 	self.points_qw.append(self._first_pose.pose.orientation.w)
			# 	quaternion = quaternion_from_euler(np.radians(self._origin_euler[0]+15),np.radians(self._origin_euler[1]), np.radians(self._origin_euler[2]))  
			# 	##############################################yaw_angle, ##########################pitch_angle, ######################roll_angle  

			# 	for i in range(6):
			# 		self.points_x.append(self.points_x[0]  + self.cam_axis_y*self.move_distance*0.01)
			# 		self.points_y.append(self.points_y[0]  - self.cam_axis_z*self.move_distance*0.01 + self.cam_axis_z*0.035*i)
			# 		self.points_z.append(self.points_z[0])
			# 		self.points_qx.append(quaternion[0])
			# 		self.points_qy.append(quaternion[1])
			# 		self.points_qz.append(quaternion[2])
			# 		self.points_qw.append(quaternion[3])
			# 		i += 1
					
			# 	quaternion = quaternion_from_euler(np.radians(self._origin_euler[0]),np.radians(self._origin_euler[1]-15), np.radians(self._origin_euler[2]))  
			# 	##############################################yaw_angle, ##########################pitch_angle, ######################roll_angle  
			# 	for i in range(6,12):
			# 		self.points_x.append(self.points_x[6]  + 0.035*(i-6))
			# 		self.points_y.append(self.points_y[6])
			# 		self.points_z.append(self.points_z[0])
			# 		self.points_qx.append(quaternion[0])
			# 		self.points_qy.append(quaternion[1])
			# 		self.points_qz.append(quaternion[2])
			# 		self.points_qw.append(quaternion[3])
			# 		i += 1
			# 	quaternion = quaternion_from_euler(np.radians(self._origin_euler[0]-15),np.radians(self._origin_euler[1]), np.radians(self._origin_euler[2]))  
			# 	##############################################yaw_angle, ##########################pitch_angle, ######################roll_angle  
			# 	for i in range(12,18):
			# 		self.points_x.append(self.points_x[12])
			# 		self.points_y.append(self.points_y[6]  - self.cam_axis_z*0.035*(i-12))
			# 		self.points_z.append(self.points_z[0])
			# 		self.points_qx.append(quaternion[0])
			# 		self.points_qy.append(quaternion[1])
			# 		self.points_qz.append(quaternion[2])
			# 		self.points_qw.append(quaternion[3])
			# 		i += 1
			# 	quaternion = quaternion_from_euler(np.radians(self._origin_euler[0]),np.radians(self._origin_euler[1]-15), np.radians(self._origin_euler[2]))  
			# 	##############################################yaw_angle, ##########################pitch_angle, ######################roll_angle  
			# 	for i in range(18,24):
			# 		self.points_x.append(self.points_x[18]  - 0.035*(i-18))
			# 		self.points_y.append(self.points_y[18])
			# 		self.points_z.append(self.points_z[0])
			# 		self.points_qx.append(quaternion[0])
			# 		self.points_qy.append(quaternion[1])
			# 		self.points_qz.append(quaternion[2])
			# 		self.points_qw.append(quaternion[3])
			# 		i += 1


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
			# userdata.hand_eye_points = hand_eye_point()
			# userdata.hand_eye_points['x'] = self.points_x
			# userdata.hand_eye_points['y'] = self.points_y
			# userdata.hand_eye_points['z'] = self.points_z
			# userdata.hand_eye_points['qw'] = self.points_qw
			# userdata.hand_eye_points['qx'] = self.points_qx
			# userdata.hand_eye_points['qy'] = self.points_qy
			# userdata.hand_eye_points['qz'] = self.points_qz
			# if self._axis == "zxy":
			# 	userdata.hand_eye_points['x'] = self.points_z
			# 	userdata.hand_eye_points['y'] = self.points_x
			# 	userdata.hand_eye_points['z'] = self.points_y
			# 	userdata.hand_eye_points['qw'] = self.points_qw
			# 	userdata.hand_eye_points['qx'] = self.points_qx
			# 	userdata.hand_eye_points['qy'] = self.points_qy
			# 	userdata.hand_eye_points['qz'] = self.points_qz
			# print(self.waypoints)
			
			# userdata.hand_eye_points = hand_eye_point()
			# userdata.hand_eye_points['x'] = self.waypoints.position.x
			# userdata.hand_eye_points['y'] = self.waypoints.position.y
			# userdata.hand_eye_points['z'] = self.waypoints.position.z
			# userdata.hand_eye_points['qx'] = self.waypoints.orientation.x
			# userdata.hand_eye_points['qy'] = self.waypoints.orientation.y
			# userdata.hand_eye_points['qz'] = self.waypoints.orientation.z
			# userdata.hand_eye_points['qw'] = self.waypoints.orientation.w


			# # print(userdata.hand_eye_points)
			# # print("points")
			# print(userdata.hand_eye_points['x'])
			# print(userdata.hand_eye_points['y'])
			# print(userdata.hand_eye_points['z'])
			# print(userdata.hand_eye_points['x'][1])
			# print(userdata.hand_eye_points['y'][1])
			# print(userdata.hand_eye_points['z'][1])
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
		# print("fajklfjlka;jfklafkl;jaklfjalk;jfkla;jdf;klaj;flaflkjalk;jfkl;ajfkljaklfjakl")
		# print("fajklfjlka;jfklafkl;jaklfjalk;jfkla;jdf;klaj;flaflkjalk;jfkl;ajfkljaklfjakl")
		origindegree = euler_from_quaternion([self._first_pose.pose.orientation.x, self._first_pose.pose.orientation.y, self._first_pose.pose.orientation.z, self._first_pose.pose.orientation.w])
		self._origin_euler[0] = origindegree[0]/3.14*180.0
		self._origin_euler[1] = origindegree[1]/3.14*180.0
		self._origin_euler[2] = origindegree[2]/3.14*180.0
		# self.First_charuco_list = np.array(userdata.camera_h_charuco.transforms).shape
		# self._result = self._move_group.execute(userdata.joint_trajectory)
		# self.points = self.points.append(self._move_group.execute(userdata.camera_h_charuco))
		# self.points.hand_eye_points['x'] = userdata.camera_h_charuco.transforms.x

		

		if self.move_distance <= 11.0:

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
				# First move up (z)
				self.wpose.position.z += self.cam_axis_z*(0.45-userdata.camera_h_charuco.transforms[0].translation.z)  # First move up (z)
				quaternion = quaternion_from_euler(np.radians(self._origin_euler[0]+(45-aruco_rotation_degree[0])),
													np.radians(self._origin_euler[1]+(10-aruco_rotation_degree[1])), 
													np.radians(self._origin_euler[2]))  	
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
				quaternion = quaternion_from_euler(np.radians(self._origin_euler[0]+(45-aruco_rotation_degree[0])+15*self.eye_in_hand_mode),
													np.radians(self._origin_euler[1]+(10-aruco_rotation_degree[1])), 
													np.radians(self._origin_euler[2]))  	
				##############################################roll_angle, ##########################pitch_angle, ######################yaw_angle  
				self.wpose.position.x += self.cam_axis_x*self.move_distance*0.01
				self.wpose.position.y -= self.cam_axis_y*self.move_distance*0.01
				for self.loop_num in range(self.loop_size):
					self.wpose.position.y += self.cam_axis_y*0.035
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
				quaternion = quaternion_from_euler(np.radians(self._origin_euler[0]+(45-aruco_rotation_degree[0])),
													np.radians(self._origin_euler[1]+(10-aruco_rotation_degree[1])), 
													np.radians(self._origin_euler[2]))  	
				##############################################roll_angle, ##########################pitch_angle, ######################yaw_angle  
				
				for self.loop_num in range(self.loop_size,self.loop_size*2):
					self.wpose.position.x -= self.cam_axis_x*0.035
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
				quaternion = quaternion_from_euler(np.radians(self._origin_euler[0]+(45-aruco_rotation_degree[0])-15*self.eye_in_hand_mode),
													np.radians(self._origin_euler[1]+(10-aruco_rotation_degree[1])), 
													np.radians(self._origin_euler[2]))  	
				##############################################roll_angle, ##########################pitch_angle, ######################yaw_angle  
				for self.loop_num in range(self.loop_size*2,self.loop_size*3):
					self.wpose.position.y -= self.cam_axis_y*0.035
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
				quaternion = quaternion_from_euler(np.radians(self._origin_euler[0]+(45-aruco_rotation_degree[0])),
													np.radians(self._origin_euler[1]+(10-aruco_rotation_degree[1])), 
													np.radians(self._origin_euler[2]))  	
				##############################################roll_angle, ##########################pitch_angle, ######################yaw_angle  
				for self.loop_num in range(self.loop_size*3,self.points_num):
					self.wpose.position.x += self.cam_axis_x*0.035
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
		pass

	def on_stop(self):
		pass

	def on_pause(self):
		pass

	def on_resume(self, userdata):
		# self.on_enter(userdata)
		pass
