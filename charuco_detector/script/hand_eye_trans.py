#!/usr/bin/env python3
import os
import numpy as np
import rospy
import rospkg
import tf
import configparser
from math import asin, atan2, degrees, pi
from charuco_detector.srv import eye2base, eye2baseResponse
from visp_hand2eye_calibration.msg import TransformArray
from geometry_msgs.msg import Transform

# from tf.listener import TransformerROS
#  tf.TransformerROS



class HandEyeTrans:
	def __init__(self, base_link, tip_link):
		self.save_camera_cali_pwd = os.path.join(os.path.dirname(__file__), '..','..','config/')
		self.hand_eye_cali_pwd = os.path.join(os.path.dirname(__file__), '..','config/')

		self.base_link = base_link
		self.tip_link = tip_link
		self.tf_listener = tf.TransformListener()
		self.base_h_tool = TransformArray()
		self.camera_h_charuco = TransformArray()
		self.base_h_tool.header.frame_id = self.base_link
		self.camera_h_charuco.header.frame_id = 'calib_camera'
		self._rtool_tool_trans = np.mat(np.identity(4))


		self._img_pose = np.zeros(6)
		self._base_pose = np.zeros(6)
		self._curr_pose = np.zeros(6)
		self._tool_coor = np.zeros(6)
		self._base_coor = np.zeros(6)
		self._base_tool_trans =  np.mat(np.identity(4))
		# self._rbase_base_trans = np.mat(np.identity(4))
		# self._rtool_tool_trans = np.mat(np.identity(4))
		self._hand_eye_trans =   np.mat(np.identity(4))
		self._rtool_eye_trans, self._camera_mat = self.__get_camera_param()

		self.__eye2base_server = rospy.Service('eye2base',
				eye2base,
				self.__eye2base_transform
		)
		self.__pix2base_server = rospy.Service('pix2base',
				eye2base,
				self.__pix2base_transform
		)
		self.__eye_trans2base_server = rospy.Service('eye_trans2base',
				eye2base,
				self.__eye_trans2base_transform
		)

	# def __get_feedback(self):
	# 	rospy.wait_for_service('get_kinematics_pose')
	# 	try:
	# 		get_endpos = rospy.ServiceProxy(
	# 			'get_kinematics_pose',
	# 			GetKinematicsPose
	# 		)
	# 		res = get_endpos('arm')
	# 		return res
	# 	except rospy.ServiceException as e:
	# 		print ("Service call failed: %s" % e)

	def __get_camera_param(self):
		config = configparser.ConfigParser()
		config.optionxform = str

		config.read(self.save_camera_cali_pwd + 'camera_calibration.ini')
		
		K00 = float(config.get("Intrinsic", "0_0"))
		K01 = float(config.get("Intrinsic", "0_1"))
		K02 = float(config.get("Intrinsic", "0_2"))
		K10 = float(config.get("Intrinsic", "1_0"))
		K11 = float(config.get("Intrinsic", "1_1"))
		K12 = float(config.get("Intrinsic", "1_2"))
		K20 = float(config.get("Intrinsic", "2_0"))
		K21 = float(config.get("Intrinsic", "2_1"))
		K22 = float(config.get("Intrinsic", "2_2"))

		# color_width = rospy.get_param('~color_width')
		# color_high = rospy.get_param('~color_high')
		# internal_name = 'Internal_' + str(color_width) + '_' + str(color_high)


		config.read(self.hand_eye_cali_pwd + 'hand_eye_calibration.ini')

		x  = float(config.get("hand_eye_calibration", "x" ))
		y  = float(config.get("hand_eye_calibration", "y" ))
		z  = float(config.get("hand_eye_calibration", "z" ))
		qx = float(config.get("hand_eye_calibration", "qx"))
		qy = float(config.get("hand_eye_calibration", "qy"))
		qz = float(config.get("hand_eye_calibration", "qz"))
		qw = float(config.get("hand_eye_calibration", "qw"))


		trans = Transform()
		trans.translation.x = x 
		trans.translation.y = y
		trans.translation.z = z
		trans.rotation.x = qx
		trans.rotation.y = qy
		trans.rotation.z = qz
		trans.rotation.w = qw
		# tool_eye_trans = self.tf_listener.fromTranslationRotation(trans.translation,trans.rotation)
		tool_eye_trans = self.tf_listener.fromTranslationRotation((x,y,z),(qx,qy,qz,qw))
		print(tool_eye_trans)

		# Ex = np.mat([[a00, a01, a02, a03],
		# 			 [a10, a11, a12, a13],
		# 			 [a20, a21, a22, a23],
		# 			 [0,   0,   0,   1]])


		Intrinsic = np.mat([[K00, K01, K02],
					 		[K10, K11, K12],
					 		[K20, K21, K22]])
		print(Intrinsic)
		return tool_eye_trans, Intrinsic

	def __get_robot_trans(self):
		try:
			(base_trans_tool, base_rot_tool) = self.tf_listener.lookupTransform(self.base_link, self.tip_link, rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			rospy.logwarn('lookupTransform for robot failed!, ' + self.base_link + ', ' + self.tip_link)

		trans = Transform()
		trans.translation.x = base_trans_tool[0]
		trans.translation.y = base_trans_tool[1]
		trans.translation.z = base_trans_tool[2]
		trans.rotation.x = base_rot_tool[0]
		trans.rotation.y = base_rot_tool[1]
		trans.rotation.z = base_rot_tool[2]
		trans.rotation.w = base_rot_tool[3]
		self.base_h_tool.transforms.append(trans)

		self._base_tool_trans = self.tf_listener.fromTranslationRotation((trans.translation.x,trans.translation.y,trans.translation.z),
																		(trans.rotation.x,trans.rotation.y,trans.rotation.z,trans.rotation.w))
			
		self._base_tool_trans = np.mat(self._base_tool_trans).reshape(4, 4)
		return

	def __rotation2rpy(self, rotation):
		_rpy = []
		_rpy.append(degrees(asin(rotation[1, 2])))
		_rpy.append(degrees(atan2(rotation[0, 2], -rotation[2, 2])))
		_rpy.append(degrees(atan2(-rotation[1, 0], -rotation[1, 1])))
		return _rpy
		
	def __eye2base_transform(self, req):
		self.__get_robot_trans()
		assert len(req.ini_pose) == 3
		eye_obj_trans = np.mat(np.append(np.array(req.ini_pose), 1)).reshape(4, 1)
		# eye_obj_trans[:3] = np.multiply(eye_obj_trans[:3], 0.01)
		result = self._base_tool_trans * np.linalg.inv(self._rtool_tool_trans) * self._rtool_eye_trans * eye_obj_trans
		res = eye2baseResponse()
		res.trans = np.mat(np.identity(4))
		res.trans[2, :] = result 
		res.pos = np.array(result[:3]).reshape(-1)
		res.euler = self.__rotation2rpy(res.trans)
		return res

	def __eye_trans2base_transform(self, req):
		self.__get_robot_trans()
		assert len(req.ini_pose) == 16
		eye_obj_trans = np.mat(req.ini_pose).reshape(4, 4)
		result = self._base_tool_trans * np.linalg.inv(self._rtool_tool_trans) * self._rtool_eye_trans * eye_obj_trans
		res = eye2baseResponse()
		res.trans = np.array(result).reshape(-1)
		res.pos = np.array(result[0:3, 3]).reshape(-1)
		res.euler = self.__rotation2rpy(result)
		return res
		

	def __pix2base_transform(self, req):
		self.__get_robot_trans()
		assert len(req.ini_pose) == 3
		eye_obj_trans = np.mat(np.append(np.array(req.ini_pose), 1)).reshape(4, 1)
		# eye_obj_trans[2] = eye_obj_trans[2] * 0.01
		eye_obj_trans[:2] = (eye_obj_trans[:2] - self._camera_mat[:2, 2:]) * eye_obj_trans[2]
		eye_obj_trans[:2] = np.multiply(eye_obj_trans[:2], [[1/self._camera_mat[0, 0]], [1/self._camera_mat[1, 1]]])
		result = self._base_tool_trans * np.linalg.inv(self._rtool_tool_trans) * self._rtool_eye_trans * eye_obj_trans
		res = eye2baseResponse()
		res.trans = np.mat(np.identity(4))
		res.trans[2, :] = result
		res.pos = np.array(result[:3]).reshape(-1)
		res.euler = self.__rotation2rpy(res.trans)
		return res

if __name__ == "__main__":
	rospy.init_node('hand_eye_trans')
	base_link = rospy.get_param('~base_link')
	tip_link = rospy.get_param('~tip_link')
	processing = HandEyeTrans(base_link, tip_link)
	rospy.spin()