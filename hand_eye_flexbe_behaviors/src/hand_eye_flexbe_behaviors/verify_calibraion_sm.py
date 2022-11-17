#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hand_eye_flexbe_states.get_ar_marker import GetArMarker
from hand_eye_flexbe_states.obj_trans_to_arm import ObjTransToArm
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Nov 17 2022
@author: Luis
'''
class verify_calibraionSM(Behavior):
	'''
	verify_calibraion
	'''


	def __init__(self):
		super(verify_calibraionSM, self).__init__()
		self.name = 'verify_calibraion'

		# parameters of this behavior
		self.add_parameter('eye_in_hand_mode', False)
		self.add_parameter('base_link', '/base_link')
		self.add_parameter('tip_link', '/tool0_controller')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:602 y:314, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:193 y:98
			OperatableStateMachine.add('get_obj_position',
										GetArMarker(),
										transitions={'done': 'obj_to_arm_base', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'camera_h_charuco': 'camera_h_charuco'})

			# x:272 y:192
			OperatableStateMachine.add('obj_to_arm_base',
										ObjTransToArm(eye_in_hand_mode=self.eye_in_hand_mode, base_link=self.base_link, tip_link=self.tip_link),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'camera_h_charuco': 'camera_h_charuco', 'obj_position': 'obj_position'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
