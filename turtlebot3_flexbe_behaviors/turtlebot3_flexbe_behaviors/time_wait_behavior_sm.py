#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2024 ggh-png
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

"""
Define Time_wait_behavior.

Created on Tue Apr 02 2024
@author: ggh-png
"""


from flexbe_core import Autonomy
from flexbe_core import Behavior
from flexbe_core import ConcurrencyContainer
from flexbe_core import Logger
from flexbe_core import OperatableStateMachine
from flexbe_core import PriorityContainer
from turtlebot3_flexbe_states.second_wait_state import SecondWaitState

# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


class Time_wait_behaviorSM(Behavior):
    """
    Define Time_wait_behavior.

    Time wait behavior 
    """

    def __init__(self, node):
        super().__init__()
        self.name = 'Time_wait_behavior'

        # parameters of this behavior

        # references to used behaviors
        OperatableStateMachine.initialize_ros(node)
        ConcurrencyContainer.initialize_ros(node)
        PriorityContainer.initialize_ros(node)
        Logger.initialize(node)
        SecondWaitState.initialize_ros(node)

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]

        # [/MANUAL_INIT]

        # Behavior comments:

    def create(self):
        target_time = 3
        # x:469 y:430, x:255 y:427
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
        _state_machine.userdata.target_time = target_time

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]

        # [/MANUAL_CREATE]
        with _state_machine:
            # x:208 y:72
            OperatableStateMachine.add('test',
                                       SecondWaitState(target_time=5.0),
                                       transitions={'done': 'test_2'},
                                       autonomy={'done': Autonomy.Off})

            # x:355 y:252
            OperatableStateMachine.add('test_2',
                                       SecondWaitState(target_time=target_time),
                                       transitions={'done': 'test_3'},
                                       autonomy={'done': Autonomy.Off})

            # x:602 y:274
            OperatableStateMachine.add('test_3',
                                       SecondWaitState(target_time=5.0),
                                       transitions={'done': 'finished'},
                                       autonomy={'done': Autonomy.Off})

        return _state_machine

    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]

    # [/MANUAL_FUNC]
