#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2024 ggh
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
Define tts.

Created on Tue Apr 02 2024
@author: ggh
"""


from flexbe_core import Autonomy
from flexbe_core import Behavior
from flexbe_core import ConcurrencyContainer
from flexbe_core import Logger
from flexbe_core import OperatableStateMachine
from flexbe_core import PriorityContainer
from turtlebot3_flexbe_states.tts_state import TTSState

# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


class ttsSM(Behavior):
    """
    Define tts.

    tts

    """

    def __init__(self, node):
        super().__init__()
        self.name = 'tts'

        # parameters of this behavior

        # references to used behaviors
        OperatableStateMachine.initialize_ros(node)
        ConcurrencyContainer.initialize_ros(node)
        PriorityContainer.initialize_ros(node)
        Logger.initialize(node)
        TTSState.initialize_ros(node)

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]

        # [/MANUAL_INIT]

        # Behavior comments:

    def create(self):
        tts_2 = "안녕하세요"
        tts_1 = "오토보 여러분"
        tts_3 = "반갑습니다"
        # x:610 y:473, x:196 y:386
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
        _state_machine.userdata.tts_1 = tts_1
        _state_machine.userdata.tts_2 = tts_2
        _state_machine.userdata.tts_3 = tts_3

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]

        # [/MANUAL_CREATE]
        with _state_machine:
            # x:270 y:80
            OperatableStateMachine.add('tts_state',
                                       TTSState(text=tts_1, service_topic='/turtlebot3/tts'),
                                       transitions={'done': 'tts_state_2', 'failed': 'failed'},
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

            # x:458 y:153
            OperatableStateMachine.add('tts_state_2',
                                       TTSState(text=tts_2, service_topic='/turtlebot3/tts'),
                                       transitions={'done': 'tts_state_3', 'failed': 'failed'},
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

            # x:674 y:239
            OperatableStateMachine.add('tts_state_3',
                                       TTSState(text=tts_3, service_topic='/turtlebot3/tts'),
                                       transitions={'done': 'finished', 'failed': 'failed'},
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

        return _state_machine

    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]

    # [/MANUAL_FUNC]
