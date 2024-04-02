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
Define go_to_pose.

Created on Tue Apr 02 2024
@author: ggh-png
"""


from flexbe_core import Autonomy
from flexbe_core import Behavior
from flexbe_core import ConcurrencyContainer
from flexbe_core import Logger
from flexbe_core import OperatableStateMachine
from flexbe_core import PriorityContainer
from turtlebot3_flexbe_behaviors.tts_sm import ttsSM
from turtlebot3_flexbe_states.nav_to_pose import NaviToPoseState
from turtlebot3_flexbe_states.tts_state import TTSState

# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


class go_to_poseSM(Behavior):
    """
    Define go_to_pose.

    go to pose

    """

    def __init__(self, node):
        super().__init__()
        self.name = 'go_to_pose'

        # parameters of this behavior

        # references to used behaviors
        OperatableStateMachine.initialize_ros(node)
        ConcurrencyContainer.initialize_ros(node)
        PriorityContainer.initialize_ros(node)
        Logger.initialize(node)
        NaviToPoseState.initialize_ros(node)
        TTSState.initialize_ros(node)
        self.add_behavior(ttsSM, 'tts', node)

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]

        # [/MANUAL_INIT]

        # Behavior comments:

    def create(self):
        waypoint_3 = [-0.5, -0.5, 1.1, 1.1]
        waypoint_2 = [0.5, -0.5, 1.1, 1.1]
        waypoint_1 = [0.5, 0.5, 1.1, 1.1]
        waypoint_4 = [-0.5, 0.5, 1.1, 1.1]
        # x:30 y:365, x:680 y:299
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
        _state_machine.userdata.waypoint_1 = waypoint_1
        _state_machine.userdata.waypoint_2 = waypoint_2
        _state_machine.userdata.waypoint_3 = waypoint_3
        _state_machine.userdata.waypoint_4 = waypoint_4

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]

        # [/MANUAL_CREATE]
        with _state_machine:
            # x:138 y:102
            OperatableStateMachine.add('tts',
                                       self.use_behavior(ttsSM, 'tts'),
                                       transitions={'finished': 'go_to_pose_1', 'failed': 'failed'},
                                       autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

            # x:794 y:145
            OperatableStateMachine.add('go_to_pose_2',
                                       NaviToPoseState(timeout=12000.0, navi_to_pos_topic="/navigate_to_pose", waypoint=waypoint_2),
                                       transitions={'failed': 'failed', 'done': 'point1_tts_2'},
                                       autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off})

            # x:791 y:397
            OperatableStateMachine.add('go_to_pose_3',
                                       NaviToPoseState(timeout=12000.0, navi_to_pos_topic="/navigate_to_pose", waypoint=waypoint_3),
                                       transitions={'failed': 'failed', 'done': 'point1_tts_3'},
                                       autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off})

            # x:439 y:422
            OperatableStateMachine.add('go_to_pose_4',
                                       NaviToPoseState(timeout=12000.0, navi_to_pos_topic="/navigate_to_pose", waypoint=waypoint_4),
                                       transitions={'failed': 'failed', 'done': 'point1_tts_4'},
                                       autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off})

            # x:685 y:59
            OperatableStateMachine.add('point1_tts',
                                       TTSState(text="첫번째 지점입니다", service_topic='/turtlebot3/tts'),
                                       transitions={'done': 'go_to_pose_2', 'failed': 'failed'},
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

            # x:814 y:244
            OperatableStateMachine.add('point1_tts_2',
                                       TTSState(text="두번째 지점입니다", service_topic='/turtlebot3/tts'),
                                       transitions={'done': 'go_to_pose_3', 'failed': 'failed'},
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

            # x:604 y:431
            OperatableStateMachine.add('point1_tts_3',
                                       TTSState(text="세번째 지점입니다.", service_topic='/turtlebot3/tts'),
                                       transitions={'done': 'go_to_pose_4', 'failed': 'failed'},
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

            # x:222 y:342
            OperatableStateMachine.add('point1_tts_4',
                                       TTSState(text="정찰이 끝났습니다 종료합니다.", service_topic='/turtlebot3/tts'),
                                       transitions={'done': 'finished', 'failed': 'failed'},
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

            # x:451 y:60
            OperatableStateMachine.add('go_to_pose_1',
                                       NaviToPoseState(timeout=12000.0, navi_to_pos_topic="/navigate_to_pose", waypoint=waypoint_1),
                                       transitions={'failed': 'failed', 'done': 'point1_tts'},
                                       autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off})

        return _state_machine

    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]

    # [/MANUAL_FUNC]
