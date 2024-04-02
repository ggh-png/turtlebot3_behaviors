#!/usr/bin/env python

from rclpy.duration import Duration
from flexbe_core import EventState, Logger

class SecondWaitState(EventState):
    """
    이 클래스는 지정된 시간(기본적으로 5초) 동안 기다린 후 'done'을 반환하는 상태를 정의합니다.
    """
    def __init__(self, target_time=5.0):
        """
        클래스 초기화 메서드.
        :param target_time: 대기할 시간 (초 단위), 기본값은 5초입니다.
        """
        super().__init__(outcomes=['done'])
        # 대기할 시간을 설정합니다.
        self._target_wait_time = Duration(seconds=target_time)

        # 상태 변수 초기화
        self._start_time = None

    def on_start(self):
        """
        상태 머신이 시작될 때 호출됩니다.
        """
        Logger.loginfo("on_start called.")

    def on_enter(self, userdata):
        """
        이 상태로 진입할 때 호출됩니다.
        현재 시간을 기록하여 대기 시간 계산을 시작합니다.
        """
        self._start_time = self._node.get_clock().now()
        Logger.loginfo("on_enter called")

    def execute(self, userdata):
        """
        상태가 활성화되어 있는 동안 주기적으로 호출됩니다.
        지정된 시간이 경과했는지 확인하고, 경과했다면 'done'을 반환합니다.
        """
        elapsed_time = self._node.get_clock().now() - self._start_time
        # 지정된 시간이 경과했는지 확인합니다.
        if elapsed_time >= self._target_wait_time:
            Logger.loginfo("5 seconds have passed. Returning 'done'.")
            return 'done'
        # 나노 초 세컨드를 초 단위로 변환합니다.
        

        # 현재 경과 시간을 로그로 출력합니다.
        Logger.loginfo("Current time: {}".format(elapsed_time))
        return None

    def on_exit(self, userdata):
        """
        이 상태에서 나갈 때 호출됩니다.
        상태 종료를 로그로 기록합니다.
        """
        Logger.loginfo("on_exit called.")
    
    def on_stop(self):
        """
        상태 머신이 중단될 때 호출됩니다.
        상태 중단을 로그로 기록합니다.
        """
        Logger.loginfo("on_stop called.")