#!/usr/bin/env python

from rclpy.duration import Duration

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

# Navigate to pose 관련 라이브러리
from nav2_msgs.action import NavigateToPose


class NaviToPoseState(EventState):
    # 초기화 함수
    def __init__(self, timeout=120.0, navi_to_pos_topic="/navigate_to_pose", waypoint=[0.0, 0.0, 0.0, 0.0]):
        super().__init__(outcomes=['failed', 'done']) 

        # 타이머 설정
        self._timeout = Duration(seconds=timeout)
        self._timeout_sec = timeout
        self._start_time = None
        
        # 토픽 설정
        self.navi_to_pos_topic = navi_to_pos_topic
        self.waypoint = waypoint
        # navigate to pose 액션 클라이언트 초기화
        ProxyActionClient.initialize(NaviToPoseState._node)
        self._client = ProxyActionClient({self.navi_to_pos_topic: NavigateToPose})

        # 액션 클라이언트 실패 가능성
        self._sucess = False
        self._failed = False


    # 목표 위치로 이동하는 함수
    def goToPose(self):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.pose.position.x = self.waypoint[0]
        goal_msg.pose.pose.position.y = self.waypoint[1]
        goal_msg.pose.pose.orientation.z = self.waypoint[2]
        goal_msg.pose.pose.orientation.w = self.waypoint[3]

        self.goal_handle = self._client.send_goal(self.navi_to_pos_topic, goal_msg)
        self.info("목표 위치로 이동 명령 발송")
        self._start_time = self._node.get_clock().now()
        self._sucess = False
    

    # 네비게이션 취소 함수
    def navi_cancel(self):
        if not self._client.has_result(self.navi_to_pos_topic):
            self._client.cancel(self.navi_to_pos_topic)
            self.info("네비게이션 취소")
        else:
            self.info("네비게이션 완료")

    # 상태 실행 함수
    def execute(self, userdata):
        elapsed_time = self._node.get_clock().now() - self._start_time
        if self._client.has_result(self.navi_to_pos_topic):
            self.info("네비게이션 완료")
            self._sucess = True

        if self._sucess:
            return 'done'
        if self._failed or elapsed_time >= self._timeout:
            return 'failed'
        
    # 상태 진입 함수
    def on_enter(self, userdata):
        self._sucess = False
        self._failed = False
        self.feedback_flag = False

        self.info("네비게이션 시작")
        self.goToPose()

    # 상태 종료 함수
    def on_exit(self, userdata):
        if not self._client.has_result(self.navi_to_pos_topic):
            self._client.cancel(self.navi_to_pos_topic)
            self.info("네비게이션 취소")
        else:
            self.info("네비게이션 완료")

    # 로그 함수
    def info(self, msg):
        Logger.loginfo(msg)
    def warn(self, msg):
        Logger.logwarn(msg)
    def err(self, msg):
        Logger.logerr(msg)