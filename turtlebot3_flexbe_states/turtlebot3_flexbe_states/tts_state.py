#!/usr/bin/env python
from flexbe_core import EventState, Logger
from turtlebot3_interfaces.srv import TTS
from flexbe_core.proxy.proxy_service_caller import ProxyServiceCaller

class TTSState(EventState):

    def __init__(self, text="안녕하세요 나는 로봇입니다.", service_topic='/turtlebot3/tts'):
        super().__init__(outcomes=['done', 'failed'])
        self.text = text
        self.service_topic = service_topic
        ProxyServiceCaller.initialize(TTSState._node)
        self.caller = ProxyServiceCaller({self.service_topic: TTS})

    def on_enter(self, userdata):
        Logger.loginfo("Entered TTS state.")
        
        try:
            request = TTS.Request()
            request.tts_str_t = self.text
            Logger.loginfo(f"Requesting TTS for: {self.text}")
            self.caller.call_async(self.service_topic, request)
        except Exception as e:
            Logger.loginfo(f"Failed to send request to TTS service: {str(e)}")
            self._outcome = 'failed'

    def execute(self, userdata):
        if self.caller.done(self.service_topic):
            result = self.caller.result(self.service_topic)
            if result is not None and result.success:
                Logger.loginfo("TTS processing completed successfully.")
                return 'done'
            else:
                Logger.logerror("TTS service reported an error.")
                return 'failed'