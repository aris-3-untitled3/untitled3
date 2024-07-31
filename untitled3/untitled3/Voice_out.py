import rclpy as rp
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from untitled_msgs.msg import TopicString
from untitled_msgs.srv import ServiceString
import threading
import os
import pygame
from gtts import gTTS
import pyttsx3

class VoiceOut(Node):

    def __init__(self):
        super().__init__('VoiceOut')

        # 콜백 그룹 정의
        self.voice_callback_group = ReentrantCallbackGroup()
        self.bgm_callback_group = ReentrantCallbackGroup()

        # /Signal_Voice_out 서비스
        self.voice_out_server = self.create_service(
            ServiceString,
            '/Call_to_Voice_out',
            self.voice_out_callback,
            callback_group=self.voice_callback_group
        )

        # /Server_Voice_out 토픽 구독
        self.voice_control_subscription = self.create_subscription(
            TopicString,
            '/Server_to_Voice_out',
            self.voice_control_callback,
            10,
            callback_group=self.bgm_callback_group
        )

        pygame.mixer.init()
        self.tts_stop_event = threading.Event()

    def voice_out_callback(self, request, response):
        self.get_logger().info(f'Received : {request.command}')

        # 음성 출력을 동기적으로 실행
        self.tts_stop_event.clear()
        self.play_tts(request.command)

        response.success = True
        response.result = f'{request.command} completed!'
        print(response.result)
        return response

    def voice_control_callback(self, msg):
        if msg.command == "play_bgm":
            self.get_logger().info('play_bgm')
            self.bgm_thread = threading.Thread(target=self.play_bgm)
            self.bgm_thread.start()
        elif msg.command == "stop_bgm":
            self.get_logger().info('stop_bgm!')
            pygame.mixer.music.stop()
        elif msg.command == "stop_tts":
            self.get_logger().info('stop_tts!')
            self.tts_stop_event.set()  # TTS 스레드를 멈추기 위한 이벤트 설정
        else:
            self.get_logger().error('ERROR!')

    def play_bgm(self):
        pygame.mixer.music.load("/home/jchj/Untitled3/src/untitled3/UI/bgm.mp3")
        pygame.mixer.music.set_volume(0.7)  # BGM 볼륨을 30%로 설정
        pygame.mixer.music.play(-1)  # 무한 반복 재생

    def play_tt_no(self,text):
        engine = pyttsx3.init()  # Initialize the TTS engine
        voices = engine.getProperty('voices')
        print(voices[10].id)
        engine.setProperty('voice', voices[2].id)  # 원하는 음성 인덱스 선택
        engine.setProperty('rate', 150)  # 음성 속도 조절
        engine.setProperty('volume', 1)  # 볼륨 조절
        engine.say(text)         # Queue the text to be spoken
        engine.runAndWait()

    def play_tts(self, text):
        # TTS 생성
        tts = gTTS(text, lang='ko')
        
        # 임시 파일로 저장
        temp_file = 'temp.mp3'
        tts.save(temp_file)

        # 재생
        sound = pygame.mixer.Sound(temp_file)
        sound.play()

        # 재생이 완료될 때까지 대기
        while pygame.mixer.get_busy():
            if self.tts_stop_event.is_set():  # 중단 요청 확인
                self.get_logger().info('Stopping TTS...')
                sound.stop()
                break
            pygame.time.Clock().tick(10)

        # 임시 파일 삭제
        os.remove(temp_file)

def main(args=None):
    rp.init(args=args)
    voice_out_node = VoiceOut()

    # MultiThreadedExecutor 사용
    executor = MultiThreadedExecutor()
    executor.add_node(voice_out_node)

    try:
        executor.spin()
    finally:
        voice_out_node.destroy_node()
        rp.shutdown()

if __name__ == '__main__':
    main()
