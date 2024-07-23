import rclpy as rp
from rclpy.node import Node
from untitled_msgs.srv import ServiceString
import speech_recognition as sr
from playsound import playsound

class Record_API(Node):
    def __init__(self):
        super().__init__('Record_API')

        self.file_name="/home/jchj/Untitled3/src/untitled3/resource/output.wav"
        self.save_path="/home/jchj/Untitled3/src/untitled3/resource/response.txt"
        self.respone_time=5

        # /Signal_Voice_out 서비스
        self.voice_in_server = self.create_service(
            ServiceString,
            '/Call_to_Voice_in',
            self.voice_in_callback,
        )

    def voice_in_callback(self, request, response):
        if request.command == "hear":
            self.get_logger().info(f'Received : {request.command}')
            text = self.run()
        else:
            self.get_logger().error(f'ERROR')
            text = "error"

        response.success = True
        response.result = f'response : {text}'
        return response


    # 음성 녹음 함수
    def record_audio(self):
        # Recognizer 객체 생성
        r = sr.Recognizer()

        # 마이크를 오디오 소스로 사용
        mic = sr.Microphone()
        
        with mic as source:
            print("################################################################################################")
            print("################################################################################################")
            print("################################################################################################")
            print("녹음 시작...")
            audio_data = r.listen(source, phrase_time_limit=self.respone_time)

            print("녹음 완료!")
            return audio_data

    # 오디오 파일 재생 함수
    def play_audio(self):
        playsound(self.file_name)

    def speech_to_text(self, audio_path):
        recognizer = sr.Recognizer()

        # Load audio file
        with sr.AudioFile(audio_path) as source:
            audio_data = recognizer.record(source)

        try:
            # Perform speech recognition
            text = recognizer.recognize_google(audio_data, language='ko-KR')  # 한국어로 설정
            return text
        except sr.UnknownValueError:
            print(f"Speech recognition could not understand audio: {audio_path}")
            return None
        except sr.RequestError as e:
            print(f"Could not request results from Google Speech Recognition service for {audio_path}: {e}")
            return None

    def run(self):
        audio_data = self.record_audio()

        if audio_data:
            # 오디오 파일로 저장
            with open(self.file_name, "wb") as f:
                f.write(audio_data.get_wav_data())

            # 음성 파일을 텍스트로 변환
            text = self.speech_to_text(self.file_name)
            if text:
                with open(self.save_path, 'w') as sp:
                    sp.write(text)
                print(f"Transcript: {text}")
                return text  # 텍스트 반환
            else:
                return None
        
        else:
            print("녹음 실패. 오디오 파일이 생성되지 않았습니다.")
            return None

def main(args=None):
    rp.init(args=args)

    Record_node = Record_API()

    try:
        rp.spin(Record_node)
    finally:
        Record_node.destroy_node()
        rp.shutdown()


if __name__ == '__main__':
    main()
