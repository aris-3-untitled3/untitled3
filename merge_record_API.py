# main 파일에서 VtoT = Record_API(file_name) 와 같은 형식으로 불러오고 
# VtoT.record_audio() 이런식으로 넣으면 실행 될 것임

# Imports the Google Cloud client library

import io
import os

import speech_recognition as sr
from playsound import playsound
from google.cloud import speech

class Record_API:
    def __init__(self, file_name):
        # Instantiates a client
        self.client = speech.SpeechClient()
        self.file_name = file_name
        os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = "/home/sophie/untitled3-speech-API.json"
    
    # 음성 녹음 함수
    def record_audio(self):
        # Recognizer 객체 생성
        r = sr.Recognizer()

        # 마이크를 오디오 소스로 사용
        mic = sr.Microphone()

        with mic as source:
            print("녹음 시작...")
            audio_data = r.listen(source, phrase_time_limit=5)

            print("녹음 완료!")
            return audio_data

    # 오디오 파일 재생 함수
    def play_audio(self):
        playsound(self.file_name)


    def speech_API(self):

        # Loads the audio into memory
        with io.open(self.file_name, 'rb') as audio_file:
            content = audio_file.read()
            audio = speech.RecognitionAudio(content=content)
        #FLAC ==> encoding=speech.RecognitionConfig.AudioEncoding.FLAC
        config = speech.RecognitionConfig(
            encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
            sample_rate_hertz=44100,
            language_code='ko-KR',
            audio_channel_count=1  # FLAC 파일의 오디오 채널 수는 2, wav는 1
        )

        # Detects speech in the audio file
        response = self.client.recognize(config=config, audio=audio)

        save_path = "response.txt"

        for result in response.results:
            print('Transcript: {}'.format(result.alternatives[0].transcript))
            with open(save_path, 'w') as sp:
                sp.write(result.alternatives[0].transcript)
            if save_path:
                print("saved!!")
            else:   
                print("Error saving")

    def run(self):
        audio_data = self.record_audio()

        if audio_data is not None:
        # 오디오 파일로 저장
            with open(self.file_name, "wb") as f:
                f.write(audio_data.get_wav_data())
            ## 오디오 파일 재생
            #self.play_audio()
            self.speech_API()
            print("성공적으로 작동됨!!!!!!!")

        else:
            print("녹음 실패. 오디오 파일이 생성되지 않았습니다.")

if __name__ == "__main__":
    file_name = "response3333.wav"
    record_api = Record_API(file_name)
    record_api.run()