import io
import os

# Imports the Google Cloud client library
from google.cloud import speech

# Instantiates a client
client = speech.SpeechClient()

# The name of the audio file to transcribe
file_name = os.path.join(
    os.path.dirname(__file__),
    '.',
    'vanilla_topB.flac')

# Loads the audio into memory
with io.open(file_name, 'rb') as audio_file:
    content = audio_file.read()
    audio = speech.RecognitionAudio(content=content)

config = speech.RecognitionConfig(
    encoding=speech.RecognitionConfig.AudioEncoding.FLAC,
    sample_rate_hertz=44100,
    language_code='ko-KR',
    audio_channel_count=2  # FLAC 파일의 오디오 채널 수를 2로 설정
)

# Detects speech in the audio file
response = client.recognize(config=config, audio=audio)

save_path = "vanilla_topB.txt"

for result in response.results:
    print('Transcript: {}'.format(result.alternatives[0].transcript))
    with open(save_path, 'w') as sp:
        sp.write(result.alternatives[0].transcript)
    if save_path:
        print("saved!!")
    else:   
        print("Error saving")
