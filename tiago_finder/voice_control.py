from google.cloud import speech
from playsound import playsound
import pyaudio
import gtts

import queue
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Voice_Control(Node):
    def __init__(self):
        super().__init__('voice_control')

        msg_1 = "Hello! Please say the name of the object that you "
        msg_2 = "would like to find. Currently mug, laptop and book "
        msg_3 = "are supported."
        welcome_msg = msg_1 + msg_2 + msg_3
        tts = gtts.gTTS(welcome_msg, lang='en')
        tts.save('read_this.mp3')
        playsound('read_this.mp3')
        os.remove('read_this.mp3')

        self.voice_services_init()

        # PUBLISHERS
        self.text_publisher = self.create_publisher(String, '/text', 10)
        self.intention_publisher = self.create_publisher(String,
                                                         '/intention', 10)

    def voice_services_init(self):
        FORMAT = pyaudio.paInt16
        CHANNELS = 1
        RATE = 16000
        self.CHUNK = int(RATE/10)
        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(format=FORMAT, channels=CHANNELS,
                                      rate=RATE, input=True,
                                      frames_per_buffer=self.CHUNK,
                                      stream_callback=self._get_data)
        self._buffer = queue.Queue()
        self.closed = False

    def _get_data(self, in_data, frame_count, time_info, status):
        self._buffer.put(in_data)
        return None, pyaudio.paContinue

    def _generator(self):
        while not self.closed:
            chunk = self._buffer.get()
            if chunk is None:
                return
            data = [chunk]

            while True:
                try:
                    chunk = self._buffer.get(block=False)
                    if chunk is None:
                        return
                    data.append(chunk)
                except queue.Empty:
                    break

            yield b''.join(data)

    def _listen_print_loop(self, responses):
        for response in responses:
            if not response.results:
                continue
            result = response.results[0]
            if not result.alternatives:
                continue

            transcript = result.alternatives[0].transcript

            if result.is_final:
                self.get_logger().info("Result: {}".format(result))
                transcript = transcript.encode('utf-8')
                transcript = transcript.strip().lower()
                if transcript.lower() == u'exit':
                    rclpy.shutdown()
                msg = String()
                msg.data = transcript.decode()
                try:
                    self.text_publisher.publish(msg)
                except Exception:
                    pass
                # CHECK IF DATA CONTAINS GIVEN OBJECTS
                obj = ['mug', 'laptop', 'book']
                text_output = transcript.decode().split(' ')
                intention_check = False
                for i in obj:
                    for j in text_output:
                        if i == j:
                            intention_check = True
                            temp_str = "OK, I will try to find a " + j
                            tts = gtts.gTTS(temp_str, lang='en')
                            tts.save('read_this.mp3')
                            playsound('read_this.mp3')
                            os.remove('read_this.mp3')
                            msg = String()
                            msg.data = j
                            self.get_logger().info("Got {}".format(j))
                            self.intention_publisher.publish(msg)
                            self.shutdown()
                if not intention_check:
                    temp_str = "Sorry, I couldn't understand!"
                    self.get_logger().info(temp_str)
                    tts = gtts.gTTS(temp_str, lang='en')
                    tts.save('read_this.mp3')
                    playsound('read_this.mp3')
                    os.remove('read_this.mp3')
                    self.shutdown()

    def google_speech_client(self):
        language_code = 'en-US'
        client = speech.SpeechClient()
        config = speech.RecognitionConfig(
            encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
            sample_rate_hertz=16000,
            language_code=language_code)
        streaming_config = speech.StreamingRecognitionConfig(
            config=config,
            interim_results=True,
            single_utterance=True)
        requests = (speech.StreamingRecognizeRequest(
            audio_content=content) for content in self._generator())
        responses = client.streaming_recognize(streaming_config, requests)
        self._listen_print_loop(responses)

    def shutdown(self):
        self.get_logger().info("Respawning")
        self.closed = True
        self._buffer.put(None)
        self.stream.close()
        self.audio.terminate()
        self.destroy_node()
        exit()

    def start_client(self):
        try:
            self.get_logger().info("Starting Google Speech client")
            self.closed = False
            self.google_speech_client()
        except KeyboardInterrupt:
            self.shutdown()


def main(args=None):
    rclpy.init(args=None)
    node = Voice_Control()
    node.start_client()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
