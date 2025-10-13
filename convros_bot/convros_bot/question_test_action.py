import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
import time
import os
import zmq
import tempfile
import functools
from std_msgs.msg import String
from shr_msgs.action import QuestionResponseRequest
from rclpy.action import GoalResponse, CancelResponse
from RealtimeSTT import AudioToTextRecorder
from gtts import gTTS

import speech_recognition as sr
import os

# set output and input device by following command:
os.environ["PULSE_SINK"] = "alsa_output.usb-Generic_USB2.0_Device_20121120222016-00.analog-stereo"
os.environ["PULSE_SOURCE"] = "alsa_input.usb-SEEED_ReSpeaker_4_Mic_Array__UAC1.0_-00.multichannel-input"

class TTS:
    def __init__(self, zmq_socket):
        """ Initialize the TTS system with ZeroMQ messaging. """
        self.zmq_socket = zmq_socket

    def speak(self, text):
        """ Synchronous method to synthesize and play speech. """
        print("🗣️ Synthesizing speech...")

        # ✅ Block listening before speaking
        self.zmq_socket.send_string("0")
        print("✅ Sent ZeroMQ message: 0 (Blocking listening)")

        # ✅ Generate speech file
        audio_file = self.create_mp3_from_text_gTTS(text)

        # ✅ Play audio (Blocking until done)
        os.system(f'mpg321 {audio_file} > /dev/null 2>&1')

        print("✅ Speech synthesis completed")

        # ✅ Unblock listening after speaking
        self.zmq_socket.send_string("1")
        print("✅ Sent ZeroMQ message: 1 (Listening can start)")

    @functools.cache
    def create_mp3_from_text_gTTS(self, text):
        """ gTTS Speech Synthesis """
        tts = gTTS(text=text, lang='en', slow=False)
        with tempfile.NamedTemporaryFile(suffix='.mp3', delete=False) as f:
            mp3filename = f.name
            tts.save(mp3filename)
        return mp3filename

class SpeechListener:
    def __init__(self, mic_index=5):
        """Initialize the Speech Listener."""
        self.recorder = AudioToTextRecorder(
            model="base.en",
            input_device_index=mic_index,
            compute_type="float32",
            spinner=True, 
            min_gap_between_recordings=0.25, 
            silero_sensitivity=0.95,  
            webrtc_sensitivity=3,
            min_length_of_recording=1.0
        )

    def listen_for_duration(self, duration=10):
        """ Listens for a fixed duration, transcribes the audio, and returns text. """
        self.recorder.start()
        time.sleep(duration)
        self.recorder.stop()
        self.recorder.wait_audio()
        return self.recorder.transcribe() or ""

class SpeechRecognitionActionServer(Node):
    def __init__(self, zmq_socket):
        super().__init__('question_response_action_node')
        
        #find the microphone index for ReSpeaker
        for index, name in enumerate(sr.Microphone.list_microphone_names()):
            print(f"Microphone with index {index}: {name}")
            if "ReSpeaker" in name:
                print(f"Using {index}: {name} as input device")
                break


        self._action_server = ActionServer(
            self,
            QuestionResponseRequest,
            'question_response_action',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        self.zmq_socket = zmq_socket  # ✅ Bind ZMQ socket
        self.listener = SpeechListener(mic_index=index)
        
        self.tts = TTS(self.zmq_socket)

    def goal_callback(self, goal_request):
        self.get_logger().info("Received speech recognition goal request.")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Received cancel request.")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info("Executing speech recognition...")
        question = goal_handle.request.question
        result = QuestionResponseRequest.Result()

        for i in range(3):
            self.get_logger().info(f"Iteration {i+1}/3: Speaking and Listening")
            self.tts.speak(question if i == 0 else "Sorry, I did not hear that. " + question)
            self.zmq_socket.send_string("2")  # Publish 2 before listening
            print("Sent ZeroMQ message: 2 (Listening started)")
            text = self.listener.listen_for_duration(duration=10).lower()
            self.zmq_socket.send_string("1")  # Publish 1 after listening
            print("Sent ZeroMQ message: 1 (Listening stopped)")

            if "yes" in text:
                result.response = "yes"
                goal_handle.succeed()
                return result
            elif "no" in text:
                result.response = "no"
                goal_handle.succeed()
                return result
            
            time.sleep(2)

        result.response = "no"
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    zmq_context = zmq.Context()
    zmq_socket = zmq_context.socket(zmq.PUB)
    zmq_socket.bind("tcp://*:5555")
    node = SpeechRecognitionActionServer(zmq_socket)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
