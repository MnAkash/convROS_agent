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
from std_msgs.msg import String
from RealtimeSTT import AudioToTextRecorder
from gtts import gTTS
import pyaudio
import threading




class TTS:
    def __init__(self, display_pub):
        """ Initialize the TTS system with ZeroMQ messaging. """
        self.display_pub = display_pub
    def speak(self, text):
        """ Synchronous method to synthesize and play speech. """
        print("Synthesizing speech...")

        # Block listening before speaking
        self.display_pub.publish(String(data="0"))
        print("Sent ZeroMQ message: 0 (Blocking listening)")

        # Generate speech file
        audio_file = self.create_mp3_from_text_gTTS(text)

        # Play audio (Blocking until done)
        os.system(f'mpg321 {audio_file} > /dev/null 2>&1')

        print("Speech synthesis completed")

        # Unblock listening after speaking
        self.display_pub.publish(String(data="1"))
        print("Sent ZeroMQ message: 1 (Listening can start)")

    @functools.cache
    def create_mp3_from_text_gTTS(self, text):
        """ gTTS Speech Synthesis """
        tts = gTTS(text=text, lang='en', slow=False)
        with tempfile.NamedTemporaryFile(suffix='.mp3', delete=False) as f:
            mp3filename = f.name
            tts.save(mp3filename)
        return mp3filename

class SpeechListener:
    def __init__(self, on_vad_detect_start=None, on_vad_detect_stop=None, on_recording_start=None, on_recording_stop=None, mic_index=5):
        """Initialize the Speech Listener."""
        self.recorder = AudioToTextRecorder(
            model="base.en",
            input_device_index=mic_index,
            compute_type="float32",
            spinner=True, 
            min_gap_between_recordings=0.5, 
            silero_sensitivity=0.85,  
            webrtc_sensitivity=2,
            min_length_of_recording=0.5,
            on_vad_detect_start = on_vad_detect_start,
            on_vad_detect_stop = on_vad_detect_stop,
            on_recording_start= on_recording_start,
            on_recording_stop= on_recording_stop
        )


    def listen_for_duration(self, duration=10):
        """ Listens for a fixed duration, transcribes the audio, and returns text. """
        self.recorder.start()
        time.sleep(duration)
        self.recorder.stop()
        self.recorder.wait_audio()
        return self.recorder.transcribe() or ""
    
    def transcribe_with_timeout(self, timeout=10.0):
        result = {"text": ""}

        def worker():
            try:
                result["text"] = self.recorder.text() or ""
            except Exception as e:
                print(f"Error in recorder.text(): {e}")

        t = threading.Thread(target=worker, daemon=True)
        t.start()
        t.join(timeout)

        if t.is_alive():
            # Timed out: optionally try to abort
            try:
                self.recorder.abort()
            except Exception:
                pass
            # Do not wait forever
            return ""

        return result["text"]



class SpeechRecognitionActionServer(Node):
    def __init__(self):
        super().__init__('question_response_action_node')
        
        self._action_server = ActionServer(
            self,
            QuestionResponseRequest,
            'question_response_action',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.display_pub = self.create_publisher(String, 'display_tx', 10)
        

        mic_index = self.get_aec_microphone_index(source_substring='default')
        self.listener = SpeechListener(on_vad_detect_start= self.on_vad_detect_start,
                                       on_vad_detect_stop = self.on_vad_detect_stop,
                                       on_recording_start=self.on_recording_start,
                                       on_recording_stop=self.on_recording_stop,
                                       mic_index=mic_index)
        
        self.tts = TTS(self.display_pub)

        self.text = ""
        # self.vad_start = False
        self.vad_stop = False
        self.recording_start = False
        self.recording_start_time = None

    def get_aec_microphone_index(self, source_substring='default'):
        # AEC (Acoustic Echo Cancellation) microphone index detection
        p = pyaudio.PyAudio()
        aec_index = None
        for i in range(p.get_device_count()):
            info = p.get_device_info_by_index(i)
            name = info['name'].lower().strip()
            print(f"Device {i}: {name} (Input Channels: {info['maxInputChannels']})")
            if info['maxInputChannels'] > 0:  # Only list input devices
                if source_substring == name or 'echo' in name:
                    aec_index = i
                    break
        if aec_index is None:
            raise RuntimeError("Couldn't find your aec_source in pyaudio devices")
        
        # print(f"Using source {name} with index {aec_index}\n\n")
        self.get_logger().info(f"Using source {name} with index {aec_index}\n\n")
        
        return aec_index
    
    def process_text(self, text):
        self.text = text.lower()
        self.get_logger().info(f"I hear: {self.text}")

    def on_vad_detect_start(self):
        self.vad_stop = False
        self.get_logger().info("Voice activity detected.")

    def on_recording_start(self):
        self.recording_start = True
        self.recording_start_time = time.time()
        self.get_logger().info("Recording Started.")
    
    def on_recording_stop(self):
        self.recording_start = False
        self.recording_start = False
        self.recording_start_time = None
        self.get_logger().info("Recording Stopped")


    def on_vad_detect_stop(self):
        self.vad_stop = True
        self.get_logger().info("Voice activity stopped.")

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
            self.display_pub.publish(String(data="2"))
            print("Sent ZeroMQ message: 2 (Listening started)")
            
            # text = self.listener.listen_for_duration(duration=10).lower()
            # self.listener.recorder.text(self.process_text)
            text = self.listener.transcribe_with_timeout(timeout=10.0).lower()
            self.process_text(text)

            
            # listenning_start_time = time.time()
            # while True:
            #     if self.recording_start:
            #         time_elasped = time.time() - self.recording_start_time
            #         if time_elasped > 2 or self.vad_stop:
            #             self.listener.recorder.stop()
            #             self.listener.recorder.wait_audio()
            #             self.text = self.listener.recorder.transcribe() or ""
            #             break
            #     if time.time() - listenning_start_time > 10:
            #         self.listener.recorder.stop()
            #         # self.listener.recorder.abort()
            #         self.listener.recorder.wait_audio()
            #         break

            # self.recording_start = False
            
            self.display_pub.publish(String(data="1"))
            print("Sent ZeroMQ message: 1 (Listening stopped)")

            if "yes" in self.text:
                result.response = "yes"
                goal_handle.succeed()
                return result
            elif "no" in self.text:
                result.response = "no"
                goal_handle.succeed()
                return result
            
            time.sleep(2)

        result.response = "no"
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = SpeechRecognitionActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()