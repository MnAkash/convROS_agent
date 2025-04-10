from RealtimeSTT import AudioToTextRecorder
import time

class SpeechRecognizer:
    def __init__(self, mic_index=5, timeout=10):
        """Initialize the Speech Recognizer with a specific microphone."""
        print("🚀 Initializing Speech Recognizer...")
        self.recorder = AudioToTextRecorder(
            model='base.en',
            input_device_index=mic_index,  # ✅ Set fixed microphone index
            spinner=True, 
            min_gap_between_recordings=0.5, 
            silero_sensitivity=0.8, 
            webrtc_sensitivity=2,
            min_length_of_recording=0.5
        )
        self.timeout = timeout  # ✅ Add timeout for listening
        self.running = False  # ✅ Flag to track if listening is active

    def process_text(self, text):
        """Processes the recognized text."""
        print(f"✅ Recognized: {text}")

    def listen(self, callback=None):
        """Starts listening and sends recognized text to the callback function."""
        print("🎤 Listening for speech...")
        start_time = time.time()
        self.running = True

        while self.running and time.time() - start_time < self.timeout:
            # ✅ Directly call self.recorder.text() inside the loop
            text = self.recorder.text()
            if text:
                print(f"✅ Recognized: {text}")
                if callback:
                    callback(text)
                self.stop()  # ✅ Stop automatically after recognition
                return  # ✅ Exit immediately after recognition
            
            if self.recorder.interrupt_stop_event.is_set():
                print("🛑 Stopping listening due to user request...")
                break
            
            time.sleep(0.1)  # ✅ Prevent CPU overload

        print("⏳ Listening timeout reached.")
        self.stop()  # ✅ Ensure it stops after timeout

    def stop(self):
        """Stops the recorder safely."""
        if self.running:
            print("🛑 Stopping the recorder...")
            self.running = False
            self.recorder.stop()
            self.recorder.interrupt_stop_event.set()
