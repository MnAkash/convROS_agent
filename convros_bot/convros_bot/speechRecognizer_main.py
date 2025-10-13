'''
This is the class of RealtimeSTT which uses Whisper model to detect real-time speech.
'''

from RealtimeSTT import AudioToTextRecorder
import speech_recognition as sr
import os

# set output and input device by following command:
# pactl set-default-sink alsa_output.usb-Generic_USB2.0_Device_20121120222016-00.analog-stereo
# pactl set-default-source alsa_input.usb-SEEED_ReSpeaker_4_Mic_Array__UAC1.0_-00.multichannel-input
os.environ["PULSE_SINK"] = "alsa_output.usb-Generic_USB2.0_Device_20121120222016-00.analog-stereo"
os.environ["PULSE_SOURCE"] = "alsa_input.usb-SEEED_ReSpeaker_4_Mic_Array__UAC1.0_-00.multichannel-input"

for index, name in enumerate(sr.Microphone.list_microphone_names()):
    print(f"Microphone with index {index}: {name}")
    if "ReSpeaker" in name:
        print(f"Using {index}: {name} as input device")
        break


class speechRecognizer():
    def __init__(self):
        # print("Wait until it says 'speak now'")
        print("Intializing speech recognizer")
        self.recorder = AudioToTextRecorder(model = 'base.en', input_device_index=index, spinner=True, 
                                       min_gap_between_recordings=0.25, silero_sensitivity=0.8, webrtc_sensitivity=3,
                                       min_length_of_recording = 1.0)
        # self.recorder = AudioToTextRecorder(model = 'base.en', input_device_index=index, spinner=True, 
        #                                min_gap_between_recordings=0.5, silero_sensitivity=0.8, webrtc_sensitivity=2,
        #                                min_length_of_recording = 0.5)
    def process_text(self, text):
        print(text)
    
    def stop(self):
        self.recorder.abort()
        self.recorder.stop()
        self.recorder.interrupt_stop_event.set()

def main():
    recognizer =  speechRecognizer()

    while True:
        recognizer.recorder.text(recognizer.process_text)


if __name__ == '__main__':
    main()

# silero_sensitivity (float, default=0.6): Sensitivity for Silero's voice activity detection ranging from 0 (least sensitive) to 1 (most sensitive). Default is 0.6.
# webrtc_sensitivity (int, default=3): Sensitivity for the WebRTC Voice Activity Detection engine ranging from 0 (least aggressive / most sensitive) to 3 (most aggressive, least sensitive). Default is 3.
# min_length_of_recording (float, default=1.0): Specifies the minimum duration in seconds that a recording session should last to ensure meaningful audio capture, preventing excessively short or fragmented recordings.