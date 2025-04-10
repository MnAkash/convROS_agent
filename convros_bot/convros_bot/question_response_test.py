import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
import time
import zmq
from std_msgs.msg import String
from shr_msgs.action import QuestionResponseRequest
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import GoalResponse, CancelResponse
from convros_bot.speechRecognizer_test import SpeechRecognizer
from convros_bot.tts_test import TTS

class QuestionResponseActionServer(Node):
    def __init__(self, zmq_socket):
        super().__init__('question_response_action_node')

        # ROS 2 Action Server
        self._action_server = ActionServer(
            self,
            QuestionResponseRequest,
            'question_response_action',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        # Use the provided ZeroMQ socket
        self.zmq_socket = zmq_socket

        # Initialize Speech Recognizer and TTS
        self.speech_recognizer = SpeechRecognizer()
        self.tts = TTS(self.zmq_socket)

        self.MAX_ATTEMPTS = 3  # Maximum retries for unanswered questions

    def goal_callback(self, goal_handle):
        self.get_logger().info("✅ Received a new question request.")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("🚫 Request to cancel received.")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """ Execute the action: Speak, listen, and check for 'yes' or 'no' """
        question = goal_handle.request.question
        attempt = 0
        response = None

        while attempt < self.MAX_ATTEMPTS:
            self.get_logger().info(f"🗣️ Attempt {attempt + 1}: Asking '{question}'")
            self.tts.speak(question)  # ✅ Speak synchronously

            self.get_logger().info("🎤 Listening for response (10s)...")
            response = self.listen_for_response()  # ✅ Listen for response

            if response in ["yes", "no"]:
                self.get_logger().info(f"✅ Detected Response: {response.upper()}")
                goal_handle.succeed()

                result = QuestionResponseRequest.Result()
                result.response = response
                return result

            self.get_logger().warning(f"⚠️ No clear 'yes' or 'no' detected. Retrying... ({attempt + 1}/{self.MAX_ATTEMPTS})")
            attempt += 1

        self.get_logger().error("❌ No valid response after 3 attempts. Failing the goal.")
        goal_handle.abort()
        
        result = QuestionResponseRequest.Result()
        result.response = "No response detected."
        return result

    def listen_for_response(self):
        """ Listens for speech and checks for 'yes' or 'no' in the response """
        response_text = None
        response_received = False

        def callback(text):
            nonlocal response_text, response_received
            response_text = text.lower()
            response_received = True
            
        self.zmq_socket.send_string("2")
        self.get_logger().info("📡 Sent ZMQ message: 2 (Listening)")

        self.speech_recognizer.listen(callback)  # ✅ Start listening
        
        self.zmq_socket.send_string("1")
        self.get_logger().info("📡 Sent ZMQ message: 1 (Listening Ended)")

        timeout = time.time() + 10  # Listen for 10 seconds
        while time.time() < timeout:
            if response_received:
                self.speech_recognizer.stop()  # ✅ Stop listening early if we got a response
                break
            time.sleep(0.1)  # Prevent high CPU usage

        if response_text:
            if "yes" in response_text:
                return "yes"
            elif "no" in response_text:
                return "no"

        return "unkown"  # ✅ Return "no" if neither "yes" nor "no" detected

def main(args=None):
    rclpy.init(args=args)
    zmq_context = zmq.Context()
    zmq_socket = zmq_context.socket(zmq.PUB)
    zmq_socket.bind("tcp://*:5555")

    node = QuestionResponseActionServer(zmq_socket)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
