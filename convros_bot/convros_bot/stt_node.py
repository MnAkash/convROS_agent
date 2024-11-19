import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from RealtimeSTT import AudioToTextRecorder
import os

class STTNode(Node):
    def __init__(self):
        super().__init__('stt_node')
        self.msg = String()
        self.publisher_ = self.create_publisher(String, 'speech_text', 10)
        self.recorder = AudioToTextRecorder(model = 'base.en', input_device_index=0, spinner=True, min_gap_between_recordings=1.0)
        self.timer = self.create_timer(0.05, self.listen_and_publish)

    def publish_text(self, text):
        self.get_logger().info(f'Recognized: {text}')
        self.msg.data = text
        self.publisher_.publish(self.msg)

    def listen_and_publish(self):
        try:
            self.recorder.text(self.publish_text)
                
        except Exception as e:
            self.get_logger().error(f'An Error occurred: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = STTNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down STT Node')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()