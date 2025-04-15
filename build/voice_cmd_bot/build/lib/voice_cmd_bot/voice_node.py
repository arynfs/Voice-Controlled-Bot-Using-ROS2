import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import queue
import sounddevice as sd
import vosk
import json
import os

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')
        self.publisher_ = self.create_publisher(String, 'voice_commands', 10)
        self.q = queue.Queue()

        # FIXED: Expand user path correctly
        model_path = os.path.expanduser('~/voice_ws/models/vosk-model-small-en-us-0.15')
        self.model = vosk.Model(model_path)

        self.samplerate = 16000
        self.get_logger().info("VoiceCommandNode started. Speak...")

        try:
            self.stream = sd.RawInputStream(
                samplerate=self.samplerate,
                blocksize=8000,
                dtype='int16',
                channels=1,
                callback=self.callback
            )
            self.stream.start()
        except Exception as e:
            self.get_logger().error(f"Failed to start audio stream: {e}")
            raise

        self.listen()

    def callback(self, indata, frames, time, status):
        if status:
            self.get_logger().warning(str(status))
        self.q.put(bytes(indata))

    def listen(self):
        rec = vosk.KaldiRecognizer(self.model, self.samplerate)
        while rclpy.ok():
            data = self.q.get()
            if rec.AcceptWaveform(data):
                result = json.loads(rec.Result())
                command = result.get("text", "")
                if command:
                    self.get_logger().info(f"Recognized: {command}")
                    msg = String()
                    msg.data = command
                    self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
