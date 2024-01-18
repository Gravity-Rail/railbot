#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from railbot_status.railbot_param_server import RailbotStatus, RailbotStatusOperation
from railbot_status.railbot_config import RailbotConfig
from openai import OpenAI
import os
import tempfile

config = RailbotConfig()
client = OpenAI(api_key=config.api_key)


class AudioOutput(Node):
    def __init__(self):
        super().__init__("audio_output", namespace="railbot")
        self.subscription = self.create_subscription(
            String, "chat_llm_text_output", self.text_callback, 10
        )
        self.get_logger().info("Initialized OpenAI TTS")
        self.railbot_operation = RailbotStatusOperation()

    def text_callback(self, msg):
        self.get_logger().info("Received text: '%s'" % msg.data)
        self.railbot_operation.set_railbot_status_value(
            RailbotStatus.TTS_PROCESSING.name
        )
        response = client.audio.speech.create(
            model="tts-1",
            voice="alloy",
            input=msg.data,
        )
        self.railbot_operation.set_railbot_status_value(RailbotStatus.ROBOT_ACTION.name)

        with tempfile.NamedTemporaryFile(suffix=".mp3") as temp_mp3:
            response.stream_to_file(temp_mp3.name)
            os.system("mpv" + " " + temp_mp3.name)

        self.get_logger().info("Finished OpenAI TTS playing.")


def main(args=None):
    rclpy.init(args=args)

    audio_output = AudioOutput()

    rclpy.spin(audio_output)

    audio_output.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
