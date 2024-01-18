#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from openai import OpenAI

from pydub import AudioSegment
import sounddevice as sd
from scipy.io.wavfile import write

from railbot_status.railbot_param_server import RailbotStatus, RailbotStatusOperation
from railbot_status.railbot_config import RailbotConfig

import os
import tempfile


config = RailbotConfig()

client = OpenAI(api_key=config.api_key)

class AudioInput(Node):
    def __init__(self):
        super().__init__("audio_input", namespace="railbot")
        self.publisher = self.create_publisher(
            String, "chat_llm_text_input", 10
        )
        self.create_timer(1, self.run_audio_input_callback)

        self.volume_gain_multiplier = config.volume_gain_multiplier
        # check if the amixer command is available
        if os.system("which amixer") != 0:
            os.system("amixer -c 0 sset 'Headphone' 100%")

        self.railbot_operation = RailbotStatusOperation()
        self.get_logger().info("Initialized OpenAI Audio Transcription")

    def run_audio_input_callback(self):
        gpt_current_status_value = self.railbot_operation.get_railbot_status_value()
        if gpt_current_status_value == RailbotStatus.WAITING_USER_INPUT.name:
            self.run_audio_input()

    def run_audio_input(self):
        self.get_logger().info("Recording Audio...")
        audio_data = sd.rec( int(config.duration * config.sample_rate), samplerate=config.sample_rate, channels=1 )
        sd.wait()  # Wait until recording is finished

        # optional volume multiplier
        audio_data *= self.volume_gain_multiplier

        self.get_logger().info("Audio Recording Complete")

        self.railbot_operation.set_railbot_status_value(
            RailbotStatus.STT_PROCESSING.name
        )

        # save as WAV using tempfile
        with tempfile.NamedTemporaryFile(suffix=".wav") as temp_wav:
            write(temp_wav.name, config.sample_rate, audio_data)

            # Step 2: Convert WAV to MP3 with pydub
            with tempfile.NamedTemporaryFile(suffix=".mp3") as temp_mp3:
                audio = AudioSegment.from_wav(temp_wav.name)
                audio.export(temp_mp3.name, format='mp3')

                with open(temp_mp3.name, "rb") as f:
                    # audio_bytes = f.read()
                    transcript = client.audio.transcriptions.create(
                        model="whisper-1",
                        file=f
                    )

                    msg = String()
                    msg.data = transcript.text
                    self.publisher.publish(msg)
                    self.get_logger().info(
                        "Audio Input Node publishing: \n'%s'" % msg.data
                    )

def main(args=None):
    rclpy.init(args=args)

    audio_input = AudioInput()

    rclpy.spin(audio_input)

    audio_input.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
