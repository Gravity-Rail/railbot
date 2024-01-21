#!/usr/bin/env python3

# This is a generic launch file for a raspberry pi with mic/speaker bringing up the Chat LLM service, TTS/STT services and other related services.

# - railbot_param_server: Manages configuration parameters
# - chat_llm_service: A node responsible to integrating with Chat LLMs via LangChain
# - audio_output: A node that turns text into audio output
# - audio_input: A node that listens for audio input and feeds the text to the LLM
# - mini_pupper_robot: A node for controlling the hardware of the Mini Pupper 2


from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription(
        [
            Node(
                package="railbot_status",
                namespace="railbot",
                executable="railbot_param_server",
                name="railbot_param_server",
                output="screen",
            ),
            Node(
                package="railbot_main",
                namespace="railbot",
                executable="chat_llm_service",
                name="chat_llm_service",
                output="screen",
            ),
            # Node(
            #     package="openai_audio",
            #     namespace="railbot",
            #     executable="audio_output",
            #     name="audio_output",
            #     output="screen",
            # ),
            # Node(
            #     package="openai_audio",
            #     namespace="railbot",
            #     executable="audio_input",
            #     name="audio_input",
            #     output="screen",
            # ),
            Node(
                package="railbot_cam",
                namespace="railbot",
                executable="img_publisher",
                name="img_publisher",
                output="screen",
            ),
            Node(
                package="railbot_cam",
                namespace="railbot",
                executable="img_subscriber",
                name="img_subscriber",
                output="screen",
            )
        ]
    )
