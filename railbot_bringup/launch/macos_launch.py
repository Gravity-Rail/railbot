#!/usr/bin/env python3

# This is a mini-pupper-specific launch file for bringing up the Chat LLM service, TTS/STT services and other related services.

# - railbot_param_server: Manages configuration parameters
# - chat_llm_service: A node responsible to integrating with Chat LLMs via LangChain
# - audio_output: A node that turns text into audio output
# - audio_input: A node that listens for audio input and feeds the text to the LLM
# - mini_pupper_robot: A node for controlling the hardware of the Mini Pupper 2


from launch import LaunchDescription
from launch_ros.actions import Node
from whisper_server_launch_mixin import WhisperServerMixin

def generate_launch_description():

    ld = LaunchDescription(
        [
            Node(
                package="railbot_status",
                namespace="railbot",
                executable="railbot_param_server",
                name="railbot_param_server",
                output="screen",
            ),
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
                executable="img_identifier",
                name="img_identifier",
                output="screen",
            ),
            Node(
                package="audio_listener",
                namespace="railbot",
                executable="audio_listener",
                output="screen",
            )
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
            # Node(
            #     package="mini_pupper_robot",
            #     namespace="railbot",
            #     executable="mini_pupper_robot",
            #     name="mini_pupper_robot",
            #     output="screen",
            # ),
        ]
    )

    # launch whisper
    ld.add_action(WhisperServerMixin.arg_model_name())
    ld.add_action(WhisperServerMixin.arg_n_threads())
    ld.add_action(WhisperServerMixin.arg_language())
    ld.add_action(WhisperServerMixin.arg_use_gpu())
    ld.add_action(WhisperServerMixin.arg_batch_capacity())
    ld.add_action(WhisperServerMixin.arg_buffer_capacity())
    ld.add_action(WhisperServerMixin.arg_carry_over_capacity())
    ld.add_action(
        WhisperServerMixin.composable_node_container(
            composable_node_descriptions=[
                WhisperServerMixin.composable_node_inference(
                    parameters=[
                        WhisperServerMixin.param_model_name(),
                        WhisperServerMixin.param_n_threads(),
                        WhisperServerMixin.param_language(),
                        WhisperServerMixin.param_use_gpu(),
                        WhisperServerMixin.param_batch_capacity(),
                        WhisperServerMixin.param_buffer_capacity(),
                        WhisperServerMixin.param_carry_over_capacity(),
                    ],
                    remappings=[("/whisper/audio", "/audio_listener/audio")],
                    namespace="railbot",
                ),
            ]
        )
    )

    return ld
