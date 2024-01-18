#!/usr/bin/env python3

# this is a mini-pupper-specific launch file for bringing up the OpenAI and other related services.
# Original Author: Daniel Walmsley, Mini Pupper project for MangDang, but modified by me (Dan Walmsley)

# This launch file is a part of OPENAI_ROS2 project developed to control and interact with the Mini Pupper robot or your own robot.
# The launch file contains a LaunchDescription object which defines the ROS2 nodes to be executed.
# The following nodes are defined:
# - railbot_param_server: A node for managing various parameters and configurations of the OPENAI_ROS2 project.
# - gpt_service: A node responsible for processing and handling user requests, and interacting with the GPT models.
# - audio_output: A node for controlling the audio output of the Mini Pupper or your own robot based on the user interaction.
# - audio_input: A node for handling and processing audio input from the Mini Pupper or your own robot, such as voice commands or environmental sounds.
# - gpt_robot: A node for controlling the actuators and sensors of the Mini Pupper or your own robot, and managing its overall behavior.
# The real_hardware parameter is used to determine if the code is run on a Mini Pupper, which affects some settings and behaviors.
# When set to True, the launch file is configured for the Mini Pupper platform.
#
# To use the OPENAI_ROS2 with mini pupper, run `ros2 launch railbot_bringup mini_pupper_launch.py real_hardware:=True`
#


from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="real_hardware",
                default_value="True",
                description="Set railbot to run on Mini Pupper",
            ),
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
                executable="gpt_service",
                name="gpt_service",
                output="screen",
            ),
            Node(
                package="gpt_audio",
                namespace="railbot",
                executable="audio_output",
                name="audio_output",
                output="screen",
                parameters=[
                    {"real_hardware": LaunchConfiguration("real_hardware")}
                ],
            ),
            Node(
                package="gpt_audio",
                namespace="railbot",
                executable="audio_input",
                name="audio_input",
                output="screen",
                parameters=[
                    {"real_hardware": LaunchConfiguration("real_hardware")}
                ],
            ),
            Node(
                package="gpt_robot",
                namespace="railbot",
                executable="gpt_robot",
                name="gpt_robot",
                output="screen",
                parameters=[
                    {"real_hardware": LaunchConfiguration("real_hardware")}
                ],
            ),
        ]
    )
