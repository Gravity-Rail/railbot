#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# flake8: noqa
#
# Copyright 2023 MangDang
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Description:
# This launch file is a part of gpt4_ros2 project developed to control and interact with the Mini Pupper robot or your own robot.
# The launch file contains a LaunchDescription object which defines the ROS2 nodes to be executed.
# The following nodes are defined:
# - gpt_param_server: A node for managing various parameters and configurations of the GPT4_ROS2 project.
# - gpt_service: A node responsible for processing and handling user requests, and interacting with the GPT models.
# - audio_output: A node for controlling the audio output of the Mini Pupper or your own robot based on the user interaction.
# - audio_input: A node for handling and processing audio input from the Mini Pupper or your own robot, such as voice commands or environmental sounds.
# - gpt_robot: A node for controlling the actuators and sensors of the Mini Pupper or your own robot, and managing its overall behavior.
# The mini_pupper parameter is used to determine if the code is run on a Mini Pupper, which affects some settings and behaviors.
# When set to True, the launch file is configured for the Mini Pupper platform.
#
# To use the GPT4_ROS2 with mini pupper, run `ros2 launch gpt_bringup gpt_bringup_launch.py mini_pupper:=True`
#
# Author: Herman Ye

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="mini_pupper",
                default_value="True",
                description="Set GPT4_ROS2 to run on Mini Pupper",
            ),
            Node(
                package="gpt_status",
                namespace="gpt",
                executable="gpt_param_server",
                name="gpt_param_server",
                output="screen",
            ),
            Node(
                package="gpt_main",
                namespace="gpt",
                executable="gpt_service",
                name="gpt_service",
                output="screen",
            ),
            Node(
                package="gpt_audio",
                namespace="gpt",
                executable="audio_output",
                name="audio_output",
                output="screen",
                parameters=[
                    {"mini_pupper": LaunchConfiguration("mini_pupper")}
                ],
            ),
            Node(
                package="gpt_audio",
                namespace="gpt",
                executable="audio_input",
                name="audio_input",
                output="screen",
                parameters=[
                    {"mini_pupper": LaunchConfiguration("mini_pupper")}
                ],
            ),
            Node(
                package="gpt_robot",
                namespace="gpt",
                executable="gpt_robot",
                name="gpt_robot",
                output="screen",
                parameters=[
                    {"mini_pupper": LaunchConfiguration("mini_pupper")}
                ],
            ),
        ]
    )
