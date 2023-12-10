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
# Python file for a ROS node that sets and gets the status of the GPT operation using ROS parameters.
# Uses a GPTStatus class to define possible statuses for the GPT operation.
# Uses a GPTStatusOperation class to get and set the GPT status value using subprocess calls.
# Defines a GPTParamServer class that declares a ROS parameter for the current GPT status,
# initializes the GPT status as "WAITING_USER_INPUT", and creates a timer to periodically check the GPT status.
# Defines a timer callback function that logs any changes in the GPT status.
# Includes a main function that initializes the ROS node, spins the GPTParamServer node, and shuts down the node.
#
# Author: Herman Ye

import rclpy
from rclpy.node import Node
from enum import Enum
import subprocess


class GPTStatus(Enum):
    WAITING_USER_INPUT = 1
    SPEECH_TO_TEXT_PROCESSING = 2
    GPT_PROCESSING = 3
    TEXT_TO_SPEECH_PROCESSING = 4
    ROBOT_ACTION = 5


class GPTStatusOperation:
    def get_gpt_status_value(self):
        cmd = "ros2 param get /gpt/gpt_param_server gpt_status"
        try:
            output = subprocess.check_output(cmd, shell=True, text=True)
            return output.split("String value is: ")[1].strip()
        except subprocess.CalledProcessError as e:
            print(f"Error when getting gpt_status: {e}")
            return None

    def set_gpt_status_value(self, gpt_status_value):
        command = f"ros2 param set /gpt/gpt_param_server gpt_status {gpt_status_value}"
        subprocess.run(command, shell=True, check=True, text=True)


class GPTParamServer(Node):
    def __init__(self):
        super().__init__("param_server", namespace="gpt")
        self.gpt_status_value = (
            GPTStatus.WAITING_USER_INPUT.name
        )  # Init status
        self.declare_parameter("gpt_status", self.gpt_status_value)
        self.create_timer(0.5, self.timer_callback)
        self.last_gpt_status_value = self.gpt_status_value

    def timer_callback(self):
        current_gpt_status_value = self.get_parameter("gpt_status").value
        if current_gpt_status_value != self.last_gpt_status_value:
            self.last_gpt_status_value = current_gpt_status_value
            self.get_logger().info(
                'GPT status: "%s"' % current_gpt_status_value
            )


def main(args=None):
    rclpy.init(args=args)
    gpt_param_server = GPTParamServer()
    rclpy.spin(gpt_param_server)
    gpt_param_server.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
