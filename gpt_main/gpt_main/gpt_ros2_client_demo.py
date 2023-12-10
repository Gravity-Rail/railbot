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
# This code defines a ROS2 client node that communicates with a GPTText service.
# The client node reads user input as a text prompt, sends it to the GPTText service and retrieves a response text from it.
# The retrieved response is then displayed to the user as the "Mini Pupper" reply.
# The client node continuously reads user input and sends requests to the GPTText service until terminated.
# This example script demonstrates the usage of a custom ROS2 service to communicate with GPT-3.5-turbo or GPT-4 and integrate it into a ROS2-based system.
#
# Author: Herman Ye


import rclpy
from rclpy.node import Node
from gpt_interfaces.srv import GPTText


class GPTClient(Node):
    def __init__(self):
        super().__init__("gpt_ros2_client")
        self.cli = self.create_client(GPTText, "GPT_service")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.req = GPTText.Request()
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("GPT Client is ready.")
        self.is_waiting_for_response = False

    def timer_callback(self):
        if self.is_waiting_for_response is True:
            return

        user_prompt = input("You: ")
        self.req = GPTText.Request()
        self.req.request_text = user_prompt
        self.future = self.cli.call_async(self.req)
        self.future.add_done_callback(self.execution_callback)
        self.is_waiting_for_response = True
        # self.get_logger().info('Sent request: %s' % self.req.request_text)
        self.timer.reset()

    def execution_callback(self, future):
        try:
            response = future.result()
            print("\n" + "Mini Pupper: " + response.response_text + "\n")
        except Exception as e:
            self.get_logger().error("Service call failed: %s" % e)
        finally:
            self.is_waiting_for_response = False


def main(args=None):
    rclpy.init(args=args)
    gpt_client = GPTClient()
    rclpy.spin(gpt_client)
    gpt_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
