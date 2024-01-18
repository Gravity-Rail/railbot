#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from enum import Enum
import subprocess


class RailbotStatus(Enum):
    WAITING_USER_INPUT = 1
    STT_PROCESSING = 2
    CHAT_LLM_PROCESSING = 3
    TTS_PROCESSING = 4
    ROBOT_ACTION = 5


class RailbotStatusOperation:
    def get_railbot_status_value(self):
        cmd = "ros2 param get /railbot/railbot_param_server railbot_status"
        try:
            output = subprocess.check_output(cmd, shell=True, text=True)
            return output.split("String value is: ")[1].strip()
        except subprocess.CalledProcessError as e:
            print(f"Error when getting railbot_status: {e}")
            return None

    def set_railbot_status_value(self, railbot_status_value):
        command = f"ros2 param set /railbot/railbot_param_server railbot_status {railbot_status_value}"
        subprocess.run(command, shell=True, check=True, text=True)


class RailbotParamServer(Node):
    def __init__(self):
        super().__init__("param_server", namespace="railbot")
        self.railbot_status_value = (
            RailbotStatus.WAITING_USER_INPUT.name
        )  # Init status
        self.declare_parameter("railbot_status", self.railbot_status_value)
        self.create_timer(0.5, self.timer_callback)
        self.last_railbot_status_value = self.railbot_status_value

    def timer_callback(self):
        current_railbot_status_value = self.get_parameter("railbot_status").value
        if current_railbot_status_value != self.last_railbot_status_value:
            self.last_railbot_status_value = current_railbot_status_value
            self.get_logger().info(
                'GPT status: "%s"' % current_railbot_status_value
            )


def main(args=None):
    rclpy.init(args=args)
    railbot_param_server = RailbotParamServer()
    rclpy.spin(railbot_param_server)
    railbot_param_server.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
