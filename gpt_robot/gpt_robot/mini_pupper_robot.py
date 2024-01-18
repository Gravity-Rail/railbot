#!/usr/bin/env python3

import rclpy
from time import sleep
from rclpy.node import Node
from geometry_msgs.msg import Pose

# RailBot
from railbot_status.railbot_param_server import RailbotStatus, RailbotStatusOperation
from railbot_status.railbot_config import RailbotConfig

# Other libraries
import os
from enum import Enum
import threading
import numpy as np

# Mini Pupper
from MangDang.mini_pupper.display import Display

config = RailbotConfig()

# For display
class BehaviorState(Enum):
    DEACTIVATED = -1
    REST = 0
    TROT = 1
    HOP = 2
    FINISHHOP = 3
    SHUTDOWN = 96
    IP = 97
    TEST = 98
    LOWBATTERY = 99

class MiniPupperRobot(Node):
    def __init__(self):
        super().__init__("mini_pupper_robot")
        # Publisher
        self.pose_publisher = self.create_publisher(Pose, "/body_pose", 10)
        # Timer
        self.create_timer(0.5, self.robot_behavior_callback)
        # GPT status
        self.gpt_operation = RailbotStatusOperation()
        self.gpt_current_status_value = (
            self.gpt_operation.get_railbot_status_value()
        )

        # Display
        self.display = Display()
        # Display
        self.display.show_state(BehaviorState.TROT)
        # Status parameter
        self.declare_parameter("real_hardware", False)  # default is False
        self.is_mini_pupper = self.get_parameter("real_hardware").value
        self.get_logger().info("GPT robot node started successfully.")

    def nod_head(self):
        self.get_logger().info("Nodding head...")
        interval = 0.02  # seconds
        duration = 0.25  # seconds
        num_values = int(duration / interval)

        angles = np.linspace(0, 0.1, num_values)
        for angle in angles:
            pose = Pose()
            pose.orientation.x = angle
            self.pose_publisher.publish(pose)
            sleep(interval)
        angles = np.linspace(0.1, 0, num_values)
        for angle in angles:
            pose = Pose()
            pose.orientation.x = angle
            self.pose_publisher.publish(pose)
            sleep(interval)

        angles = np.linspace(0, -0.1, num_values)
        for angle in angles:
            pose = Pose()
            pose.orientation.x = angle
            self.pose_publisher.publish(pose)
            sleep(interval)
        angles = np.linspace(-0.1, 0, num_values)
        for angle in angles:
            pose = Pose()
            pose.orientation.x = angle
            self.pose_publisher.publish(pose)
            sleep(interval)

        self.get_logger().info("Head is stopped.")

    def shake_head(self):
        self.get_logger().info("Shaking head...")
        interval = 0.02  # seconds
        duration = 0.25  # seconds
        num_values = int(duration / interval)

        angles = np.linspace(0, 0.1, num_values)
        for angle in angles:
            pose = Pose()
            pose.orientation.y = angle
            self.pose_publisher.publish(pose)
            sleep(interval)
        angles = np.linspace(0.1, 0, num_values)
        for angle in angles:
            pose = Pose()
            pose.orientation.y = angle
            self.pose_publisher.publish(pose)
            sleep(interval)

        angles = np.linspace(0, -0.1, num_values)
        for angle in angles:
            pose = Pose()
            pose.orientation.y = angle
            self.pose_publisher.publish(pose)
            sleep(interval)
        angles = np.linspace(-0.1, 0, num_values)
        for angle in angles:
            pose = Pose()
            pose.orientation.y = angle
            self.pose_publisher.publish(pose)
            sleep(interval)

        self.get_logger().info("Head is stopped.")

    def robot_behavior_callback(self):
        self.gpt_current_status_value = (
            self.gpt_operation.get_railbot_status_value()
        )
        if self.gpt_current_status_value == RailbotStatus.ROBOT_ACTION.name:
            shake_head_thread = threading.Thread(target=self.shake_head)
            shake_head_thread.start()
            # Display
            self.display.show_state(BehaviorState.TROT)

            self.gpt_operation.set_railbot_status_value(
                RailbotStatus.WAITING_USER_INPUT.name
            )
            self.get_logger().info("Returning to WAITING_USER_INPUT.")
        elif (
            self.gpt_current_status_value == RailbotStatus.WAITING_USER_INPUT.name
        ):
            # Display
            self.display.show_state(BehaviorState.TROT)

        elif (
            self.gpt_current_status_value
            == RailbotStatus.STT_PROCESSING.name
        ):
            # Display
            self.display.show_state(BehaviorState.HOP)
            play_music_thread = threading.Thread(target=self.play_music)
            play_music_thread.start()
            self.nod_head()  # Multi-threaded implementation
        elif self.gpt_current_status_value == RailbotStatus.CHAT_LLM_PROCESSING.name:
            # Display
            self.display.show_state(BehaviorState.HOP)
            play_music_thread = threading.Thread(target=self.play_music)
            play_music_thread.start()
            self.nod_head()  # Multi-threaded implementation
        elif (
            self.gpt_current_status_value
            == RailbotStatus.TTS_PROCESSING.name
        ):
            # Display
            self.display.show_state(BehaviorState.TROT)
        else:
            self.get_logger().info("No GPT Status input.")
            # Display
            self.display.show_state(BehaviorState.SHUTDOWN)

    def get_real_path(self):
        file_path = os.path.realpath(__file__)
        real_path = os.path.dirname(file_path)
        return real_path

    def play_music(self):
        self.get_logger().info("Playing music...")
        # Add music playing logic here
        music_file_name = "robot_music.mp3"
        music_file_path = os.path.join(self.get_real_path(), music_file_name)

        if self.is_mini_pupper:
            # os.system("mpv --audio-device=alsa/hw:1,0" + " " + music_file_path)
            os.system("mpv" + " " + music_file_path)
        else:
            os.system("mpv" + " " + music_file_path)

        self.get_logger().info("Music is stopped.")


def main(args=None):
    rclpy.init(args=args)
    gpt_robot = MiniPupperRobot()
    rclpy.spin(gpt_robot)
    gpt_robot.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
