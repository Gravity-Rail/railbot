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

class VirtualDisplay:
    def __init__(self, logger):
        self.state = BehaviorState.TROT
        self.logger = logger

    def show_state(self, state):
        self.logger.info("Display: " + state.name)
        self.state = state

# this function falls back to VirtualDisplay if the package `MangDang.mini_pupper.display` does not exist
def get_display(logger):
    try:
        # Mini Pupper
        from MangDang.mini_pupper.display import Display
        return Display()
    except ImportError:
        logger.info("Display: using virtual display")
        return VirtualDisplay(logger)

class MiniPupperRobot(Node):
    def __init__(self):
        super().__init__("mini_pupper_robot", namespace="railbot")
        # Publisher
        self.pose_publisher = self.create_publisher(Pose, "/body_pose", 10)
        # Timer
        self.create_timer(0.5, self.robot_behavior_callback)

        self.railbot_operation = RailbotStatusOperation()
        self.railbot_current_status_value = (
            self.railbot_operation.get_railbot_status_value()
        )

        self.display = get_display(self.get_logger())

        # Display
        self.display.show_state(BehaviorState.TROT)
        self.get_logger().info("Railbot Mini Pupper node started successfully.")

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
        self.railbot_current_status_value = (
            self.railbot_operation.get_railbot_status_value()
        )
        if self.railbot_current_status_value == RailbotStatus.ROBOT_ACTION.name:
            shake_head_thread = threading.Thread(target=self.shake_head)
            shake_head_thread.start()
            # Display
            self.display.show_state(BehaviorState.TROT)

            self.railbot_operation.set_railbot_status_value(
                RailbotStatus.WAITING_USER_INPUT.name
            )
            self.get_logger().info("Returning to WAITING_USER_INPUT.")
        elif (
            self.railbot_current_status_value == RailbotStatus.WAITING_USER_INPUT.name
        ):
            # Display
            self.display.show_state(BehaviorState.TROT)

        elif (
            self.railbot_current_status_value
            == RailbotStatus.STT_PROCESSING.name
        ):
            # Display
            self.display.show_state(BehaviorState.HOP)
            play_music_thread = threading.Thread(target=self.play_music)
            play_music_thread.start()
            self.nod_head()  # Multi-threaded implementation
        elif self.railbot_current_status_value == RailbotStatus.CHAT_LLM_PROCESSING.name:
            # Display
            self.display.show_state(BehaviorState.HOP)
            play_music_thread = threading.Thread(target=self.play_music)
            play_music_thread.start()
            self.nod_head()  # Multi-threaded implementation
        elif (
            self.railbot_current_status_value
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
        os.system("mpv" + " " + music_file_path)
        self.get_logger().info("Music is stopped.")


def main(args=None):
    rclpy.init(args=args)
    mini_pupper_robot = MiniPupperRobot()
    rclpy.spin(mini_pupper_robot)
    mini_pupper_robot.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
