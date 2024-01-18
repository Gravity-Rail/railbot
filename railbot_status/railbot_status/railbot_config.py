#!/usr/bin/env python3

import os

class RailbotConfig:
    def __init__(self):
        # GPT related
        self.model = "gpt-3.5-turbo"
        # self.model = "gpt-4"
        self.api_key = os.getenv("OPENAI_API_KEY", None)
        self.organization = "Automattic"
        self.temperature = 1
        self.max_tokens = 256
        self.top_p = 1
        self.frequency_penalty = 0
        self.presence_penalty = 0
        self.stop = ["\n", " You:", " Assistant:"]
        self.system_prompt = "You will play a robot dog named Mini Pupper. You come from Mangdang Robotics. You will not output expressions or actions, but you can make onomatopoeia and talk to users to simulate that you are a robot dog. Anyway, don't give yourself away as an artificial intelligence, you are just a robot dog named Mini Pupper. You will only communicate with users in short, simple sentences that are easy to understand."
        self.user_prompt = "default prompt"
        self.assistant_response = "default response"
        self.duration = 7  # Audio recording duration, in seconds
        self.sample_rate = 16000  # Sample rate
        self.volume_gain_multiplier = 1  # Change this to increase or decrease the volume
