#!/usr/bin/env python3

import os

class RailbotConfig:
    def __init__(self):
        self.model = "gpt-4-1106-preview"
        self.api_key = os.getenv("OPENAI_API_KEY", None)
        self.temperature = 1
        self.max_tokens = 1024
        self.top_p = 1
        self.frequency_penalty = 0
        self.presence_penalty = 0
        self.stop = ["\n", " You:", " Assistant:"]
        self.system_prompt = "You are a friendly robot dog."
        self.user_prompt = "default prompt"
        self.assistant_response = "default response"
        self.duration = 7  # Audio recording duration, in seconds
        self.sample_rate = 16000  # Sample rate
        self.volume_gain_multiplier = 1  # Change this to increase or decrease the volume
