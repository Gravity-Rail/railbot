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
# Defines a GPTConfig class to hold various configurations related to GPT and audio recording.
# Sets default values for GPT parameters such as model, API key, temperature, max tokens, and penalties.
# Defines system and user prompts, assistant response, and chat history as default configurations.
# Defines audio recording related configurations such as duration, sample rate, and volume gain multiplier.
#
# Author: Herman Ye

import os

class GPTConfig:
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
        self.chat_history = [{"role": "system", "content": self.system_prompt}]
        self.duration = 7  # Audio recording duration, in seconds
        self.sample_rate = 16000  # Sample rate
        self.volume_gain_multiplier = 1  # Change this to increase or decrease the volume
