# Package
gpt_audio

# Description
This package provides two modules for handling bidirectional audio communication between a user and a robotic system using ROS2 and the OpenAI STT/TTS services.

# Components
The package contains two main components:

## 1. Audio_input.py:
A Python script that serves as the audio input node for the robotic system. It is responsible for recording audio, transcribing the recorded audio using the OpenAI STT service, and publishing the transcribed text to a ROS2 topic named `chat_llm_text_input`. Upon receiving an audio signal, the node records it for a specified duration, after which it amplifies the volume by a specified multiplier, saves the amplified audio to a file, transcribes it synchronously, the transcribed text is published to the `chat_llm_text_input`topic.

## 2. Audio_output.py
A Python script that serves as the audio output node for the robotic system. This node subscribes to a ROS2 topic named `chat_llm_text_output`. When the node receives a message containing text on this topic, it calls the OpenAI TTS service to synthesize speech from the input text. The synthesized speech is saved to a file named `/tmp/speech_output.mp3` and played using the `mpv` command. The node also sets GPT's status to `ROBOT_ACTION` once the speech playback is finished.

These two nodes together implement an audio-based interface for a robotic system, enabling users to communicate with the system using their voice and receive text messages translated to speech by the system in response.

