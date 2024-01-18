# Package
openai_audio

# Description
This package provides two modules for handling bidirectional audio communication between a user and a robotic system using ROS2 and the OpenAI STT/TTS services.

# Components

## audio_input

Records audio for duration `config.duration` at sample rate `config.sample_rate`.

Publishes parsed text to the `chat_llm_text_input` topic.

## audio_output

Subscribes to the `chat_llm_text_output` topic.

Converts the text to speech using the OpenAI TTS service.

The synthesized speech is saved to a temporary file and played using the `mpv` command.

The node also sets Railbot's status to `ROBOT_ACTION` once the speech playback is finished.


